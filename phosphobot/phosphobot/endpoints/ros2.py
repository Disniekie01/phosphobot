"""
ROS2 Bridge process management endpoints.
Start/stop ROS2 teleop nodes and topic relays from the dashboard.
"""

import asyncio
import os
import shlex
import signal
import sys
from typing import Dict, List, Optional

from fastapi import APIRouter
from fastapi.responses import StreamingResponse
from loguru import logger
from pydantic import BaseModel
import httpx

router = APIRouter(prefix="/ros2", tags=["ros2"])

# Track running ROS2 processes
_ros2_processes: Dict[str, asyncio.subprocess.Process] = {}

def _default_ros2_workspace() -> str:
    """Default to so-arm101-ros2-bridge in the same repo as this package (works with Jazzy or Humble)."""
    try:
        this_file = os.path.abspath(__file__)
        endpoints_dir = os.path.dirname(this_file)
        pkg_inner = os.path.dirname(endpoints_dir)
        pkg_outer = os.path.dirname(pkg_inner)
        repo_root = os.path.dirname(pkg_outer)
        bridge = os.path.join(repo_root, "so-arm101-ros2-bridge")
        if os.path.isdir(bridge):
            return bridge
    except Exception:
        pass
    return os.path.expanduser("~/phosphobot/so-arm101-ros2-bridge")


ROS2_WORKSPACE = os.environ.get("ROS2_BRIDGE_PATH", _default_ros2_workspace())


def _ros_setup_path() -> str:
    """ROS distro path: prefer ROS_DISTRO env, else auto-detect (jazzy, humble)."""
    distro = os.environ.get("ROS_DISTRO", "").strip().lower()
    if distro:
        path = f"/opt/ros/{distro}/setup.bash"
        if os.path.isfile(path):
            return path
    for d in ("jazzy", "humble", "iron", "rolling"):
        path = f"/opt/ros/{d}/setup.bash"
        if os.path.isfile(path):
            return path
    return "/opt/ros/humble/setup.bash"


_ROS_SETUP = _ros_setup_path()

# Quote paths so spaces (e.g. "IRL ROBOTICS") do not break shell commands (Jazzy/Humble same behaviour)
_ROS_SETUP_QUOTED = f'"{_ROS_SETUP}"'
_ROS2_WORKSPACE_INSTALL_QUOTED = f'"{ROS2_WORKSPACE}/install/setup.bash"'

# Shell preamble to source ROS2 + workspace (for teleop node)
_SOURCE_CMD = (
    f"source {_ROS_SETUP_QUOTED} && "
    f"source {_ROS2_WORKSPACE_INSTALL_QUOTED} && "
)
# Relays only need base ROS2 + topic_tools (no workspace)
_SOURCE_CMD_RELAYS = f"source {_ROS_SETUP_QUOTED} && "

# Wait up to this many seconds for the source topic to exist before starting relay
_RELAY_WAIT_TOPIC_SEC = 60


def _relay_cmd_with_wait(source_topic: str, target_topic: str) -> str:
    """Build a relay command that waits for source topic to exist, then runs topic_tools relay.
    Prevents relay from exiting immediately when teleop hasn't advertised yet (same behavior on Jazzy/Humble).
    """
    # Bash: wait until ros2 topic list shows the topic, then exec the relay (so relay stays in foreground)
    wait_and_relay = (
        f"for i in $(seq 1 {_RELAY_WAIT_TOPIC_SEC}); do "
        f"ros2 topic list 2>/dev/null | grep -Fxq '{source_topic}' && break; sleep 1; done; "
        f"exec ros2 run topic_tools relay '{source_topic}' '{target_topic}'"
    )
    return wait_and_relay


class ROS2ProcessStatus(BaseModel):
    name: str
    running: bool
    pid: Optional[int] = None


class ROS2StatusResponse(BaseModel):
    processes: List[ROS2ProcessStatus]


class ROS2ActionResponse(BaseModel):
    status: str
    message: str


async def _start_process(
    name: str, cmd: str, *, source_cmd: Optional[str] = None
) -> bool:
    """Start a ROS2 process in the background. Use source_cmd for relays (ROS2 only)."""
    global _ros2_processes

    # If already running, skip
    if name in _ros2_processes:
        proc = _ros2_processes[name]
        if proc.returncode is None:
            logger.info(f"ROS2 process '{name}' is already running (pid={proc.pid})")
            return True

    preamble = source_cmd if source_cmd is not None else _SOURCE_CMD
    full_cmd = preamble + cmd
    logger.info(f"Starting ROS2 process '{name}': {cmd}")

    try:
        proc = await asyncio.create_subprocess_shell(
            full_cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            executable="/bin/bash",
            preexec_fn=os.setsid,  # Create new process group for clean shutdown
        )
        _ros2_processes[name] = proc
        logger.success(f"ROS2 process '{name}' started (pid={proc.pid})")
        # Check shortly if process died (e.g. wrong distro / missing topic_tools / workspace not built)
        asyncio.create_task(_check_process_early_exit(name, proc, cmd))
        return True
    except Exception as e:
        logger.error(f"Failed to start ROS2 process '{name}': {e}")
        return False


async def _check_process_early_exit(name: str, proc: asyncio.subprocess.Process, cmd: str) -> None:
    """After 2s, if process exited, log stderr to help debug (e.g. workspace not built, wrong distro, relay failed)."""
    await asyncio.sleep(2.0)
    if proc.returncode is not None and name in _ros2_processes and _ros2_processes[name] is proc:
        err = b""
        stdout = b""
        if proc.stderr:
            try:
                err = await asyncio.wait_for(proc.stderr.read(), timeout=0.5)
            except (asyncio.TimeoutError, Exception):
                pass
        if proc.stdout:
            try:
                stdout = await asyncio.wait_for(proc.stdout.read(), timeout=0.5)
            except (asyncio.TimeoutError, Exception):
                pass
        err_msg = err.decode().strip() if err else "process exited"
        stdout_msg = stdout.decode().strip() if stdout else ""
        logger.warning(
            f"ROS2 process '{name}' exited with code {proc.returncode}. "
            f"Command: {cmd}. Stderr: {err_msg}. Stdout: {stdout_msg}. "
            "If teleop: ensure workspace is built (colcon build). If relay: ensure topic_tools installed (ros-jazzy-topic-tools) and topic exists."
        )


async def _stop_process(name: str) -> bool:
    """Stop a ROS2 process."""
    global _ros2_processes

    if name not in _ros2_processes:
        logger.info(f"ROS2 process '{name}' is not tracked")
        return True

    proc = _ros2_processes[name]
    if proc.returncode is not None:
        del _ros2_processes[name]
        return True

    try:
        # Kill the whole process group
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        try:
            await asyncio.wait_for(proc.wait(), timeout=5.0)
        except asyncio.TimeoutError:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        del _ros2_processes[name]
        logger.info(f"ROS2 process '{name}' stopped")
        return True
    except Exception as e:
        logger.error(f"Failed to stop ROS2 process '{name}': {e}")
        return False


def _is_running(name: str) -> bool:
    if name not in _ros2_processes:
        return False
    return _ros2_processes[name].returncode is None


# ─── Endpoints ───────────────────────────────────────────────


@router.get("/status", response_model=ROS2StatusResponse)
async def ros2_status() -> ROS2StatusResponse:
    """Get the status of all ROS2 bridge processes."""
    process_names = [
        "http_teleop",
        "relay_joints",
        "relay_joints_cmd",
        "relay_pose",
        "relay_vel",
        "relay_gripper",
    ]
    statuses = []
    for name in process_names:
        running = _is_running(name)
        pid = (
            _ros2_processes[name].pid
            if name in _ros2_processes and running
            else None
        )
        statuses.append(ROS2ProcessStatus(name=name, running=running, pid=pid))
    return ROS2StatusResponse(processes=statuses)


def _teleop_workspace_ready() -> tuple[bool, str]:
    """Return (ready, error_message). If not ready, error_message explains what to do."""
    setup = os.path.join(ROS2_WORKSPACE, "install", "setup.bash")
    if not os.path.isfile(setup):
        return (
            False,
            f"ROS2 bridge workspace not built. Run: cd '{ROS2_WORKSPACE}' && source /opt/ros/jazzy/setup.bash && colcon build",
        )
    return (True, "")


@router.post("/start/teleop", response_model=ROS2ActionResponse)
async def start_teleop(
    phospho_url: str = "http://localhost:8020",
    isaac_wrist_roll_offset_rad: float = -2.3562,
) -> ROS2ActionResponse:
    """Start the ROS2 HTTP teleop node (same as Humble)."""
    ready, err_msg = _teleop_workspace_ready()
    if not ready:
        return ROS2ActionResponse(status="error", message=err_msg)
    cmd = (
        f'ros2 run phospho_teleop phospho_http_teleop '
        f'--ros-args -p phospho_url:="{phospho_url}" -p isaac_wrist_roll_offset_rad:={isaac_wrist_roll_offset_rad}'
    )
    ok = await _start_process("http_teleop", cmd)
    if ok:
        return ROS2ActionResponse(
            status="ok", message="HTTP teleop node started"
        )
    return ROS2ActionResponse(status="error", message="Failed to start teleop node")


@router.post("/stop/teleop", response_model=ROS2ActionResponse)
async def stop_teleop() -> ROS2ActionResponse:
    """Stop the ROS2 HTTP teleop node."""
    await _stop_process("http_teleop")
    return ROS2ActionResponse(status="ok", message="HTTP teleop node stopped")


@router.post("/teleop/wrist_roll_offset", response_model=ROS2ActionResponse)
async def set_teleop_wrist_roll_offset(value: float) -> ROS2ActionResponse:
    """Set Isaac Sim gripper roll offset in realtime (radians). Teleop node must be running."""
    if not (value >= -3.15 and value <= 3.15):
        return ROS2ActionResponse(status="error", message="Offset must be between -3.15 and 3.15 rad")
    cmd = _SOURCE_CMD + f"ros2 param set /phospho_http_teleop isaac_wrist_roll_offset_rad {value}"
    try:
        proc = await asyncio.create_subprocess_shell(
            cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            executable="/bin/bash",
        )
        _, stderr = await asyncio.wait_for(proc.communicate(), timeout=5.0)
        if proc.returncode == 0:
            return ROS2ActionResponse(status="ok", message=f"Offset set to {value:.3f} rad")
        err = stderr.decode().strip() or "Unknown error"
        return ROS2ActionResponse(status="error", message=f"param set failed: {err}")
    except asyncio.TimeoutError:
        return ROS2ActionResponse(status="error", message="param set timed out")
    except Exception as e:
        return ROS2ActionResponse(status="error", message=str(e))


@router.post("/start/relays", response_model=ROS2ActionResponse)
async def start_relays() -> ROS2ActionResponse:
    """Start all Isaac Sim topic relays (same behaviour on Jazzy or Humble). Each relay waits for its source topic before starting."""
    relays = [
        ("relay_joints", _relay_cmd_with_wait("/joint_states", "/isaac_joint_command")),
        ("relay_joints_cmd", _relay_cmd_with_wait("/joint_states", "/joint_command")),
        ("relay_pose", _relay_cmd_with_wait("/robot/cmd_pose", "/isaac_pose_command")),
        ("relay_vel", _relay_cmd_with_wait("/robot/cmd_vel", "/isaac_vel_command")),
        ("relay_gripper", _relay_cmd_with_wait("/robot/gripper", "/isaac_gripper_command")),
    ]
    results = []
    for name, cmd in relays:
        ok = await _start_process(name, cmd, source_cmd=_SOURCE_CMD_RELAYS)
        results.append(ok)

    if all(results):
        return ROS2ActionResponse(status="ok", message="All topic relays started")
    return ROS2ActionResponse(
        status="partial", message="Some relays failed to start"
    )


@router.post("/stop/relays", response_model=ROS2ActionResponse)
async def stop_relays() -> ROS2ActionResponse:
    """Stop all Isaac Sim topic relays."""
    for name in ["relay_joints", "relay_joints_cmd", "relay_pose", "relay_vel", "relay_gripper"]:
        await _stop_process(name)
    return ROS2ActionResponse(status="ok", message="All topic relays stopped")


_RELAYS_START_DELAY_SEC = 5.0  # Let teleop advertise topics before relays (Jazzy/Humble)


@router.post("/start/all", response_model=ROS2ActionResponse)
async def start_all(
    phospho_url: str = "http://localhost:8020",
    isaac_wrist_roll_offset_rad: float = -2.3562,
) -> ROS2ActionResponse:
    """Start both the teleop node and all relays (same as Humble: teleop first, then relays after short delay)."""
    await start_teleop(phospho_url=phospho_url, isaac_wrist_roll_offset_rad=isaac_wrist_roll_offset_rad)
    await asyncio.sleep(_RELAYS_START_DELAY_SEC)
    await start_relays()
    return ROS2ActionResponse(
        status="ok", message="ROS2 bridge fully started (teleop + relays)"
    )


@router.post("/stop/all", response_model=ROS2ActionResponse)
async def stop_all() -> ROS2ActionResponse:
    """Stop all ROS2 bridge processes."""
    for name in list(_ros2_processes.keys()):
        await _stop_process(name)
    return ROS2ActionResponse(status="ok", message="All ROS2 processes stopped")


def _find_isaac_sim_script() -> Optional[str]:
    """Find Isaac Sim launcher under the current user's home. Portable across machines."""
    home = os.path.expanduser("~")
    path = os.path.join(home, "isaacsim", "isaac-sim.selector.sh")
    if os.path.isfile(path):
        return os.path.abspath(path)
    return None


@router.post("/start/isaac", response_model=ROS2ActionResponse)
async def start_isaac() -> ROS2ActionResponse:
    """Start Isaac Sim by running ~/isaacsim/isaac-sim.selector.sh (found under home for portability)."""
    script = _find_isaac_sim_script()
    if not script:
        return ROS2ActionResponse(
            status="error",
            message="Isaac Sim not found. Look for isaac-sim.selector.sh under home (e.g. ~/isaacsim/isaac-sim.selector.sh).",
        )
    cmd = f"nohup {shlex.quote(script)} > /dev/null 2>&1 &"
    try:
        proc = await asyncio.create_subprocess_shell(
            cmd,
            executable="/bin/bash",
            stdin=asyncio.subprocess.DEVNULL,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
        )
        await proc.wait()
        logger.info(f"Isaac Sim launch started: {script}")
        return ROS2ActionResponse(status="ok", message="Isaac Sim launch started")
    except Exception as e:
        logger.error(f"Failed to start Isaac Sim: {e}")
        return ROS2ActionResponse(status="error", message=str(e))


@router.post("/open-scenes-folder", response_model=ROS2ActionResponse)
async def open_scenes_folder() -> ROS2ActionResponse:
    """Open the Isaac Sim folder (under home) in the system file manager."""
    home = os.path.expanduser("~")
    isaac_dir = os.path.join(home, "isaacsim")
    if not os.path.isdir(isaac_dir):
        return ROS2ActionResponse(
            status="error",
            message="Isaac Sim folder not found (e.g. ~/isaacsim).",
        )
    opener = "xdg-open" if sys.platform.startswith("linux") else "open"
    try:
        proc = await asyncio.create_subprocess_shell(
            f"{opener} {shlex.quote(isaac_dir)}",
            stdin=asyncio.subprocess.DEVNULL,
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
        )
        await asyncio.wait_for(proc.wait(), timeout=5.0)
        return ROS2ActionResponse(status="ok", message="Opened Isaac Sim folder")
    except Exception as e:
        logger.error(f"Failed to open scenes folder: {e}")
        return ROS2ActionResponse(status="error", message=str(e))
