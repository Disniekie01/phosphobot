"""
ROS2 Bridge process management endpoints.
Start/stop ROS2 teleop nodes and topic relays from the dashboard.
"""

import asyncio
import os
import signal
from typing import Dict, List, Optional

from fastapi import APIRouter
from loguru import logger
from pydantic import BaseModel

router = APIRouter(prefix="/ros2", tags=["ros2"])

# Track running ROS2 processes
_ros2_processes: Dict[str, asyncio.subprocess.Process] = {}

ROS2_WORKSPACE = os.environ.get(
    "ROS2_BRIDGE_PATH",
    os.path.expanduser("~/phosphobot/so-arm101-ros2-bridge"),
)

# Shell preamble to source ROS2 + workspace
_SOURCE_CMD = (
    "source /opt/ros/humble/setup.bash && "
    f"source {ROS2_WORKSPACE}/install/setup.bash && "
)


class ROS2ProcessStatus(BaseModel):
    name: str
    running: bool
    pid: Optional[int] = None


class ROS2StatusResponse(BaseModel):
    processes: List[ROS2ProcessStatus]


class ROS2ActionResponse(BaseModel):
    status: str
    message: str


async def _start_process(name: str, cmd: str) -> bool:
    """Start a ROS2 process in the background."""
    global _ros2_processes

    # If already running, skip
    if name in _ros2_processes:
        proc = _ros2_processes[name]
        if proc.returncode is None:
            logger.info(f"ROS2 process '{name}' is already running (pid={proc.pid})")
            return True

    full_cmd = _SOURCE_CMD + cmd
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
        return True
    except Exception as e:
        logger.error(f"Failed to start ROS2 process '{name}': {e}")
        return False


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


@router.post("/start/teleop", response_model=ROS2ActionResponse)
async def start_teleop(
    phospho_url: str = "http://localhost:8020",
) -> ROS2ActionResponse:
    """Start the ROS2 HTTP teleop node."""
    cmd = (
        f'ros2 run phospho_teleop phospho_http_teleop '
        f'--ros-args -p phospho_url:="{phospho_url}"'
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


@router.post("/start/relays", response_model=ROS2ActionResponse)
async def start_relays() -> ROS2ActionResponse:
    """Start all Isaac Sim topic relays."""
    relays = [
        ("relay_joints", "ros2 run topic_tools relay /joint_states /isaac_joint_command"),
        ("relay_pose", "ros2 run topic_tools relay /robot/cmd_pose /isaac_pose_command"),
        ("relay_vel", "ros2 run topic_tools relay /robot/cmd_vel /isaac_vel_command"),
        ("relay_gripper", "ros2 run topic_tools relay /robot/gripper /isaac_gripper_command"),
    ]
    results = []
    for name, cmd in relays:
        ok = await _start_process(name, cmd)
        results.append(ok)

    if all(results):
        return ROS2ActionResponse(status="ok", message="All topic relays started")
    return ROS2ActionResponse(
        status="partial", message="Some relays failed to start"
    )


@router.post("/stop/relays", response_model=ROS2ActionResponse)
async def stop_relays() -> ROS2ActionResponse:
    """Stop all Isaac Sim topic relays."""
    for name in ["relay_joints", "relay_pose", "relay_vel", "relay_gripper"]:
        await _stop_process(name)
    return ROS2ActionResponse(status="ok", message="All topic relays stopped")


@router.post("/start/all", response_model=ROS2ActionResponse)
async def start_all(
    phospho_url: str = "http://localhost:8020",
) -> ROS2ActionResponse:
    """Start both the teleop node and all relays."""
    await start_teleop(phospho_url=phospho_url)
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
