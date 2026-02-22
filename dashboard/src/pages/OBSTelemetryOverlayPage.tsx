/**
 * OBS Browser Source overlay: telemetry + joint state sliders + camera feed.
 * URL: https://your-phospho-host/obs
 * In OBS: Add Source → Browser → URL. Resize to taste (e.g. 960×320).
 */
import { useEffect, useState } from "react";
import { Slider } from "@/components/ui/slider";
import type { ServerStatus } from "@/types";

const REFRESH_MS = 500;
const JOINT_NAMES = ["J0", "J1", "J2", "J3", "J4", "J5"];
const RAD_MIN = -3.15;
const RAD_MAX = 3.15;
const CAMERA_QUALITY = 70;
const CAMERA_WIDTH = 280;
const CAMERA_HEIGHT = 158;

interface JointsResponse {
  angles: (number | null)[];
  unit: string;
}

function buildCameraStreamUrl(cameraId: number) {
  return `/video/${cameraId}?quality=${CAMERA_QUALITY}&width=${CAMERA_WIDTH}&height=${CAMERA_HEIGHT}`;
}

export function OBSTelemetryOverlayPage() {
  const [status, setStatus] = useState<ServerStatus | null>(null);
  const [joints, setJoints] = useState<(number | null)[]>([]);
  const [error, setError] = useState(false);
  const [selectedCameraId, setSelectedCameraId] = useState<number>(0);
  const [cameraError, setCameraError] = useState(false);

  const cameraIds = status?.cameras?.video_cameras_ids ?? [];
  const effectiveCameraId = cameraIds.includes(selectedCameraId)
    ? selectedCameraId
    : cameraIds[0] ?? 0;

  useEffect(() => {
    const fetchData = async () => {
      try {
        const [statusRes, jointsRes] = await Promise.all([
          fetch("/status"),
          fetch("/joints/read?robot_id=0", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ unit: "rad", source: "robot" }),
          }),
        ]);
        if (statusRes.ok) {
          setStatus(await statusRes.json());
          setError(false);
        } else {
          setError(true);
        }
        if (jointsRes.ok) {
          const data: JointsResponse = await jointsRes.json();
          setJoints(Array.isArray(data.angles) ? data.angles : []);
        } else {
          setJoints([]);
        }
      } catch {
        setError(true);
        setJoints([]);
      }
    };
    fetchData();
    const id = setInterval(fetchData, REFRESH_MS);
    return () => clearInterval(id);
  }, []);

  return (
    <>
      <style>{`
        @keyframes obs-gradient-shift {
          0%, 100% { opacity: 1; }
          50% { opacity: 0.92; }
        }
        @keyframes obs-glow-pulse {
          0%, 100% { box-shadow: 0 8px 32px rgba(0,0,0,0.4), 0 0 0 1px rgba(255,255,255,0.06) inset; }
          50% { box-shadow: 0 8px 40px rgba(0,0,0,0.5), 0 0 20px rgba(59, 130, 246, 0.08), 0 0 0 1px rgba(255,255,255,0.08) inset; }
        }
        .obs-panel-bg {
          animation: obs-gradient-shift 8s ease-in-out infinite;
        }
        .obs-panel-glass {
          animation: obs-glow-pulse 6s ease-in-out infinite;
        }
      `}</style>
      <div
        className="min-h-screen w-full flex flex-col justify-end items-center p-4 font-sans"
        style={{
          background: "transparent",
          boxSizing: "border-box",
        }}
      >
        {/* Panel: wide, animated glass, content + camera */}
        <div
          className="w-full rounded-2xl border flex flex-row gap-6 p-6 shadow-2xl max-w-[1100px] min-w-[720px] obs-panel-bg obs-panel-glass"
          style={{
            background: "linear-gradient(160deg, rgba(20, 28, 45, 0.9) 0%, rgba(30, 41, 59, 0.82) 50%, rgba(15, 23, 42, 0.92) 100%)",
            borderColor: "rgba(148, 163, 184, 0.35)",
            backdropFilter: "blur(12px)",
            boxShadow: "0 8px 32px rgba(0,0,0,0.4), 0 0 0 1px rgba(255,255,255,0.06) inset",
          }}
        >
          {/* Left: status + joints */}
          <div className="flex flex-1 flex-col gap-5 min-w-0">
            {error && (
              <div className="text-red-400 text-base font-medium">No connection to server</div>
            )}

            {status && !error && (
              <>
                <div className="flex flex-wrap items-center gap-x-6 gap-y-2 text-base">
                  <span className="text-slate-100 font-bold tracking-wide text-lg">
                    {status.name}
                  </span>
                  <span className="inline-flex items-center gap-2">
                    <span
                      className={`inline-block w-3 h-3 rounded-full shrink-0 ${
                        status.is_recording ? "bg-red-500 shadow-[0_0_10px_rgba(239,68,68,0.9)]" : "bg-slate-500"
                      }`}
                    />
                    <span className="text-slate-400">REC</span>
                    <span className={status.is_recording ? "text-red-400 font-bold" : "text-slate-500"}>
                      {status.is_recording ? "ON" : "OFF"}
                    </span>
                  </span>
                  <span className="text-slate-400">
                    Robots{" "}
                    <span className="text-slate-100 font-semibold">{status.robots?.length ?? 0}</span>
                    {status.robots?.length ? (
                      <span className="text-slate-500 ml-1">({status.robots.join(", ")})</span>
                    ) : null}
                  </span>
                  <span className="text-slate-400">
                    AI{" "}
                    <span className="text-slate-100 font-semibold">{status.ai_running_status ?? "—"}</span>
                  </span>
                  <span className="text-slate-400">
                    L/F{" "}
                    <span className={status.leader_follower_status ? "text-emerald-400 font-semibold" : "text-slate-500"}>
                      {status.leader_follower_status ? "ON" : "OFF"}
                    </span>
                  </span>
                </div>

                {joints.length > 0 && (
                  <div className="space-y-3 pt-3 border-t border-slate-500/40">
                    <div className="text-sm text-slate-400 font-semibold uppercase tracking-widest">
                      Joint state (rad)
                    </div>
                    <div className="grid gap-4" style={{ gridTemplateColumns: `repeat(${joints.length}, minmax(0,1fr))` }}>
                      {joints.slice(0, 6).map((angle, i) => (
                        <div key={i} className="flex flex-col gap-1.5 min-w-0">
                          <div className="flex justify-between items-baseline text-sm">
                            <span className="text-slate-400 font-medium">{JOINT_NAMES[i] ?? `J${i}`}</span>
                            <span className="text-slate-300 tabular-nums font-mono text-base">
                              {angle != null ? angle.toFixed(2) : "—"}
                            </span>
                          </div>
                          <Slider
                            value={[angle != null ? Math.max(RAD_MIN, Math.min(RAD_MAX, angle)) : 0]}
                            min={RAD_MIN}
                            max={RAD_MAX}
                            step={0.01}
                            disabled
                            className="opacity-95 pointer-events-none h-3"
                          />
                        </div>
                      ))}
                    </div>
                  </div>
                )}
              </>
            )}
          </div>

          {/* Right: camera selector + feed */}
          <div className="shrink-0 flex flex-col gap-2">
            {cameraIds.length > 0 && (
              <select
                value={effectiveCameraId}
                onChange={(e) => {
                  setSelectedCameraId(Number(e.target.value));
                  setCameraError(false);
                }}
                className="w-full rounded-lg border border-slate-500/40 bg-slate-800/80 text-slate-200 text-sm font-medium px-3 py-1.5 focus:outline-none focus:ring-2 focus:ring-slate-400/50"
              >
                {cameraIds.map((id) => (
                  <option key={id} value={id}>
                    Camera {id}
                  </option>
                ))}
              </select>
            )}
            <div
              className="rounded-xl overflow-hidden border border-slate-500/40 bg-slate-900/60"
              style={{ width: CAMERA_WIDTH, height: CAMERA_HEIGHT }}
            >
              {cameraError ? (
                <div className="w-full h-full flex items-center justify-center text-slate-500 text-sm">
                  No camera
                </div>
              ) : (
                <img
                  key={effectiveCameraId}
                  src={buildCameraStreamUrl(effectiveCameraId)}
                  alt={`Camera ${effectiveCameraId}`}
                  className="w-full h-full object-cover"
                  onError={() => setCameraError(true)}
                />
              )}
            </div>
          </div>
        </div>
      </div>
    </>
  );
}
