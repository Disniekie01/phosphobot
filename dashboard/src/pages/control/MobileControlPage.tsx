import { LoadingPage } from "@/components/common/loading";
import { Button } from "@/components/ui/button";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";
import { fetcher } from "@/lib/utils";
import type { ServerStatus } from "@/types";
import {
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  ArrowUp,
  Hand,
  HandMetal,
  Smartphone,
} from "lucide-react";
import { useCallback, useEffect, useRef, useState } from "react";
import useSWR from "swr";

const BASE_URL = `http://${window.location.hostname}:${window.location.port}/`;

// Step size: 1 cm position
const STEP_CM = 1;

// Tilt control: dead zone (degrees), scale (cm per 10° tilt), send interval (ms)
const TILT_DEAD_ZONE = 8;
const TILT_SCALE_CM = 0.15; // cm per degree beyond dead zone
const TILT_SEND_INTERVAL_MS = 250;

type ControlMode = "buttons" | "tilt";

export function MobileControlPage() {
  const { data: serverStatus, error: serverError } = useSWR<ServerStatus>(
    ["/status"],
    fetcher,
    { refreshInterval: 5000 },
  );

  const [selectedRobotName, setSelectedRobotName] = useState<string | null>(null);
  const [sending, setSending] = useState(false);
  const [controlMode, setControlMode] = useState<ControlMode>("buttons");
  const [gyroEnabled, setGyroEnabled] = useState(false);
  const [gyroError, setGyroError] = useState<string | null>(null);
  const [tilt, setTilt] = useState<{ beta: number; gamma: number } | null>(null);
  const lastSendRef = useRef<number>(0);
  const tiltRef = useRef<{ beta: number; gamma: number }>({ beta: 0, gamma: 0 });

  const robotIDFromName = (name: string | null | undefined) => {
    if (name == null || !serverStatus?.robot_status) return 0;
    const match = name.match(/^robot-(\d+)$/);
    if (match) return parseInt(match[1], 10);
    const index = serverStatus.robot_status.findIndex(
      (robot) => robot.device_name === name,
    );
    return index === -1 ? 0 : index;
  };

  const sendMove = useCallback(
    async (
      payload: {
        x?: number;
        y?: number;
        z?: number;
        rx?: number;
        ry?: number;
        rz?: number;
        open?: number;
      },
      silent = false,
    ) => {
      if (!selectedRobotName) return;
      if (!silent) setSending(true);
      try {
        const robotId = robotIDFromName(selectedRobotName);
        const response = await fetch(
          `${BASE_URL}move/relative?robot_id=${robotId}`,
          {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(payload),
          },
        );
        if (!response.ok) throw new Error(response.statusText);
      } catch (e) {
        console.error("Mobile move failed:", e);
      } finally {
        if (!silent) setSending(false);
      }
    },
    [selectedRobotName, serverStatus],
  );

  // Request motion/orientation permission (required on iOS 13+)
  const requestGyroPermission = useCallback(async () => {
    setGyroError(null);
    const req =
      typeof (DeviceOrientationEvent as unknown as { requestPermission?: () => Promise<string> })
        .requestPermission === "function"
        ? (DeviceOrientationEvent as unknown as { requestPermission: () => Promise<string> })
            .requestPermission()
        : null;
    if (req) {
      try {
        const state = await req;
        if (state !== "granted") {
          setGyroError("Permission denied. Enable motion access in Settings.");
          return;
        }
      } catch (e) {
        setGyroError(
          e instanceof Error ? e.message : "Could not request permission",
        );
        return;
      }
    }
    setGyroEnabled(true);
  }, []);

  // Tilt-based control: listen to deviceorientation and throttle-send moves
  useEffect(() => {
    if (!gyroEnabled || controlMode !== "tilt" || !selectedRobotName) return;

    const onOrientation = (e: DeviceOrientationEvent) => {
      const beta = e.beta ?? 0; // front-back tilt
      const gamma = e.gamma ?? 0; // left-right tilt
      tiltRef.current = { beta, gamma };
      setTilt({ beta, gamma });
    };

    window.addEventListener("deviceorientation", onOrientation, true);

    const interval = setInterval(() => {
      const { beta, gamma } = tiltRef.current;
      const now = Date.now();
      if (now - lastSendRef.current < TILT_SEND_INTERVAL_MS) return;

      const absBeta = Math.abs(beta);
      const absGamma = Math.abs(gamma);
      if (absBeta < TILT_DEAD_ZONE && absGamma < TILT_DEAD_ZONE) return;

      const signBeta = beta > 0 ? 1 : -1;
      const signGamma = gamma > 0 ? 1 : -1;
      const x =
        absBeta > TILT_DEAD_ZONE
          ? signBeta * (absBeta - TILT_DEAD_ZONE) * TILT_SCALE_CM
          : undefined;
      const y =
        absGamma > TILT_DEAD_ZONE
          ? -signGamma * (absGamma - TILT_DEAD_ZONE) * TILT_SCALE_CM
          : undefined;
      if (x === undefined && y === undefined) return;

      lastSendRef.current = now;
      sendMove({ x, y }, true);
    }, TILT_SEND_INTERVAL_MS);

    return () => {
      window.removeEventListener("deviceorientation", onOrientation, true);
      clearInterval(interval);
    };
  }, [gyroEnabled, controlMode, selectedRobotName, sendMove]);

  if (serverError) {
    return (
      <div className="p-4 text-destructive">
        Failed to load. Make sure the app is running and you’re on the same
        network.
      </div>
    );
  }
  if (!serverStatus) return <LoadingPage />;

  const robots = serverStatus.robot_status || [];

  return (
    <div className="min-h-[60vh] flex flex-col items-center justify-center p-4 pb-8">
      <h1 className="text-xl font-semibold mb-2">Phone control</h1>
      <p className="text-sm text-muted-foreground mb-4">
        {controlMode === "buttons"
          ? "Tap a button to send one move."
          : "Tilt your phone to move the robot (X/Y)."}
        {" "}Use same Wi‑Fi as the robot.
      </p>

      <div className="flex gap-2 mb-4">
        <Button
          variant={controlMode === "buttons" ? "default" : "outline"}
          size="sm"
          onClick={() => setControlMode("buttons")}
        >
          Buttons
        </Button>
        <Button
          variant={controlMode === "tilt" ? "default" : "outline"}
          size="sm"
          onClick={() => setControlMode("tilt")}
        >
          <Smartphone className="h-4 w-4 mr-1" />
          Tilt / Gyro
        </Button>
      </div>

      <div className="w-full max-w-xs space-y-4">
        <div className="space-y-1">
          <label className="text-sm font-medium">Robot</label>
          <Select
            value={selectedRobotName ?? ""}
            onValueChange={(v) => setSelectedRobotName(v || null)}
          >
            <SelectTrigger className="h-12 text-base">
              <SelectValue placeholder="Select robot" />
            </SelectTrigger>
            <SelectContent>
              {robots.map((robot, index) => (
                <SelectItem
                  key={robot.device_name || `robot-${index}`}
                  value={robot.device_name || `robot-${index}`}
                >
                  {robot.name} ({robot.device_name || "—"})
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>

        {controlMode === "tilt" && (
          <div className="space-y-3 rounded-lg border p-4 bg-muted/50">
            {!gyroEnabled ? (
              <>
                <p className="text-sm text-muted-foreground">
                  Use your phone’s motion sensors. On iPhone you’ll be asked for
                  permission.
                </p>
                <Button
                  className="w-full touch-manipulation"
                  size="lg"
                  disabled={!selectedRobotName}
                  onClick={requestGyroPermission}
                >
                  Enable motion / gyro
                </Button>
                {gyroError && (
                  <p className="text-sm text-destructive">{gyroError}</p>
                )}
              </>
            ) : (
              <>
                <p className="text-sm font-medium">Tilt = move (X/Y)</p>
                {tilt !== null ? (
                  <p className="text-xs text-muted-foreground font-mono">
                    β {tilt.beta.toFixed(0)}° γ {tilt.gamma.toFixed(0)}°
                  </p>
                ) : (
                  <p className="text-xs text-muted-foreground">
                    Waiting for sensor…
                  </p>
                )}
              </>
            )}
          </div>
        )}

        {controlMode === "buttons" && (
        <div className="grid grid-cols-3 gap-3">
          {/* Row 1: Forward (X+) center */}
          <div />
          <Button
            size="lg"
            variant="outline"
            className="h-16 text-lg touch-manipulation"
            disabled={!selectedRobotName || sending}
            onClick={() => sendMove({ x: STEP_CM })}
          >
            <ArrowUp className="h-6 w-6" />
            <span className="sr-only">Forward (X+)</span>
          </Button>
          <div />

          {/* Row 2: Left, Back, Right */}
          <Button
            size="lg"
            variant="outline"
            className="h-16 text-lg touch-manipulation"
            disabled={!selectedRobotName || sending}
            onClick={() => sendMove({ y: STEP_CM })}
          >
            <ArrowLeft className="h-6 w-6" />
            <span className="sr-only">Left (Y+)</span>
          </Button>
          <Button
            size="lg"
            variant="outline"
            className="h-16 text-lg touch-manipulation"
            disabled={!selectedRobotName || sending}
            onClick={() => sendMove({ x: -STEP_CM })}
          >
            <ArrowDown className="h-6 w-6" />
            <span className="sr-only">Back (X-)</span>
          </Button>
          <Button
            size="lg"
            variant="outline"
            className="h-16 text-lg touch-manipulation"
            disabled={!selectedRobotName || sending}
            onClick={() => sendMove({ y: -STEP_CM })}
          >
            <ArrowRight className="h-6 w-6" />
            <span className="sr-only">Right (Y-)</span>
          </Button>

          {/* Row 3: Up Z, Down Z */}
          <Button
            size="lg"
            variant="secondary"
            className="h-14 text-sm touch-manipulation"
            disabled={!selectedRobotName || sending}
            onClick={() => sendMove({ z: STEP_CM })}
          >
            Z+
          </Button>
          <Button
            size="lg"
            variant="secondary"
            className="h-14 text-sm touch-manipulation"
            disabled={!selectedRobotName || sending}
            onClick={() => sendMove({ z: -STEP_CM })}
          >
            Z-
          </Button>
          <div />
        </div>
        )}

        <div className="flex gap-3">
          <Button
            size="lg"
            variant="default"
            className="flex-1 h-14 text-base touch-manipulation"
            disabled={!selectedRobotName || sending}
            onClick={() => sendMove({ open: 1 })}
          >
            <Hand className="h-5 w-5 mr-2" />
            Open gripper
          </Button>
          <Button
            size="lg"
            variant="default"
            className="flex-1 h-14 text-base touch-manipulation"
            disabled={!selectedRobotName || sending}
            onClick={() => sendMove({ open: 0 })}
          >
            <HandMetal className="h-5 w-5 mr-2" />
            Close gripper
          </Button>
        </div>

        {sending && (
          <p className="text-center text-sm text-muted-foreground">
            Sending…
          </p>
        )}
      </div>
    </div>
  );
}
