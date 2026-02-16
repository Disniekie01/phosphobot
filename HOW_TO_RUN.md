# How to run IRL Robotics (Phosphobot)

This guide explains how to build the dashboard and run the server from the phosphobot repo.

## Prerequisites

- **Python 3.9+** (3.10 recommended)
- **uv** – [Install](https://docs.astral.sh/uv/getting-started/installation/) (e.g. `curl -LsSf https://astral.sh/uv/install.sh | sh`)
- **Node.js and npm** – required to build the dashboard (only if you want the web UI)

On a fresh Ubuntu/Pop!_OS machine you can use the project install script:

```bash
bash install.sh
```

## Repo layout

From the **repo root** (the folder that contains both `dashboard/` and `phosphobot/`):

```
phosphobot/                 ← repo root (run make and commands from here)
├── dashboard/               ← frontend source (npm build)
├── phosphobot/              ← Python package
│   ├── phosphobot/          ← app code
│   └── resources/
│       └── dist/            ← built dashboard (created by make build_frontend)
├── Makefile
└── HOW_TO_RUN.md
```

## 1. Build the dashboard

The server expects the built frontend at `phosphobot/resources/dist/`. Build and copy it from the repo root:

```bash
make build_frontend
```

This installs dashboard deps, runs `npm run build`, and copies `dashboard/dist/*` to `phosphobot/resources/dist/`. You can run `make build_frontend` from any directory; the Makefile uses its own path so the copy target is always correct.

If you see *"The 'dist' directory does not exist"* when starting the server, run `make build_frontend` from the repo root and try again.

## 2. Run the server

All run commands assume you start from the **repo root**.

### Quick run (no telemetry)

```bash
cd phosphobot && uv run irlrobotics run --port 8020
```

Or with simulation GUI:

```bash
cd phosphobot && uv run irlrobotics run --simulation=gui --port 8020 --no-telemetry
```

### Using the Makefile

From the repo root:

| Target | Description |
|--------|-------------|
| `make prod_no_telemetry` | Build frontend (if needed) and run server, headless sim, no telemetry |
| `make prod` | Run with telemetry (builds frontend first) |
| `make prod_gui` | Run with simulation GUI |
| `make local` | Run on localhost:8080, GUI, no telemetry |
| `make build_frontend` | Only build and copy the dashboard |

Example:

```bash
make prod_no_telemetry
```

The dashboard is then available at `http://<your-ip>:8020` (or the port you passed with `--port`). Default port is 80 if you don’t pass `--port`.

## 3. Port and host

- **Port:** e.g. `irlrobotics run --port 8020` (default is 80).
- **Host:** default is `0.0.0.0` (all interfaces). For local-only: `--host 127.0.0.1`.

## 4. App data directory (optional)

User data (recordings, config, tokens, calibration) is stored under a “home app” directory. By default it is:

- `~/phosphobot` (e.g. `/home/ubuntu/phosphobot` on that machine)

To use a different path (e.g. on a new machine or a custom volume), set:

```bash
export PHOSPHOBOT_HOME=/path/to/your/phosphobot-data
cd phosphobot && uv run irlrobotics run --port 8020
```

Then run the server as usual.

## 5. Troubleshooting

| Issue | What to do |
|-------|------------|
| `FileNotFoundError: The 'dist' directory does not exist` | From repo root run `make build_frontend`. Ensure Node/npm are installed. |
| `Failed to spawn: irlrobotics` | Run from inside the `phosphobot` package dir: `cd phosphobot && uv run irlrobotics run ...` (from repo root: `phosphobot` is the inner package that has `pyproject.toml`). |
| Serial/robot not found | Check USB/serial connections and that your user is in the `dialout` group: `sudo usermod -aG dialout $USER` (then log out and back in). |

## 6. One-liner from repo root

Build frontend and run on port 8020 with no telemetry:

```bash
make build_frontend && cd phosphobot && uv run irlrobotics run --port 8020 --no-telemetry
```

Or use the combined Make target:

```bash
make prod_no_telemetry
```
