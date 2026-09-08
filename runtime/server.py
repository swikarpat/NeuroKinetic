import sys
from pathlib import Path

# Add project root to sys.path
ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

import asyncio
import json
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

from runtime.shm_client import NeuroKineticClient, DOF
from runtime.fno_surrogate import fno_twin

client = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    global client
    try:
        client = NeuroKineticClient()
        print("✓ Connected to C++23 Safety Daemon via POSIX SHM")
    except Exception as e:
        print(f"[Warning] C++ Daemon not running yet: {e}")
    yield

app = FastAPI(title="NeuroKinetic Digital Twin Gateway", lifespan=lifespan)

active_connections: list[WebSocket] = []

@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    await websocket.accept()
    active_connections.append(websocket)
    try:
        while True:
            await asyncio.sleep(0.02)  # 50 Hz refresh rate
            if not client:
                continue

            telem = client.read_telemetry()
            physics = fno_twin.evaluate_physics_twin(
                telem["positions"],
                telem["velocities"],
                telem["torques"]
            )

            payload = {
                "sequence": telem["sequence"],
                "timestamp_ns": telem["timestamp_ns"],
                "positions": telem["positions"],
                "velocities": telem["velocities"],
                "torques": telem["torques"],
                "barrier_margin": round(telem["barrier_margin"], 4),
                "clamped": telem["clamped"],
                "error_code": telem["error_code"],
                "temperatures": physics["joint_temperatures_c"],
                "stresses": physics["von_mises_stress_mpa"],
                "max_stress": physics["max_stress_mpa"],
                "max_temp": physics["max_temp_c"],
                "fno_eval_ms": physics["fno_eval_time_ms"]
            }

            await websocket.send_text(json.dumps(payload))
    except (WebSocketDisconnect, Exception):
        if websocket in active_connections:
            active_connections.remove(websocket)

@app.post("/api/v1/actuate/hazard")
async def trigger_hazard():
    """Triggers an intentional +45.0 Nm spike on Joint 5 to demonstrate live CBF clamping."""
    if client:
        client.dispatch_vla_torque([0.0, 0.0, 0.0, 0.0, 0.0, 45.0, 0.0])
        return {"status": "HAZARD_DISPATCHED", "joint": 5, "commanded_torque_nm": 45.0}
    return {"status": "ERROR_NO_SHM"}

@app.post("/api/v1/actuate/reset")
async def trigger_reset():
    """Resets the arm torques back to zero."""
    if client:
        client.dispatch_vla_torque([0.0] * DOF)
        return {"status": "RESET_DISPATCHED"}
    return {"status": "ERROR_NO_SHM"}

app.mount("/static", StaticFiles(directory="frontend"), name="static")

@app.get("/")
async def root():
    return FileResponse("frontend/index.html")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("runtime.server:app", host="0.0.0.0", port=8000, reload=False, log_level="warning")
