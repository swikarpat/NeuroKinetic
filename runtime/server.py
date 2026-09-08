import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

import asyncio
import json
import subprocess
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel

from runtime.shm_client import NeuroKineticClient, DOF
from runtime.fno_surrogate import fno_twin

client = None
current_safety_margin = 0.08
current_gamma = 15.0

class TuningRequest(BaseModel):
    safety_margin: float
    gamma: float

@asynccontextmanager
async def lifespan(app: FastAPI):
    global client
    try:
        client = NeuroKineticClient()
        print("✓ Connected to C++23 Safety Daemon via POSIX SHM")
    except Exception as e:
        print(f"[Warning] C++ Daemon not running: {e}")
    yield

app = FastAPI(title="NeuroKinetic Digital Twin Gateway", lifespan=lifespan)
active_connections: list[WebSocket] = []

@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    await websocket.accept()
    active_connections.append(websocket)
    try:
        while True:
            await asyncio.sleep(0.02)  # 50 Hz
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
                "watchdog_tripped": telem.get("watchdog_tripped", False),
                "active_safety_margin": current_safety_margin,
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

@app.get("/api/v1/incidents")
async def get_rocksdb_incidents():
    """Extracts recent intervention audit logs directly from RocksDB NVMe store."""
    reader_path = ROOT_DIR / "build" / "read_incidents"
    if not reader_path.exists():
        return JSONResponse([])
    try:
        proc = subprocess.run([str(reader_path)], capture_output=True, text=True, timeout=2.0)
        data = json.loads(proc.stdout)
        return JSONResponse(data)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)

@app.post("/api/v1/tuning/barrier")
async def update_barrier_tuning(tuning: TuningRequest):
    global current_safety_margin, current_gamma
    current_safety_margin = float(tuning.safety_margin)
    current_gamma = float(tuning.gamma)
    if client:
        client.dispatch_vla_torque([0.0]*DOF, safety_margin=current_safety_margin, gamma=current_gamma)
    return {"status": "TUNING_APPLIED", "margin": current_safety_margin, "gamma": current_gamma}

@app.post("/api/v1/actuate/hazard")
async def trigger_hazard():
    if client:
        client.dispatch_vla_torque([0.0, 0.0, 0.0, 0.0, 0.0, 45.0, 0.0], safety_margin=current_safety_margin, gamma=current_gamma)
        return {"status": "HAZARD_DISPATCHED", "joint": 5, "commanded_torque_nm": 45.0}
    return {"status": "ERROR_NO_SHM"}

@app.post("/api/v1/actuate/reset")
async def trigger_reset():
    if client:
        client.dispatch_vla_torque([0.0] * DOF, safety_margin=current_safety_margin, gamma=current_gamma)
        return {"status": "RESET_DISPATCHED"}
    return {"status": "ERROR_NO_SHM"}

app.mount("/static", StaticFiles(directory="frontend"), name="static")

@app.get("/")
async def root():
    return FileResponse("frontend/index.html")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("runtime.server:app", host="0.0.0.0", port=8000, reload=False, log_level="warning")
