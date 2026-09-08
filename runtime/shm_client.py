import mmap
import struct
import os
import time

DOF = 7
SHM_NAME = "/tmp/neurokinetic_ipc.shm"

# Offset definitions
MAGIC_OFFSET = 0
HEARTBEAT_CPP_OFFSET = 8
HEARTBEAT_PY_OFFSET = 16
TELEM_OFFSET = 64
CMD_OFFSET = 128

TELEM_FORMAT = "=QQ7f7f7ffIII16s"
CMD_FORMAT = "=Q7f7fffI16s"

class NeuroKineticClient:
    def __init__(self):
        if not os.path.exists(SHM_NAME):
            raise FileNotFoundError(f"Shared memory file {SHM_NAME} not found. Start daemon first.")
        
        self.fd = os.open(SHM_NAME, os.O_RDWR)
        self.shm_size = 256
        self.buf = mmap.mmap(self.fd, self.shm_size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        self.heartbeat_counter = 0

    def send_heartbeat(self):
        """Sends non-blocking heartbeat to satisfy C++ dead-man's watchdog."""
        self.heartbeat_counter += 1
        self.buf.seek(HEARTBEAT_PY_OFFSET)
        self.buf.write(struct.pack("=Q", self.heartbeat_counter))

    def read_telemetry(self) -> dict:
        self.send_heartbeat()
        for _ in range(10):
            self.buf.seek(TELEM_OFFSET)
            raw = self.buf.read(struct.calcsize(TELEM_FORMAT))
            unpacked = struct.unpack(TELEM_FORMAT, raw)
            
            seq1 = unpacked[0]
            if seq1 % 2 != 0:
                continue

            ts_ns = unpacked[1]
            positions = list(unpacked[2:9])
            velocities = list(unpacked[9:16])
            torques = list(unpacked[16:23])
            margin = unpacked[23]
            clamped = bool(unpacked[24])
            error_code = unpacked[25]
            watchdog_tripped = bool(unpacked[26])

            return {
                "sequence": seq1,
                "timestamp_ns": ts_ns,
                "positions": positions,
                "velocities": velocities,
                "torques": torques,
                "barrier_margin": margin,
                "clamped": clamped,
                "error_code": error_code,
                "watchdog_tripped": watchdog_tripped
            }
        return {"clamped": False, "barrier_margin": 0.0, "positions": [0.0]*DOF, "velocities": [0.0]*DOF, "torques": [0.0]*DOF, "sequence": 0, "timestamp_ns": 0, "watchdog_tripped": False}

    def dispatch_vla_torque(self, torques: list[float], safety_margin: float = 0.08, gamma: float = 15.0):
        self.send_heartbeat()
        self.buf.seek(CMD_OFFSET)
        curr_seq = struct.unpack("=Q", self.buf.read(8))[0]
        new_seq = curr_seq + 1

        payload = struct.pack(
            CMD_FORMAT,
            new_seq,
            *torques,
            *[0.0] * DOF,
            float(safety_margin),
            float(gamma),
            1,
            b"\x00" * 16
        )
        self.buf.seek(CMD_OFFSET)
        self.buf.write(payload)

    def close(self):
        self.buf.close()
        os.close(self.fd)
