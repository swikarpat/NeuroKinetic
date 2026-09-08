import mmap
import struct
import os

DOF = 7
SHM_NAME = "/tmp/neurokinetic_ipc.shm"

# Memory Layout matching C++23 alignas(64) boundaries:
# 0..64:   Magic (4B) + Heartbeats (16B) + Padding
# 64..256: TelemetrySlot (192B with alignas(64))
# 256..384: CommandSlot (128B with alignas(64))
HEARTBEAT_PY_OFFSET = 16
TELEM_OFFSET = 64
CMD_OFFSET = 256
SHM_TOTAL_SIZE = 384

TELEM_FORMAT = "=QQ7f7f7ffIII16s"
CMD_FORMAT = "=Q7f7fffI16s"

class NeuroKineticClient:
    def __init__(self):
        if not os.path.exists(SHM_NAME):
            raise FileNotFoundError(f"Shared memory file {SHM_NAME} not found. Start daemon first.")
        
        self.fd = os.open(SHM_NAME, os.O_RDWR)
        self.buf = mmap.mmap(self.fd, SHM_TOTAL_SIZE, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        self.heartbeat_counter = 0

    def send_heartbeat(self):
        self.heartbeat_counter += 1
        self.buf[HEARTBEAT_PY_OFFSET:HEARTBEAT_PY_OFFSET + 8] = struct.pack("=Q", self.heartbeat_counter)

    def read_telemetry(self) -> dict:
        self.send_heartbeat()
        for _ in range(10):
            raw = self.buf[TELEM_OFFSET:TELEM_OFFSET + struct.calcsize(TELEM_FORMAT)]
            unpacked = struct.unpack(TELEM_FORMAT, raw)
            
            seq1 = unpacked[0]
            if seq1 % 2 != 0 or seq1 == 0:
                continue

            return {
                "sequence": seq1,
                "timestamp_ns": unpacked[1],
                "positions": list(unpacked[2:9]),
                "velocities": list(unpacked[9:16]),
                "torques": list(unpacked[16:23]),
                "barrier_margin": unpacked[23],
                "clamped": bool(unpacked[24]),
                "error_code": unpacked[25],
                "watchdog_tripped": bool(unpacked[26])
            }
        return {"clamped": False, "barrier_margin": 0.0, "positions": [0.0]*DOF, "velocities": [0.0]*DOF, "torques": [0.0]*DOF, "sequence": 0, "timestamp_ns": 0, "watchdog_tripped": False}

    def dispatch_vla_torque(self, torques: list[float], safety_margin: float = 0.08, gamma: float = 15.0):
        self.send_heartbeat()
        curr_seq = struct.unpack("=Q", self.buf[CMD_OFFSET:CMD_OFFSET + 8])[0]
        
        # 1. Monotonic even sequence counter (Even = read-safe in Seqlock)
        final_seq = curr_seq + 2 if curr_seq % 2 == 0 else curr_seq + 1

        payload = struct.pack(
            CMD_FORMAT,
            final_seq,
            *torques,
            *[0.0] * DOF,
            float(safety_margin),
            float(gamma),
            1,
            b"\x00" * 16
        )
        self.buf[CMD_OFFSET:CMD_OFFSET + len(payload)] = payload

    def close(self):
        self.buf.close()
        os.close(self.fd)
