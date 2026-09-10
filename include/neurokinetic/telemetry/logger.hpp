#pragma once

#include "neurokinetic/telemetry/spsc_ring_buffer.hpp"
#include <atomic>
#include <cstdint>
#include <thread>

namespace neurokinetic::telemetry {

struct LogEvent {
    uint64_t timestamp_ns{};
    char tag[32]{};
    char message[160]{};
};

class AsyncLogger {
public:
    AsyncLogger() = default;
    ~AsyncLogger();
    AsyncLogger(const AsyncLogger&) = delete;
    AsyncLogger& operator=(const AsyncLogger&) = delete;

    void start();
    void stop() noexcept;
    bool try_log(const LogEvent& event) noexcept;

private:
    void drain() noexcept;
    SpscRingBuffer<LogEvent, 2048> queue_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

} // namespace neurokinetic::telemetry
