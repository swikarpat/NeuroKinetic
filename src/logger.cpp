#include "neurokinetic/telemetry/logger.hpp"

#include <iostream>

namespace neurokinetic::telemetry {

AsyncLogger::~AsyncLogger() { stop(); }

void AsyncLogger::start() {
    bool expected = false;
    if (running_.compare_exchange_strong(expected, true, std::memory_order_release)) {
        thread_ = std::thread(&AsyncLogger::drain, this);
    }
}

void AsyncLogger::stop() noexcept {
    if (!running_.exchange(false, std::memory_order_acq_rel)) return;
    if (thread_.joinable()) thread_.join();
}

bool AsyncLogger::try_log(const LogEvent& event) noexcept { return queue_.try_push(event); }

void AsyncLogger::drain() noexcept {
    LogEvent event{};
    while (running_.load(std::memory_order_acquire)) {
        if (queue_.try_pop(event)) {
            std::cout << "{\"timestamp_ns\":" << event.timestamp_ns << ",\"tag\":\""
                      << event.tag << "\",\"message\":\"" << event.message << "\"}\n";
        } else {
            std::this_thread::yield();
        }
    }
    while (queue_.try_pop(event)) {
        std::cout << "{\"timestamp_ns\":" << event.timestamp_ns << ",\"tag\":\""
                  << event.tag << "\",\"message\":\"" << event.message << "\"}\n";
    }
}

} // namespace neurokinetic::telemetry
