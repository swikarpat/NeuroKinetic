#include "neurokinetic/telemetry/telemetry_worker.hpp"

#include <arpa/inet.h>
#include <bit>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <netinet/in.h>
#include <pthread.h>
#ifdef __APPLE__
#include <pthread/qos.h>
#endif
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

namespace neurokinetic::telemetry {
namespace {

uint32_t float_bits(float value) noexcept {
    return std::bit_cast<uint32_t>(value);
}

float bits_float(uint32_t value) noexcept {
    return std::bit_cast<float>(value);
}

void send_json_event(const KinematicTelemetryFrame& frame) noexcept {
    const int socket_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
        std::cout << "{\"event\":\"SAFETY_LIMIT_BREACH\",\"timestamp_ns\":"
                  << frame.timestamp_ns << ",\"flags\":" << frame.safety_flags << "}\n";
        return;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(3100);
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
    const std::string body = "{\"streams\":[{\"stream\":{\"service\":\"neurokinetic\",\"event\":\"SAFETY_LIMIT_BREACH\"},\"values\":[[\"" +
        std::to_string(frame.timestamp_ns) + "\",\"{\\\"flags\\\":" + std::to_string(frame.safety_flags) +
        ",\\\"loop_duration_us\\\":" + std::to_string(frame.loop_duration_us) + "}\"]]}]}";
    if (::connect(socket_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0) {
        const std::string request = "POST /loki/api/v1/push HTTP/1.1\r\nHost: localhost:3100\r\nContent-Type: application/json\r\nContent-Length: " +
            std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body;
        (void)::send(socket_fd, request.data(), request.size(), MSG_NOSIGNAL);
    } else {
        std::cout << body << '\n';
    }
    ::close(socket_fd);
}

void send_trace_event(const KinematicTelemetryFrame& frame) noexcept {
    const int socket_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) return;
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(4318);
    ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
    const std::string body = "{\"resourceSpans\":[{\"resource\":{\"attributes\":[{\"key\":\"service.name\",\"value\":{\"stringValue\":\"neurokinetic\"}}]},\"scopeSpans\":[{\"spans\":[{\"name\":\"SAFETY_LIMIT_BREACH\",\"kind\":1,\"startTimeUnixNano\":\"" +
        std::to_string(frame.timestamp_ns) + "\",\"endTimeUnixNano\":\"" + std::to_string(frame.timestamp_ns + 1000) +
        "\",\"attributes\":[{\"key\":\"safety_flags\",\"value\":{\"intValue\":\"" + std::to_string(frame.safety_flags) + "\"}}]}]}]}]}";
    if (::connect(socket_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0) {
        const std::string request = "POST /v1/traces HTTP/1.1\r\nHost: localhost:4318\r\nContent-Type: application/json\r\nContent-Length: " +
            std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n" + body;
        (void)::send(socket_fd, request.data(), request.size(), MSG_NOSIGNAL);
    }
    ::close(socket_fd);
}

} // namespace

TelemetryWorker::TelemetryWorker(uint16_t metrics_port) noexcept : metrics_port_(metrics_port) {}

TelemetryWorker::~TelemetryWorker() {
    stop();
}

void TelemetryWorker::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true, std::memory_order_release)) {
        return;
    }
    worker_thread_ = std::thread(&TelemetryWorker::drain_loop, this);
    metrics_thread_ = std::thread(&TelemetryWorker::metrics_loop, this);
}

void TelemetryWorker::stop() noexcept {
    if (!running_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    if (worker_thread_.joinable()) worker_thread_.join();
    if (metrics_thread_.joinable()) metrics_thread_.join();
}

bool TelemetryWorker::try_push(const KinematicTelemetryFrame& frame) noexcept {
    return queue_.try_push(frame);
}

void TelemetryWorker::record(const KinematicTelemetryFrame& frame) noexcept {
    loop_samples_.fetch_add(1, std::memory_order_relaxed);
    loop_sum_us_.fetch_add(static_cast<uint64_t>(frame.loop_duration_us * 1000.0f), std::memory_order_relaxed);
    const float buckets[] = {50.0f, 100.0f, 250.0f, 500.0f, 1000.0f};
    std::size_t bucket = 5;
    for (std::size_t index = 0; index < 5; ++index) {
        if (frame.loop_duration_us <= buckets[index]) {
            bucket = index;
            break;
        }
    }
    loop_buckets_[bucket].fetch_add(1, std::memory_order_relaxed);
    condition_number_bits_.store(float_bits(frame.condition_number), std::memory_order_relaxed);
    torque_ratio_bits_.store(float_bits(frame.max_torque_saturation_ratio), std::memory_order_relaxed);
    if (frame.safety_flags != 0) {
        if (frame.safety_flags & 1u) safety_violations_[0].fetch_add(1, std::memory_order_relaxed);
        if (frame.safety_flags & 2u) safety_violations_[1].fetch_add(1, std::memory_order_relaxed);
        if (frame.safety_flags & 4u) safety_violations_[2].fetch_add(1, std::memory_order_relaxed);
        export_safety_event(frame);
    }
}

void TelemetryWorker::export_safety_event(const KinematicTelemetryFrame& frame) noexcept {
    exported_events_.fetch_add(1, std::memory_order_relaxed);
    send_trace_event(frame);
    send_json_event(frame);
}

void TelemetryWorker::drain_loop() noexcept {
#ifdef __APPLE__
    (void)pthread_set_qos_class_self_np(QOS_CLASS_UTILITY, 0);
#endif
    KinematicTelemetryFrame frame{};
    while (running_.load(std::memory_order_acquire)) {
        if (queue_.try_pop(frame)) {
            record(frame);
        } else {
            std::this_thread::yield();
        }
    }
    while (queue_.try_pop(frame)) record(frame);
}

void TelemetryWorker::metrics_loop() noexcept {
    const int server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) return;
    int reuse = 1;
    (void)::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(metrics_port_);
    if (::bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 || ::listen(server_fd, 4) != 0) {
        ::close(server_fd);
        return;
    }

    while (running_.load(std::memory_order_acquire)) {
        timeval timeout{1, 0};
        (void)::setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        const int client_fd = ::accept(server_fd, nullptr, nullptr);
        if (client_fd < 0) continue;
        std::ostringstream body;
        const uint64_t samples = loop_samples_.load(std::memory_order_relaxed);
        body << "# TYPE neurokinetic_loop_duration_us histogram\n"
             << "neurokinetic_loop_duration_us_count " << samples << "\n"
             << "neurokinetic_loop_duration_us_sum " << (loop_sum_us_.load(std::memory_order_relaxed) / 1000.0) << "\n";
        const char* le[] = {"50", "100", "250", "500", "1000", "+Inf"};
        for (std::size_t index = 0; index < 6; ++index) {
            uint64_t cumulative = 0;
            for (std::size_t bucket = 0; bucket <= index; ++bucket) cumulative += loop_buckets_[bucket].load(std::memory_order_relaxed);
            body << "neurokinetic_loop_duration_us_bucket{le=\"" << le[index] << "\"} " << cumulative << "\n";
        }
        body << "# TYPE neurokinetic_condition_number gauge\nneurokinetic_condition_number " << bits_float(condition_number_bits_.load(std::memory_order_relaxed)) << "\n"
             << "# TYPE neurokinetic_torque_saturation_ratio gauge\nneurokinetic_torque_saturation_ratio " << bits_float(torque_ratio_bits_.load(std::memory_order_relaxed)) << "\n"
             << "# TYPE neurokinetic_safety_violations_total counter\n"
             << "neurokinetic_safety_violations_total{type=\"estop\"} " << safety_violations_[0].load(std::memory_order_relaxed) << "\n"
             << "neurokinetic_safety_violations_total{type=\"kinematic_singularity\"} " << safety_violations_[1].load(std::memory_order_relaxed) << "\n"
             << "neurokinetic_safety_violations_total{type=\"velocity_clamp\"} " << safety_violations_[2].load(std::memory_order_relaxed) << "\n"
             << "# TYPE neurokinetic_telemetry_drops_total counter\nneurokinetic_telemetry_drops_total " << queue_.dropped() << "\n";
        const std::string payload = body.str();
        const std::string response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain; version=0.0.4\r\nContent-Length: " + std::to_string(payload.size()) + "\r\nConnection: close\r\n\r\n" + payload;
        (void)::send(client_fd, response.data(), response.size(), MSG_NOSIGNAL);
        ::close(client_fd);
    }
    ::close(server_fd);
}

} // namespace neurokinetic::telemetry
