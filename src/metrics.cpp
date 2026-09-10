#include "neurokinetic/telemetry/metrics.hpp"

#include <algorithm>
#include <sstream>

namespace neurokinetic::telemetry {

void Metrics::observe(std::array<std::atomic<uint64_t>, bucket_count>& buckets, std::atomic<uint64_t>& sum, double value) noexcept {
    static constexpr double limits[] = {50.0, 100.0, 250.0, 500.0, 1000.0};
    std::size_t bucket = 5;
    for (std::size_t index = 0; index < 5; ++index) {
        if (value <= limits[index]) { bucket = index; break; }
    }
    buckets[bucket].fetch_add(1, std::memory_order_relaxed);
    sum.fetch_add(static_cast<uint64_t>(std::max(0.0, value) * 1000.0), std::memory_order_relaxed);
}

void Metrics::observe_loop_execution_duration_us(double value) noexcept { observe(loop_buckets_, loop_sum_milli_us_, value); }
void Metrics::observe_kinematics_solve_duration_us(double value) noexcept { observe(solve_buckets_, solve_sum_milli_us_, value); }
void Metrics::increment_deadline_misses() noexcept { deadline_misses_.fetch_add(1, std::memory_order_relaxed); }
void Metrics::increment_emergency_stops() noexcept { emergency_stops_.fetch_add(1, std::memory_order_relaxed); }

std::string Metrics::prometheus_text() const {
    std::ostringstream output;
    const char* names[] = {"50", "100", "250", "500", "1000", "+Inf"};
    auto histogram = [&output, names](const char* name, const auto& buckets, uint64_t sum) {
        output << "# TYPE " << name << " histogram\n";
        uint64_t count = 0;
        for (std::size_t index = 0; index < bucket_count; ++index) {
            count += buckets[index].load(std::memory_order_relaxed);
            output << name << "_bucket{le=\"" << names[index] << "\"} " << count << '\n';
        }
        output << name << "_count " << count << '\n' << name << "_sum " << sum / 1000.0 << '\n';
    };
    histogram("neurokinetic_loop_execution_duration_us", loop_buckets_, loop_sum_milli_us_.load());
    histogram("neurokinetic_kinematics_solve_duration_us", solve_buckets_, solve_sum_milli_us_.load());
    output << "# TYPE neurokinetic_deadline_misses_total counter\nneurokinetic_deadline_misses_total " << deadline_misses_.load() << '\n'
           << "# TYPE neurokinetic_emergency_stops_total counter\nneurokinetic_emergency_stops_total " << emergency_stops_.load() << '\n';
    return output.str();
}

} // namespace neurokinetic::telemetry
