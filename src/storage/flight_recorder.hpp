#pragma once

#include <rocksdb/db.h>
#include <rocksdb/options.h>
#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <span>

namespace neurokinetic::storage {

struct InterventionRecord {
    uint64_t sequence_number;
    uint64_t timestamp_ns;
    float barrier_margin;
    float commanded_joint5_torque;
    float safe_joint5_torque;
    uint32_t active_mask;
};

class FlightRecorder {
public:
    explicit FlightRecorder(const std::string& db_path = "/tmp/neurokinetic_flight_recorder");
    ~FlightRecorder();

    FlightRecorder(const FlightRecorder&) = delete;
    FlightRecorder& operator=(const FlightRecorder&) = delete;

    bool log_cbf_intervention(const InterventionRecord& record) noexcept;
    bool log_telemetry_frame(uint64_t seq, uint64_t ts_ns, std::span<const float> positions, std::span<const float> torques) noexcept;
    void flush() noexcept;

private:
    std::string db_path_;
    std::unique_ptr<rocksdb::DB> db_{nullptr};
    rocksdb::ColumnFamilyHandle* cf_interventions_{nullptr};
    rocksdb::ColumnFamilyHandle* cf_telemetry_{nullptr};
    rocksdb::WriteOptions write_options_fast_;
};

} // namespace neurokinetic::storage
