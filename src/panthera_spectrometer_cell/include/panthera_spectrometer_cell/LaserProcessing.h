#pragma once

#include <cstdint>
#include <deque>
#include <string>

namespace panthera_spectrometer_cell
{

inline constexpr double kScanDurationSec = 40.0;

struct LaserSnapshot
{
  bool rawValid{false};
  bool filteredValid{false};
  bool stable{false};
  double rawMm{0.0};
  double filteredMm{0.0};
  double stableMm{0.0};
  double spanMm{0.0};
  double ageSec{0.0};
  std::uint64_t sequence{0};
};

class LaserFilter
{
public:
  LaserFilter(double min_mm = 120.0, double max_mm = 280.0);

  void configure(double min_mm, double max_mm);
  void add(double value_mm, bool valid, double now_sec);
  std::uint64_t beginCapture() const;
  LaserSnapshot snapshot(double now_sec, std::uint64_t capture_after = 0) const;

private:
  struct FilteredSample
  {
    std::uint64_t sequence{0};
    double valueMm{0.0};
  };

  double min_mm_{120.0};
  double max_mm_{280.0};
  double latest_raw_mm_{0.0};
  double latest_time_sec_{0.0};
  std::uint64_t sequence_{0};
  std::deque<double> raw_window_;
  std::deque<FilteredSample> filtered_window_;
};

enum class ScanPhase
{
  WAIT_STANDARD_RETURN,
  WAIT_STANDARD_EXIT,
  SEEK_FARTHEST,
  SCANNING,
  COMPLETE
};

struct ScanStatus
{
  ScanPhase phase{ScanPhase::WAIT_STANDARD_RETURN};
  bool done{false};
  double farthestMm{0.0};
  double remainingSec{kScanDurationSec};
  std::string message;
};

class ScanDetectionTracker
{
public:
  void start(double reference_mm, double now_sec, std::uint64_t initial_sequence = 0);
  void update(const LaserSnapshot & laser, double now_sec);
  ScanStatus status(double now_sec) const;

private:
  void waitForStandardReturn();
  void waitForStandardExit();

  ScanPhase phase_{ScanPhase::WAIT_STANDARD_RETURN};
  double reference_mm_{0.0};
  double farthest_mm_{0.0};
  double farthest_time_sec_{0.0};
  double last_sample_time_sec_{0.0};
  std::uint64_t last_sequence_{0};
  int exit_count_{0};
  int return_count_{0};
  bool active_{false};
};

const char * toString(ScanPhase phase);

}  // namespace panthera_spectrometer_cell
