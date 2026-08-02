#include "panthera_spectrometer_cell/LaserProcessing.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace panthera_spectrometer_cell
{
namespace
{

double median(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const auto middle = values.size() / 2;
  return values.size() % 2 == 0 ?
         (values[middle - 1] + values[middle]) * 0.5 : values[middle];
}

}  // namespace

LaserFilter::LaserFilter(double min_mm, double max_mm)
: min_mm_(min_mm), max_mm_(max_mm)
{
}

void LaserFilter::configure(double min_mm, double max_mm)
{
  min_mm_ = min_mm;
  max_mm_ = max_mm;
  raw_window_.clear();
  filtered_window_.clear();
  latest_time_sec_ = 0.0;
}

void LaserFilter::add(double value_mm, bool valid, double now_sec)
{
  if (!valid || !std::isfinite(value_mm) || value_mm < min_mm_ || value_mm > max_mm_) {
    return;
  }
  if (latest_time_sec_ > 0.0 && now_sec - latest_time_sec_ > 1.0) {
    raw_window_.clear();
    filtered_window_.clear();
  }
  latest_raw_mm_ = value_mm;
  latest_time_sec_ = now_sec;
  raw_window_.push_back(value_mm);
  if (raw_window_.size() > 5) {
    raw_window_.pop_front();
  }
  if (raw_window_.size() < 5) {
    return;
  }

  std::vector<double> values(raw_window_.begin(), raw_window_.end());
  filtered_window_.push_back(FilteredSample{++sequence_, median(values)});
  if (filtered_window_.size() > 64) {
    filtered_window_.pop_front();
  }
}

std::uint64_t LaserFilter::beginCapture() const
{
  return sequence_;
}

LaserSnapshot LaserFilter::snapshot(double now_sec, std::uint64_t capture_after) const
{
  LaserSnapshot result;
  result.sequence = sequence_;
  if (latest_time_sec_ <= 0.0) {
    return result;
  }
  result.ageSec = std::max(0.0, now_sec - latest_time_sec_);
  result.rawMm = latest_raw_mm_;
  result.rawValid = result.ageSec <= 1.0;
  if (!result.rawValid || filtered_window_.empty()) {
    return result;
  }
  result.filteredValid = true;
  result.filteredMm = filtered_window_.back().valueMm;

  std::vector<double> captured;
  for (const auto & sample : filtered_window_) {
    if (sample.sequence > capture_after) {
      captured.push_back(sample.valueMm);
    }
  }
  if (captured.size() < 15) {
    return result;
  }
  captured.erase(captured.begin(), captured.end() - 15);
  const auto limits = std::minmax_element(captured.begin(), captured.end());
  result.spanMm = *limits.second - *limits.first;
  result.stable = result.spanMm <= 1.0;
  result.stableMm = median(captured);
  return result;
}

void ScanDetectionTracker::start(
  double reference_mm, double, std::uint64_t initial_sequence)
{
  reference_mm_ = reference_mm;
  active_ = true;
  last_sequence_ = initial_sequence;
  last_sample_time_sec_ = 0.0;
  waitForStandardReturn();
}

void ScanDetectionTracker::waitForStandardReturn()
{
  phase_ = ScanPhase::WAIT_STANDARD_RETURN;
  exit_count_ = 0;
  return_count_ = 0;
  farthest_mm_ = 0.0;
  farthest_time_sec_ = 0.0;
}

void ScanDetectionTracker::waitForStandardExit()
{
  phase_ = ScanPhase::WAIT_STANDARD_EXIT;
  exit_count_ = 0;
  return_count_ = 0;
  farthest_mm_ = 0.0;
  farthest_time_sec_ = 0.0;
}

void ScanDetectionTracker::update(const LaserSnapshot & laser, double now_sec)
{
  if (!active_ || !laser.filteredValid || laser.ageSec > 1.0 ||
    laser.sequence == last_sequence_)
  {
    return;
  }
  last_sequence_ = laser.sequence;
  const bool resumed_after_gap =
    last_sample_time_sec_ > 0.0 && now_sec - last_sample_time_sec_ > 1.0;
  last_sample_time_sec_ = now_sec;
  const double value = laser.filteredMm;

  if (phase_ == ScanPhase::WAIT_STANDARD_RETURN) {
    return_count_ = std::abs(value - reference_mm_) <= 2.0 ? return_count_ + 1 : 0;
    if (return_count_ >= 5) {
      waitForStandardExit();
    }
    return;
  }

  if (phase_ == ScanPhase::WAIT_STANDARD_EXIT) {
    exit_count_ = value >= reference_mm_ + 5.0 ? exit_count_ + 1 : 0;
    if (exit_count_ >= 5) {
      phase_ = ScanPhase::SEEK_FARTHEST;
      farthest_mm_ = value;
      farthest_time_sec_ = now_sec;
      return_count_ = 0;
    }
    return;
  }

  if (phase_ == ScanPhase::SEEK_FARTHEST) {
    if (resumed_after_gap) {
      farthest_time_sec_ = now_sec;
    }
    return_count_ = std::abs(value - reference_mm_) <= 2.0 ? return_count_ + 1 : 0;
    if (return_count_ >= 5) {
      waitForStandardExit();
      return;
    }
    if (value > farthest_mm_ + 0.5) {
      farthest_mm_ = value;
      farthest_time_sec_ = now_sec;
    } else if (now_sec - farthest_time_sec_ >= 2.0) {
      phase_ = ScanPhase::SCANNING;
    }
  }

  if (phase_ == ScanPhase::SCANNING) {
    return_count_ = std::abs(value - reference_mm_) <= 2.0 ? return_count_ + 1 : 0;
    if (return_count_ >= 5) {
      waitForStandardExit();
    } else if (now_sec - farthest_time_sec_ >= kScanDurationSec) {
      phase_ = ScanPhase::COMPLETE;
    }
  }
}

ScanStatus ScanDetectionTracker::status(double now_sec) const
{
  ScanStatus result;
  const bool timer_complete =
    phase_ == ScanPhase::SCANNING && now_sec - farthest_time_sec_ >= kScanDurationSec;
  result.phase = timer_complete ? ScanPhase::COMPLETE : phase_;
  result.done = active_ && (phase_ == ScanPhase::COMPLETE || timer_complete);
  result.farthestMm = farthest_mm_;
  result.remainingSec = phase_ == ScanPhase::SCANNING ?
    std::max(0.0, kScanDurationSec - (now_sec - farthest_time_sec_)) :
    kScanDurationSec;
  result.message = active_ ? toString(result.phase) : "inactive";
  return result;
}

const char * toString(ScanPhase phase)
{
  switch (phase) {
    case ScanPhase::WAIT_STANDARD_RETURN: return "wait_standard_return";
    case ScanPhase::WAIT_STANDARD_EXIT: return "wait_standard_exit";
    case ScanPhase::SEEK_FARTHEST: return "seek_farthest";
    case ScanPhase::SCANNING: return "scanning";
    case ScanPhase::COMPLETE: return "complete";
  }
  return "unknown";
}

}  // namespace panthera_spectrometer_cell
