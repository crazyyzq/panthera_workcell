#include <gtest/gtest.h>

#include "panthera_spectrometer_cell/LaserProcessing.h"

using panthera_spectrometer_cell::LaserFilter;
using panthera_spectrometer_cell::ScanDetectionTracker;
using panthera_spectrometer_cell::ScanPhase;
using panthera_spectrometer_cell::kScanDurationSec;

TEST(LaserFilter, RejectsRangeAndRequiresFreshStableWindow)
{
  LaserFilter filter(120.0, 280.0);
  filter.add(999.0, true, 0.1);
  EXPECT_FALSE(filter.snapshot(0.1).rawValid);

  const auto capture = filter.beginCapture();
  for (int i = 0; i < 19; ++i) {
    filter.add(162.2 + (i % 3 - 1) * 0.1, true, 0.2 * i + 0.2);
  }
  const auto result = filter.snapshot(4.0, capture);
  EXPECT_TRUE(result.stable);
  EXPECT_NEAR(result.stableMm, 162.2, 0.1);
  EXPECT_LE(result.spanMm, 1.0);
  EXPECT_FALSE(filter.snapshot(5.1, capture).rawValid);
}

TEST(LaserFilter, ClearsWindowsAfterGap)
{
  LaserFilter filter;
  for (int i = 0; i < 20; ++i) {
    filter.add(160.0, true, i * 0.2 + 0.1);
  }
  EXPECT_TRUE(filter.snapshot(4.0).stable);
  filter.add(170.0, true, 6.0);
  EXPECT_FALSE(filter.snapshot(6.0).filteredValid);
}

TEST(ScanDetection, WaitsForMotionThenCompletesFromLastMaximum)
{
  LaserFilter filter;
  ScanDetectionTracker tracker;
  tracker.start(162.0, 0.0);
  double time = 0.0;
  for (int i = 0; i < 10; ++i) {
    time += 0.2;
    filter.add(162.0, true, time);
    tracker.update(filter.snapshot(time), time);
  }
  EXPECT_EQ(tracker.status(time).phase, ScanPhase::WAIT_STANDARD_EXIT);

  for (int i = 0; i < 20; ++i) {
    time += 0.2;
    filter.add(168.0 + i, true, time);
    tracker.update(filter.snapshot(time), time);
  }
  for (int i = 0; i < 16; ++i) {
    time += 0.2;
    filter.add(188.0, true, time);
    tracker.update(filter.snapshot(time), time);
  }
  EXPECT_EQ(tracker.status(time).phase, ScanPhase::SCANNING);
  time += kScanDurationSec;
  filter.add(187.9, true, time);
  tracker.update(filter.snapshot(time), time);
  EXPECT_TRUE(tracker.status(time).done);
}

TEST(ScanDetection, ReturnToStandardBeforeFarthestResets)
{
  LaserFilter filter;
  ScanDetectionTracker tracker;
  tracker.start(160.0, 0.0);
  double time = 0.0;
  for (int i = 0; i < 10; ++i) {
    time += 0.2;
    filter.add(160.0, true, time);
    tracker.update(filter.snapshot(time), time);
  }
  ASSERT_EQ(tracker.status(time).phase, ScanPhase::WAIT_STANDARD_EXIT);
  for (int i = 0; i < 12; ++i) {
    time += 0.2;
    filter.add(170.0, true, time);
    tracker.update(filter.snapshot(time), time);
  }
  EXPECT_EQ(tracker.status(time).phase, ScanPhase::SEEK_FARTHEST);
  for (int i = 0; i < 12; ++i) {
    time += 0.2;
    filter.add(160.0, true, time);
    tracker.update(filter.snapshot(time), time);
  }
  EXPECT_EQ(tracker.status(time).phase, ScanPhase::WAIT_STANDARD_EXIT);
}

TEST(ScanDetection, InvalidGapCannotDeclareFarthest)
{
  LaserFilter filter;
  ScanDetectionTracker tracker;
  tracker.start(160.0, 0.0);
  double time = 0.0;
  for (int i = 0; i < 10; ++i) {
    time += 0.2;
    filter.add(160.0, true, time);
    tracker.update(filter.snapshot(time), time);
  }
  ASSERT_EQ(tracker.status(time).phase, ScanPhase::WAIT_STANDARD_EXIT);
  for (int i = 0; i < 12; ++i) {
    time += 0.2;
    filter.add(170.0, true, time);
    tracker.update(filter.snapshot(time), time);
  }
  ASSERT_EQ(tracker.status(time).phase, ScanPhase::SEEK_FARTHEST);

  time += 10.0;
  for (int i = 0; i < 5; ++i) {
    time += 0.2;
    filter.add(170.0, true, time);
    tracker.update(filter.snapshot(time), time);
  }
  EXPECT_EQ(tracker.status(time).phase, ScanPhase::SEEK_FARTHEST);
}

TEST(ScanDetection, RandomSamplePositionMustReturnToStandardBeforeScanStarts)
{
  LaserFilter filter;
  ScanDetectionTracker tracker;
  tracker.start(162.0, 0.0);
  double time = 0.0;
  for (int i = 0; i < 20; ++i) {
    time += 0.2;
    filter.add(214.0, true, time);
    tracker.update(filter.snapshot(time), time);
  }
  EXPECT_EQ(tracker.status(time).phase, ScanPhase::WAIT_STANDARD_RETURN);

  for (int i = 0; i < 12; ++i) {
    time += 0.2;
    filter.add(162.0, true, time);
    tracker.update(filter.snapshot(time), time);
  }
  EXPECT_EQ(tracker.status(time).phase, ScanPhase::WAIT_STANDARD_EXIT);
}
