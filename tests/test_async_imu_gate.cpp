// Copyright 2026 Ardia Robotics. SPDX-License-Identifier: MIT
#include <glim/odometry/async_odometry_estimation.hpp>
#include <atomic>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

class CountingBackend : public glim::OdometryEstimationBase {
public:
  explicit CountingBackend(bool requires) : needs_imu(requires) {}
  bool requires_imu() const override { return needs_imu; }
  void insert_imu(double, const Eigen::Vector3d&, const Eigen::Vector3d&) override { ++imus; }
  glim::EstimationFrame::ConstPtr insert_frame(
      const glim::PreprocessedFrame::Ptr&,
      std::vector<glim::EstimationFrame::ConstPtr>&) override {
    ++frames;
    return nullptr;
  }
#ifdef GLIM_USE_OPENCV
  void insert_image(double, const cv::Mat&) override { ++images; }
  std::atomic_int images{0};
#endif
  const bool needs_imu;
  std::atomic_int imus{0}, frames{0};
};

template<class Predicate>
bool until(Predicate predicate, std::chrono::milliseconds limit = 500ms) {
  const auto end = std::chrono::steady_clock::now() + limit;
  while (std::chrono::steady_clock::now() < end) {
    if (predicate()) return true;
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
}

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

void check(bool needs_imu) {
  auto backend = std::make_shared<CountingBackend>(needs_imu);
  glim::AsyncOdometryEstimation worker(backend, backend->requires_imu());
  worker.insert_imu(1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  require(until([&] { return backend->imus.load() == 1; }), "IMU was not forwarded");
  auto frame = std::make_shared<glim::PreprocessedFrame>();
  frame->stamp = 2.0;
  frame->scan_end_time = 2.1;
  worker.insert_frame(frame);
#ifdef GLIM_USE_OPENCV
  worker.insert_image(2.0, cv::Mat{});
#endif
  if (needs_imu) {
    require(!until([&] { return backend->frames.load() != 0; }, 100ms),
            "IMU-dependent backend consumed a frame without scan-end coverage");
#ifdef GLIM_USE_OPENCV
    require(backend->images.load() == 0, "IMU-dependent image lost its coverage gate");
#endif
    worker.insert_imu(2.2, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }
  require(until([&] { return backend->frames.load() == 1; }),
          "Backend that does not need IMU stalled behind an optional old IMU sample");
#ifdef GLIM_USE_OPENCV
  require(until([&] { return backend->images.load() == 1; }), "Image did not progress");
#endif
}

int main() {
  try {
    check(true);
    check(false);
    std::cout << "PASS: required IMU coverage retained; optional IMU cannot impose a gate\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
