// Copyright 2026 team3
// SPDX-License-Identifier: Apache-2.0

#include "hikrobot_camera_ros2/mock_backend.hpp"

#include <algorithm>
#include <chrono>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "hikrobot_camera_ros2/frame.hpp"

namespace hikrobot_camera_ros2
{
namespace
{
double steady_seconds()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

int64_t wall_clock_ns()
{
  using namespace std::chrono;
  return duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
}
}  // namespace

const std::vector<std::string> & MockBackend::patterns()
{
  static const std::vector<std::string> kPatterns{"gradient", "moving_bar", "checker"};
  return kPatterns;
}

MockBackend::MockBackend(MockBackendOptions options, Clock clock, TimeNs time_ns)
: options_(std::move(options)), clock_(std::move(clock)), time_ns_(std::move(time_ns))
{
  if (std::find(patterns().begin(), patterns().end(), options_.pattern) == patterns().end()) {
    throw std::invalid_argument("unknown pattern '" + options_.pattern + "'");
  }
  if (!(options_.fps > 0.0)) {throw std::invalid_argument("fps must be > 0");}
  validate_geometry(options_.width, options_.height, options_.encoding);
  if (!clock_) {clock_ = steady_seconds;}
  if (!time_ns_) {time_ns_ = wall_clock_ns;}
}

void MockBackend::open()
{
  validate_geometry(options_.width, options_.height, options_.encoding);

  scene_bytes_.clear();
  if (!options_.image_path.empty()) {
    // bgr8 or mono8 only: those are what the rest of the pipeline handles here.
    const cv::Mat raw = cv::imread(options_.image_path, cv::IMREAD_COLOR);
    if (raw.empty()) {
      throw std::runtime_error(
              "mock_image_path set but the file could not be read: " + options_.image_path);
    }
    cv::Mat resized;
    cv::resize(
      raw, resized, cv::Size(options_.width, options_.height), 0, 0, cv::INTER_AREA);
    if (options_.encoding == "mono8") {
      cv::Mat gray;
      cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);
      scene_bytes_.assign(gray.data, gray.data + gray.total() * gray.elemSize());
    } else if (options_.encoding == "bgr8") {
      scene_bytes_.assign(resized.data, resized.data + resized.total() * resized.elemSize());
    } else {
      throw std::runtime_error(
              "mock_image_path supports mono8 and bgr8 only, got '" + options_.encoding + "'");
    }
  }

  background_ = render_background();
  sequence_ = 0;
  fail_until_.reset();
  opened_ = true;
}

void MockBackend::close()
{
  opened_ = false;
}

std::optional<Frame> MockBackend::grab(double timeout_s)
{
  if (!opened_) {throw std::runtime_error("MockBackend::grab() called before open()");}

  // Data-driven failure injection: models an unplugged / unresponsive camera.
  if (options_.fail_after_frames.has_value() &&
    sequence_ >= static_cast<uint64_t>(*options_.fail_after_frames))
  {
    const double now = clock_();
    if (!fail_until_.has_value() && options_.fail_duration_s > 0.0) {
      fail_until_ = now + options_.fail_duration_s;
    }
    if (!fail_until_.has_value() || now < *fail_until_) {
      if (timeout_s > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(std::min(timeout_s, 0.05)));
      }
      return std::nullopt;
    }
    fail_until_.reset();
  }

  if (options_.raise_after_frames.has_value() &&
    sequence_ >= static_cast<uint64_t>(*options_.raise_after_frames))
  {
    throw std::runtime_error("injected mock backend failure");
  }

  Frame frame;
  frame.width = options_.width;
  frame.height = options_.height;
  frame.encoding = options_.encoding;
  frame.data = render(sequence_);
  frame.stamp_ns = time_ns_();
  frame.sequence = sequence_;
  frame.hardware_stamp_ns = std::nullopt;  // a mock has no device clock; never fake one
  ++sequence_;

  if (timeout_s > 0.0) {
    std::this_thread::sleep_for(
      std::chrono::duration<double>(std::min(timeout_s, 1.0 / options_.fps)));
  }
  return frame;
}

std::string MockBackend::describe() const
{
  std::ostringstream os;
  os << "backend=mock requires_hardware=false pattern=" << options_.pattern
     << " fps=" << options_.fps
     << " fail_after_frames="
     << (options_.fail_after_frames.has_value() ? std::to_string(*options_.fail_after_frames) : "none")
     << " raise_after_frames="
     << (options_.raise_after_frames.has_value() ? std::to_string(*options_.raise_after_frames) : "none");
  return os.str();
}

std::vector<uint8_t> MockBackend::render_background() const
{
  const std::size_t bpp = bytes_per_pixel(options_.encoding);
  const int w = options_.width;
  const int h = options_.height;
  std::vector<uint8_t> buffer(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * bpp, 0);

  if (options_.pattern == "checker") {
    const int tile = std::max(8, w / 16);
    for (int y = 0; y < h; ++y) {
      const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) * bpp;
      for (int x = 0; x < w; ++x) {
        const uint8_t value = (((x / tile) + (y / tile)) % 2 == 0) ? 255 : 32;
        const std::size_t index = row + static_cast<std::size_t>(x) * bpp;
        for (std::size_t c = 0; c < bpp; ++c) {buffer[index + c] = value;}
      }
    }
    return buffer;
  }

  for (int y = 0; y < h; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) * bpp;
    const int g = (255 * (y + 1)) / h;
    for (int x = 0; x < w; ++x) {
      const int ramp = (x * 255) / std::max(1, w - 1);
      const uint8_t value = static_cast<uint8_t>((ramp + g) / 2);
      const std::size_t index = row + static_cast<std::size_t>(x) * bpp;
      for (std::size_t c = 0; c < bpp; ++c) {buffer[index + c] = value;}
    }
  }
  return buffer;
}

std::vector<uint8_t> MockBackend::render(uint64_t sequence) const
{
  // A replayed scene is time-invariant by design: it makes raw-vs-rect and
  // latency measurements meaningful across different timestamps.
  if (!scene_bytes_.empty()) {return scene_bytes_;}

  std::vector<uint8_t> buffer = background_;
  if (options_.pattern != "moving_bar") {return buffer;}

  const std::size_t bpp = bytes_per_pixel(options_.encoding);
  const int w = options_.width;
  const int h = options_.height;
  const int bar_width = std::max(2, w / 20);
  const uint64_t stride = static_cast<uint64_t>(std::max(1, w / 64));
  const int x0 = static_cast<int>((sequence * stride) % static_cast<uint64_t>(w));
  const int y_from = h / 8;
  const int y_to = h - h / 8;

  for (int y = y_from; y < y_to; ++y) {
    const std::size_t row = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) * bpp;
    for (int dx = 0; dx < bar_width; ++dx) {
      const int x = (x0 + dx) % w;
      const std::size_t index = row + static_cast<std::size_t>(x) * bpp;
      const uint8_t value = (dx < bar_width - 1) ? 255 : 0;
      for (std::size_t c = 0; c < bpp; ++c) {buffer[index + c] = value;}
    }
  }
  return buffer;
}

}  // namespace hikrobot_camera_ros2
