#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstdint>

namespace falcon_cone_map_builder
{

static constexpr uint8_t COLOR_UNKNOWN    = 0;
static constexpr uint8_t COLOR_BLUE       = 1;
static constexpr uint8_t COLOR_YELLOW     = 2;
static constexpr uint8_t COLOR_ORANGE     = 3;
static constexpr uint8_t COLOR_BIG_ORANGE = 4;
static constexpr uint8_t NUM_COLORS       = 5;

struct Landmark
{
  uint32_t id = 0;
  uint8_t color = COLOR_UNKNOWN;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Identity();
  double confidence = 0.0;
  uint32_t observation_count = 0;
  double last_seen_sec = 0.0;
  std::array<uint32_t, NUM_COLORS> color_votes = {};

  uint8_t dominantColor() const
  {
    uint8_t best = COLOR_UNKNOWN;
    uint32_t best_count = 0;
    for (uint8_t i = 0; i < NUM_COLORS; ++i) {
      if (color_votes[i] > best_count) {
        best_count = color_votes[i];
        best = i;
      }
    }
    return best;
  }
};

}  // namespace falcon_cone_map_builder
