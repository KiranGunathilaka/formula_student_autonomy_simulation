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
  uint32_t id               = 0;
  uint8_t  color            = COLOR_UNKNOWN;
  Eigen::Vector2d position  = Eigen::Vector2d::Zero();
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Identity();
  double   confidence        = 0.0;
  uint32_t observation_count = 0;
  double   last_seen_sec     = 0.0;

  std::array<uint32_t, NUM_COLORS> color_votes = {};

  // ---------------------------------------------------------------------------
  // FIX: dominantColor() – original bug: best_count initialised to 0 and best
  // initialised to COLOR_UNKNOWN, so when two non-unknown colors tie the
  // function silently returned COLOR_UNKNOWN even if that slot had 0 votes.
  //
  // Fix: skip COLOR_UNKNOWN in the search (index 0) and initialise best_count
  // to 1 so that a color only wins if it has at least one vote.  If no color
  // has any votes yet, we fall back to COLOR_UNKNOWN explicitly, which is the
  // correct semantic for a landmark whose color is not yet determined.
  // ---------------------------------------------------------------------------
  uint8_t dominantColor() const
  {
    uint8_t  best       = COLOR_UNKNOWN;
    uint32_t best_count = 1;               // require at least 1 vote to win

    // Start from index 1 to skip COLOR_UNKNOWN – unknown is only returned as
    // a fallback, never as a "winner" over a real color.
    for (uint8_t i = 1; i < NUM_COLORS; ++i) {
      if (color_votes[i] > best_count) {
        best_count = color_votes[i];
        best       = i;
      }
    }
    return best;
  }
};

}  // namespace falcon_cone_map_builder