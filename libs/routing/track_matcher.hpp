#pragma once

#include "geometry/point2d.hpp"

#include <optional>

namespace location
{
class GpsInfo;
}

namespace routing
{
class FollowedPolyline;
struct RoutingSettings;

// Constrains source progress using entry reach and the last accepted raw fix.
// Track geometry starts with an approach segment, including an empty approach.
class TrackMatcher
{
public:
  bool Update(FollowedPolyline & poly, location::GpsInfo const & info, RoutingSettings const & settings,
              double entryDistanceM);

private:
  struct Fix
  {
    m2::PointD m_point;
    double m_timestamp = 0.0;
  };
  bool m_joined = false;
  std::optional<Fix> m_lastFix;
};
}  // namespace routing
