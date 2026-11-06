#include "routing/track_matcher.hpp"

#include "routing/base/followed_polyline.hpp"
#include "routing/routing_settings.hpp"

#include "platform/location.hpp"

#include "geometry/mercator.hpp"

#include "base/math.hpp"

#include <algorithm>
#include <cmath>

namespace routing
{
namespace
{
// Tolerate rounded projections at the entry and noise smaller than one metre.
double constexpr kTrackProgressToleranceM = 1.0;
// Allow sparse fixes without using arbitrarily old speed estimates.
double constexpr kMaxTrackFixGapSeconds = 120.0;
// Displacement underestimates along-track travel around bends.
double constexpr kTrackDisplacementAllowanceFactor = 2.0;
// Small GPS errors must not freeze matching on densely sampled recordings.
double constexpr kMinTrackProgressAllowanceM = 5.0;
// A stationary receiver's bearing does not establish traversal direction.
double constexpr kMinTrackBearingSpeedMpS = 0.5;

std::optional<m2::PointD> GetTrackDirection(location::GpsInfo const & info, m2::PointD const & movement,
                                            double displacementM)
{
  double const speed = info.HasSpeed() ? info.m_speed : 0.0;
  if (displacementM > kTrackProgressToleranceM && info.HasBearing() && speed >= kMinTrackBearingSpeedMpS)
  {
    double const angle = math::DegToRad(info.m_bearing);
    return m2::PointD(sin(angle), cos(angle));
  }
  if (displacementM >
          std::max(kMinTrackProgressAllowanceM, kTrackDisplacementAllowanceFactor * info.m_horizontalAccuracy) &&
      movement.SquaredLength() > 0.0)
    return movement / movement.Length();
  return std::nullopt;
}
}  // namespace

bool TrackMatcher::Update(FollowedPolyline & poly, location::GpsInfo const & info, RoutingSettings const & settings,
                          double entryDistanceM)
{
  ASSERT(poly.IsValid(), ());
  ASSERT(AlmostEqualAbs(poly.GetSegDistanceMeters().front(), entryDistanceM, 1e-3), (entryDistanceM));

  m2::RectD const rect = mercator::MetersToXY(info.m_longitude, info.m_latitude,
                                              std::max(settings.m_matchingThresholdM, info.m_horizontalAccuracy));
  double const tolerance = std::max(kTrackProgressToleranceM, info.m_horizontalAccuracy);
  auto const position = mercator::FromLatLon(info.m_latitude, info.m_longitude);
  double const currentDistance = poly.GetDistanceFromStartMeters();
  bool matched = false;
  // Outside the entry corridor, prefer the approach over nearby later laps.
  if (!m_joined && entryDistanceM > settings.m_matchingThresholdM)
  {
    matched = poly.UpdateTrackMatchingProjection(
        {.m_positionRect = rect, .m_maxProgressM = entryDistanceM, .m_toleranceM = tolerance});
  }
  if (!matched)
  {
    // Seed the first source window from the entry at the end of the approach.
    auto const referencePoint = m_lastFix ? m_lastFix->m_point : poly.GetPolyline().GetPoint(1);
    double const displacement = mercator::DistanceOnEarth(referencePoint, position);
    double const elapsed =
        m_lastFix ? std::clamp(info.m_timestamp - m_lastFix->m_timestamp, 0.0, kMaxTrackFixGapSeconds) : 0.0;
    double const speed = info.HasSpeed() ? info.m_speed : 0.0;
    // Displacement supplies the window when no previous timestamp is available.
    double const allowance = std::max(kTrackDisplacementAllowanceFactor * displacement, speed * elapsed) +
                             std::max(kMinTrackProgressAllowanceM, info.m_horizontalAccuracy);
    matched = poly.UpdateTrackMatchingProjection(
        {.m_positionRect = rect,
         .m_minProgressM = std::max(entryDistanceM, currentDistance - allowance),
         .m_maxProgressM = std::max(entryDistanceM, currentDistance) + allowance,
         .m_direction = m_lastFix ? GetTrackDirection(info, position - referencePoint, displacement) : std::nullopt,
         .m_toleranceM = tolerance,
         .m_forwardDirection = displacement > 0.0 ? std::optional(position - referencePoint) : std::nullopt,
         .m_isFirstFix = !m_lastFix});
  }
  if (matched)
  {
    m_joined = m_joined || poly.GetDistanceFromStartMeters() >= entryDistanceM - tolerance;
    m_lastFix = Fix{position, info.m_timestamp};
  }
  return matched;
}
}  // namespace routing
