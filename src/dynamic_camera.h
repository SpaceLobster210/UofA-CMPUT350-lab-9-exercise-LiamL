#ifndef DYNAMIC_CAMERA_H
#define DYNAMIC_CAMERA_H

#include <iostream>
#include <limits>

#include "camera.h"

constexpr float INF = std::numeric_limits<float>::infinity();

// AABB in 1D
struct Interval {
    float min;
    float max;

    constexpr bool contains(const Interval& other) const {
        return (min <= other.min) && (max >= other.max);
    }

    constexpr bool contains(float x) const { return (min <= x) && (x <= max); }

    // signedDistance returns a positive smallest distance of x from the interval boundary
    // if it is not contained, else it returns a smallest (in magnitude) negative distance
    // from the interval boundary.
    constexpr float signedDistance(float x) const {
        if (contains(x)) {
            // 0 or negative
            return std::max(x - max, min - x);
        }
        // positive
        return std::max(x - max, min - x);
    }
};

struct DynamicCamera {
    void update(Camera& camera,                           // the camera to update
                const sf::Vector2f& trackedPosWorld,      // world coords of player to track
                const sf::Vector2f& trackedVelocityWorld  // world velocity of player to track
    ) {
        // TODO: (HINTS/outline)
        //// collect statistics on the player position and velocity in (normalized, if you like)
        //// window coordinates.
        sf::Vector2f trackedPosWindow = camera.worldToWindow(trackedPosWorld);
        sf::Vector2f trackedPosPercent = camera.worldToWindowPercent(trackedPosWorld) + camera.worldToWindowPercentDist({lookaheadState, 0.0f});
        float cameraDisplacementX = 0;

        //// Compute lookAhead distance in world coords, adjusting for if the player is moving left or right.
        sf::Vector2f newLookAhead = camera.windowPercentToWorldDist({lookAhead, 0.0f});
        if (leftLastFrame && trackedVelocityWorld.x == 0) {
            newLookAhead.x *= -1;
        }
        else if (trackedVelocityWorld.x < 0) {
            newLookAhead.x *= -1;
            leftLastFrame = true;
        }
        else { leftLastFrame = false; }

        //// Now update lookAhead state using an EMA over its previous state and the lookAhead world vector.
        lookaheadState = lookaheadState + alpha * (newLookAhead.x - lookaheadState);

        //// Then determine if the player is in the dead zone, soft rejection zone, or hard
        //// rejection zone. For each, perform EMA on the delta of the camera displacement (ignoring
        //// lookahead) appropriately.
        if (trackedPosPercent.x > softZoneX.max) {
            cameraDisplacementX = camera.windowPercentToWorldDist({trackedPosPercent.x - deadZoneX.max, 0}).x;
        }
        else if (trackedPosPercent.x < softZoneX.min) {
            cameraDisplacementX = camera.windowPercentToWorldDist({trackedPosPercent.x - deadZoneX.min, 0}).x;
        }
        else if (trackedPosPercent.x > deadZoneX.max || trackedPosPercent.x < deadZoneX.min) {
            cameraDisplacementX = alpha * (trackedPosWorld.x - cameraUnadjustedCenter.x);
        }
        else {
            cameraDisplacementX = 0.0f;
        }

        // Next, set the camera center by offsetting the cameraUnadjustedCenter, combining lookAhead.
        camera.setCameraCenter(cameraUnadjustedCenter + sf::Vector2f(cameraDisplacementX + lookaheadState, 0));

        // Finally update cameraUnadjustedCenter, but without lookahead.
        cameraUnadjustedCenter.x += cameraDisplacementX;
        
    }

    float alpha;  // alpha to use for damping of camera position, *as well as* for smoothing of
                  // lookahead offset.
    Interval deadZoneX;  // percentage interval along x-axis to use for dead zone in [0, 1]
    // percentage interval along x-axis to use for soft rejection zone in [0, 1]
    Interval softZoneX = {-INF, INF};
    // (everything beyond softZone is considered hard zone).
    float lookAhead;  // screen-percentage lookahead
    mutable sf::Vector2f
        cameraUnadjustedCenter;      // the camera world center from last frame, ignoring lookahead
    mutable float lookaheadState{};  // the EMA of the accumulated lookahead offset in world coords

    bool leftLastFrame = false; // tracks whether the player is facing left
};

#endif // DYNAMIC_CAMERA_H
