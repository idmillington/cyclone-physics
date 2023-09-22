/*
 * Implementation file for the coarse collision detector.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

#include <cyclone/collide_coarse.h>

using namespace cyclone;

BoundingSphere::BoundingSphere(const Vector3 &centre, real radius)
{
    BoundingSphere::centre = centre;
    BoundingSphere::radius = radius;
}

BoundingSphere::BoundingSphere(const BoundingSphere &one,
                               const BoundingSphere &two)
{
    Vector3 centreOffset = two.centre - one.centre;
    real distance = centreOffset.squareMagnitude();
    real radiusDiff = two.radius - one.radius;

    // Check if the larger sphere encloses the small one
    if (radiusDiff*radiusDiff >= distance)
    {
        if (one.radius > two.radius)
        {
            centre = one.centre;
            radius = one.radius;
        }
        else
        {
            centre = two.centre;
            radius = two.radius;
        }
    }

    // Otherwise we need to work with partially
    // overlapping spheres
    else
    {
        distance = real_sqrt(distance);
        radius = (distance + one.radius + two.radius) * ((real)0.5);

        // The new centre is based on one's centre, moved towards
        // two's centre by an ammount proportional to the spheres'
        // radii.
        centre = one.centre;
        if (distance > 0)
        {
            centre += centreOffset * ((radius - one.radius)/distance);
        }
    }

}

bool BoundingSphere::overlaps(const BoundingSphere *other) const
{
    real distanceSquared = (centre - other->centre).squareMagnitude();
    return distanceSquared < (radius+other->radius)*(radius+other->radius);
}

real BoundingSphere::getGrowth(const BoundingSphere &other) const
{
    BoundingSphere newSphere(*this, other);

    // We return a value proportional to the change in surface
    // area of the sphere.
    return newSphere.radius*newSphere.radius - radius*radius;
}