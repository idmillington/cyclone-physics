/*
 * Implementation file for the particle class.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

#include <assert.h>
#include <cyclone/particle.h>

using namespace cyclone;


/*
 * --------------------------------------------------------------------------
 * FUNCTIONS DECLARED IN HEADER:
 * --------------------------------------------------------------------------
 */

void Particle::integrate(real duration)
{
    // We don't integrate things with zero mass.
    if (inverseMass <= 0.0f) return;

    assert(duration > 0.0);

    // Update linear position.
    position.addScaledVector(velocity, duration);

    // Work out the acceleration from the force
    Vector3 resultingAcc = acceleration;
    resultingAcc.addScaledVector(forceAccum, inverseMass);

    // Update linear velocity from the acceleration.
    velocity.addScaledVector(resultingAcc, duration);

    // Impose drag.
    velocity *= real_pow(damping, duration);

    // Clear the forces.
    clearAccumulator();
}



void Particle::setMass(const real mass)
{
    assert(mass != 0);
    Particle::inverseMass = ((real)1.0)/mass;
}

real Particle::getMass() const
{
    if (inverseMass == 0) {
        return REAL_MAX;
    } else {
        return ((real)1.0)/inverseMass;
    }
}

void Particle::setInverseMass(const real invMass)
{
    inverseMass = invMass;
}

real Particle::getInverseMass() const
{
    return inverseMass;
}

bool Particle::hasFiniteMass() const
{
    return inverseMass >= 0.0f;
}

void Particle::setDamping(const real damp)
{
    damping = damp;
}

real Particle::getDamping() const
{
    return damping;
}

void Particle::setPosition(const Vector3 &pos)
{
    position = pos;
}

void Particle::setPosition(const real x, const real y, const real z)
{
    position.x = x;
    position.y = y;
    position.z = z;
}

void Particle::getPosition(Vector3 *pos) const
{
    *pos = position;
}

Vector3 Particle::getPosition() const
{
    return position;
}

void Particle::setVelocity(const Vector3 &vel)
{
    velocity = vel;
}

void Particle::setVelocity(const real x, const real y, const real z)
{
    velocity.x = x;
    velocity.y = y;
    velocity.z = z;
}

void Particle::getVelocity(Vector3 *vel) const
{
    *vel = velocity;
}

Vector3 Particle::getVelocity() const
{
    return velocity;
}

void Particle::setAcceleration(const Vector3 &accel)
{
    acceleration = accel;
}

void Particle::setAcceleration(const real x, const real y, const real z)
{
    acceleration.x = x;
    acceleration.y = y;
    acceleration.z = z;
}

void Particle::getAcceleration(Vector3 *accel) const
{
    *accel = acceleration;
}

Vector3 Particle::getAcceleration() const
{
    return acceleration;
}

void Particle::clearAccumulator()
{
    forceAccum.clear();
}

void Particle::addForce(const Vector3 &force)
{
    forceAccum += force;
}
