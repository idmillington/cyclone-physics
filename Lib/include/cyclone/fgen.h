/*
 * Interface file for the force generators.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under license. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software license.
 */

/**
 * @file
 *
 * This file contains the interface and sample force generators.
 */
#ifndef CYCLONE_FGEN_H
#define CYCLONE_FGEN_H

#include "body.h"
#include "pfgen.h"
#include <vector>

namespace cyclone {

    /**
     * A force generator can be asked to add a force to one or more
     * bodies.
     */
    class ForceGenerator
    {
    public:

        /**
         * Overload this in implementations of the interface to calculate
         * and update the force applied to the given rigid body.
         */
        virtual void updateForce(RigidBody *body, real duration) = 0;
    };

    /**
     * A force generator that applies a gravitational force. One instance
     * can be used for multiple rigid bodies.
     */
    class Gravity : public ForceGenerator
    {
        /** Holds the acceleration due to gravity. */
        Vector3 gravity;

    public:

        /** Creates the generator with the given acceleration. */
        Gravity(const Vector3 &gravity);

        /** Applies the gravitational force to the given rigid body. */
        virtual void updateForce(RigidBody *body, real duration);
    };

    /**
     * A force generator that applies a Spring force.
     */
    class Spring : public ForceGenerator
    {
        /**
         * The point of connection of the spring, in local
         * coordinates.
         */
        Vector3 connectionPoint;

        /**
         * The point of connection of the spring to the other object,
         * in that object's local coordinates.
         */
        Vector3 otherConnectionPoint;

        /** The particle at the other end of the spring. */
        RigidBody *other;

        /** Holds the sprint constant. */
        real springConstant;

        /** Holds the rest length of the spring. */
        real restLength;

    public:

        /** Creates a new spring with the given parameters. */
        Spring(const Vector3 &localConnectionPt,
               RigidBody *other,
               const Vector3 &otherConnectionPt,
               real springConstant,
               real restLength);

        /** Applies the spring force to the given rigid body. */
        virtual void updateForce(RigidBody *body, real duration);
    };

    /**
     * A force generator showing a three component explosion effect.
     * This force generator is intended to represent a single
     * explosion effect for multiple rigid bodies. The force generator
     * can also act as a particle force generator.
     */
    class Explosion : public ForceGenerator,
                      public ParticleForceGenerator
    {
        /**
         * Tracks how long the explosion has been in operation, used
         * for time-sensitive effects.
         */
        real timePassed;

    public:
        // Properties of the explosion, these are public because
        // there are so many and providing a suitable constructor
        // would be cumbersome:

        /**
         * The location of the detonation of the weapon.
         */
        Vector3 detonation;

        // ... Other Explosion code as before ...


        /**
         * The radius up to which objects implode in the first stage
         * of the explosion.
         */
        real implosionMaxRadius;

        /**
         * The radius within which objects don't feel the implosion
         * force. Objects near to the detonation aren't sucked in by
         * the air implosion.
         */
        real implosionMinRadius;

        /**
         * The length of time that objects spend imploding before the
         * concussion phase kicks in.
         */
        real implosionDuration;

        /**
         * The maximal force that the implosion can apply. This should
         * be relatively small to avoid the implosion pulling objects
         * through the detonation point and out the other side before
         * the concussion wave kicks in.
         */
        real implosionForce;

        /**
         * The speed that the shock wave is traveling, this is related
         * to the thickness below in the relationship:
         *
         * thickness >= speed * minimum frame duration
         */
        real shockwaveSpeed;

        /**
         * The shock wave applies its force over a range of distances,
         * this controls how thick. Faster waves require larger
         * thicknesses.
         */
        real shockwaveThickness;

        /**
         * This is the force that is applied at the very centre of the
         * concussion wave on an object that is stationary. Objects
         * that are in front or behind of the wavefront, or that are
         * already moving outwards, get proportionally less
         * force. Objects moving in towards the centre get
         * proportionally more force.
         */
         real peakConcussionForce;

         /**
          * The length of time that the concussion wave is active.
          * As the wave nears this, the forces it applies reduces.
          */
         real concussionDuration;

         /**
          * This is the peak force for stationary objects in
          * the centre of the convection chimney. Force calculations
          * for this value are the same as for peakConcussionForce.
          */
         real peakConvectionForce;

         /**
          * The radius of the chimney cylinder in the xz plane.
          */
         real chimneyRadius;

         /**
          * The maximum height of the chimney.
          */
         real chimneyHeight;

         /**
          * The length of time the convection chimney is active. Typically
          * this is the longest effect to be in operation, as the heat
          * from the explosion outlives the shock wave and implosion
          * itself.
          */
         real convectionDuration;

    public:
        /**
         * Creates a new explosion with sensible default values.
         */
        Explosion();

        /**
         * Calculates and applies the force that the explosion
         * has on the given rigid body.
         */
        virtual void updateForce(RigidBody * body, real duration);

        /**
         * Calculates and applies the force that the explosion has
         * on the given particle.
         */
        virtual void updateForce(Particle *particle, real duration) = 0;

    };

    /**
     * A force generator that applies an aerodynamic force.
     */
    class Aero : public ForceGenerator
    {
    protected:
        /**
         * Holds the aerodynamic tensor for the surface in body
         * space.
         */
        Matrix3 tensor;

        /**
         * Holds the relative position of the aerodynamic surface in
         * body coordinates.
         */
        Vector3 position;

        /**
         * Holds a pointer to a vector containing the windspeed of the
         * environment. This is easier than managing a separate
         * windspeed vector per generator and having to update it
         * manually as the wind changes.
         */
        const Vector3* windspeed;

    public:
        /**
         * Creates a new aerodynamic force generator with the
         * given properties.
         */
        Aero(const Matrix3 &tensor, const Vector3 &position,
             const Vector3 *windspeed);

        /**
         * Applies the force to the given rigid body.
         */
        virtual void updateForce(RigidBody *body, real duration);

    protected:
        /**
         * Uses an explicit tensor matrix to update the force on
         * the given rigid body. This is exactly the same as for updateForce
         * only it takes an explicit tensor.
         */
        void updateForceFromTensor(RigidBody *body, real duration,
                                   const Matrix3 &tensor);
    };

    /**
    * A force generator with a control aerodynamic surface. This
    * requires three inertia tensors, for the two extremes and
    * 'resting' position of the control surface.  The latter tensor is
    * the one inherited from the base class, the two extremes are
    * defined in this class.
    */
    class AeroControl : public Aero
    {
    protected:
        /**
         * The aerodynamic tensor for the surface, when the control is at
         * its maximum value.
         */
        Matrix3 maxTensor;

        /**
         * The aerodynamic tensor for the surface, when the control is at
         * its minimum value.
         */
        Matrix3 minTensor;

        /**
        * The current position of the control for this surface. This
        * should range between -1 (in which case the minTensor value
        * is used), through 0 (where the base-class tensor value is
        * used) to +1 (where the maxTensor value is used).
        */
        real controlSetting;

    private:
        /**
         * Calculates the final aerodynamic tensor for the current
         * control setting.
         */
        Matrix3 getTensor();

    public:
        /**
         * Creates a new aerodynamic control surface with the given
         * properties.
         */
        AeroControl(const Matrix3 &base,
                    const Matrix3 &min, const Matrix3 &max,
                    const Vector3 &position, const Vector3 *windspeed);

        /**
         * Sets the control position of this control. This * should
        range between -1 (in which case the minTensor value is *
        used), through 0 (where the base-class tensor value is used) *
        to +1 (where the maxTensor value is used). Values outside that
        * range give undefined results.
        */
        void setControl(real value);

        /**
         * Applies the force to the given rigid body.
         */
        virtual void updateForce(RigidBody *body, real duration);
    };

    /**
     * A force generator with an aerodynamic surface that can be
     * re-oriented relative to its rigid body. This derives the
     */
    class AngledAero : public Aero
    {
        /**
         * Holds the orientation of the aerodynamic surface relative
         * to the rigid body to which it is attached.
         */
        Quaternion orientation;

    public:
        /**
         * Creates a new aerodynamic surface with the given properties.
         */
        AngledAero(const Matrix3 &tensor, const Vector3 &position,
             const Vector3 *windspeed);

        /**
         * Sets the relative orientation of the aerodynamic surface,
         * relative to the rigid body it is attached to. Note that
         * this doesn't affect the point of connection of the surface
         * to the body.
         */
        void setOrientation(const Quaternion &quat);

        /**
         * Applies the force to the given rigid body.
         */
        virtual void updateForce(RigidBody *body, real duration);
    };

    /**
     * A force generator to apply a buoyant force to a rigid body.
     */
    class Buoyancy : public ForceGenerator
    {
        /**
         * The maximum submersion depth of the object before
         * it generates its maximum buoyancy force.
         */
        real maxDepth;

        /**
         * The volume of the object.
         */
        real volume;

        /**
         * The height of the water plane above y=0. The plane will be
         * parallel to the XZ plane.
         */
        real waterHeight;

        /**
         * The density of the liquid. Pure water has a density of
         * 1000kg per cubic meter.
         */
        real liquidDensity;

        /**
         * The centre of buoyancy of the rigid body, in body coordinates.
         */
        Vector3 centreOfBuoyancy;

    public:

        /** Creates a new buoyancy force with the given parameters. */
        Buoyancy(const Vector3 &cOfB,
            real maxDepth, real volume, real waterHeight,
            real liquidDensity = 1000.0f);

        /**
         * Applies the force to the given rigid body.
         */
        virtual void updateForce(RigidBody *body, real duration);
    };

    /**
    * Holds all the force generators and the bodies they apply to.
    */
    class ForceRegistry
    {
    protected:

        /**
        * Keeps track of one force generator and the body it
        * applies to.
        */
        struct ForceRegistration
        {
            RigidBody *body;
            ForceGenerator *fg;
        };

        /**
        * Holds the list of registrations.
        */
        typedef std::vector<ForceRegistration> Registry;
        Registry registrations;

    public:
        /**
        * Registers the given force generator to apply to the
        * given body.
        */
        void add(RigidBody* body, ForceGenerator *fg);

        /**
        * Removes the given registered pair from the registry.
        * If the pair is not registered, this method will have
        * no effect.
        */
        void remove(RigidBody* body, ForceGenerator *fg);

        /**
        * Clears all registrations from the registry. This will
        * not delete the bodies or the force generators
        * themselves, just the records of their connection.
        */
        void clear();

        /**
        * Calls all the force generators to update the forces of
        * their corresponding bodies.
        */
        void updateForces(real duration);
    };
}


#endif // CYCLONE_FGEN_H