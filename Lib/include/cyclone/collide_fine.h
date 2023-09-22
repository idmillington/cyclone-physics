/*
 * Interface file for the fine grained collision detection system.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

/**
 * @file
 *
 * This file contains the fine grained collision detection system.
 * It is used to return contacts between pairs of primitives.
 *
 * There are two groups of tests in this file. Intersection tests
 * use the fastest separating axis method to check if two objects
 * intersect, and the collision tests generate the contacts. The
 * collision tests typically use the intersection tests as an early
 * out.
 */
#ifndef CYCLONE_COLLISION_FINE_H
#define CYCLONE_COLLISION_FINE_H

#include "contacts.h"

namespace cyclone {

    // Forward declarations of primitive friends
    class IntersectionTests;
    class CollisionDetector;

    /**
     * Represents a primitive to detect collisions against.
     */
    class CollisionPrimitive
    {
    public:
        /**
         * This class exists to help the collision detector
         * and intersection routines, so they should have
         * access to its data.
         */
        friend class IntersectionTests;
        friend class CollisionDetector;

        /**
         * The rigid body that is represented by this primitive.
         */
        RigidBody * body;

        /**
         * The offset of this primitive from the given rigid body.
         */
        Matrix4 offset;

        /**
         * Calculates the internals for the primitive.
         */
        void calculateInternals();

        /**
         * This is a convenience function to allow access to the
         * axis vectors in the transform for this primitive.
         */
        Vector3 getAxis(unsigned index) const
        {
            return transform.getAxisVector(index);
        }

        /**
         * Returns the resultant transform of the primitive, calculated from
         * the combined offset of the primitive and the transform
         * (orientation + position) of the rigid body to which it is
         * attached.
         */
        const Matrix4& getTransform() const
        {
            return transform;
        }


    protected:
        /**
         * The resultant transform of the primitive. This is
         * calculated by combining the offset of the primitive
         * with the transform of the rigid body.
         */
        Matrix4 transform;
    };

    /**
     * Represents a rigid body that can be treated as a sphere
     * for collision detection.
     */
    class CollisionSphere : public CollisionPrimitive
    {
    public:
        /**
         * The radius of the sphere.
         */
        real radius;
    };

    /**
     * The plane is not a primitive: it doesn't represent another
     * rigid body. It is used for contacts with the immovable
     * world geometry.
     */
    class CollisionPlane
    {
    public:
        /**
         * The plane normal
         */
        Vector3 direction;

        /**
         * The distance of the plane from the origin.
         */
        real offset;
    };

    /**
     * Represents a rigid body that can be treated as an aligned bounding
     * box for collision detection.
     */
    class CollisionBox : public CollisionPrimitive
    {
    public:
        /**
         * Holds the half-sizes of the box along each of its local axes.
         */
        Vector3 halfSize;
    };

    /**
     * A wrapper class that holds fast intersection tests. These
     * can be used to drive the coarse collision detection system or
     * as an early out in the full collision tests below.
     */
    class IntersectionTests
    {
    public:

        static bool sphereAndHalfSpace(
            const CollisionSphere &sphere,
            const CollisionPlane &plane);

        static bool sphereAndSphere(
            const CollisionSphere &one,
            const CollisionSphere &two);

        static bool boxAndBox(
            const CollisionBox &one,
            const CollisionBox &two);

        /**
         * Does an intersection test on an arbitrarily aligned box and a
         * half-space.
         *
         * The box is given as a transform matrix, including
         * position, and a vector of half-sizes for the extend of the
         * box along each local axis.
         *
         * The half-space is given as a direction (i.e. unit) vector and the
         * offset of the limiting plane from the origin, along the given
         * direction.
         */
        static bool boxAndHalfSpace(
            const CollisionBox &box,
            const CollisionPlane &plane);
    };


    /**
     * A helper structure that contains information for the detector to use
     * in building its contact data.
     */
    struct CollisionData
    {
        /**
         * Holds the base of the collision data: the first contact
         * in the array. This is used so that the contact pointer (below)
         * can be incremented each time a contact is detected, while
         * this pointer points to the first contact found.
         */
        Contact *contactArray;

        /** Holds the contact array to write into. */
        Contact *contacts;

        /** Holds the maximum number of contacts the array can take. */
        int contactsLeft;

        /** Holds the number of contacts found so far. */
        unsigned contactCount;

        /** Holds the friction value to write into any collisions. */
        real friction;

        /** Holds the restitution value to write into any collisions. */
        real restitution;

        /**
         * Holds the collision tolerance, even uncolliding objects this
         * close should have collisions generated.
         */
        real tolerance;

        /**
         * Checks if there are more contacts available in the contact
         * data.
         */
        bool hasMoreContacts()
        {
            return contactsLeft > 0;
        }

        /**
         * Resets the data so that it has no used contacts recorded.
         */
        void reset(unsigned maxContacts)
        {
            contactsLeft = maxContacts;
            contactCount = 0;
            contacts = contactArray;
        }

        /**
         * Notifies the data that the given number of contacts have
         * been added.
         */
        void addContacts(unsigned count)
        {
            // Reduce the number of contacts remaining, add number used
            contactsLeft -= count;
            contactCount += count;

            // Move the array forward
            contacts += count;
        }
    };

    /**
     * A wrapper class that holds the fine grained collision detection
     * routines.
     *
     * Each of the functions has the same format: it takes the details
     * of two objects, and a pointer to a contact array to fill. It
     * returns the number of contacts it wrote into the array.
     */
    class CollisionDetector
    {
    public:

        static unsigned sphereAndHalfSpace(
            const CollisionSphere &sphere,
            const CollisionPlane &plane,
            CollisionData *data
            );

        static unsigned sphereAndTruePlane(
            const CollisionSphere &sphere,
            const CollisionPlane &plane,
            CollisionData *data
            );

        static unsigned sphereAndSphere(
            const CollisionSphere &one,
            const CollisionSphere &two,
            CollisionData *data
            );

        /**
         * Does a collision test on a collision box and a plane representing
         * a half-space (i.e. the normal of the plane
         * points out of the half-space).
         */
        static unsigned boxAndHalfSpace(
            const CollisionBox &box,
            const CollisionPlane &plane,
            CollisionData *data
            );

        static unsigned boxAndBox(
            const CollisionBox &one,
            const CollisionBox &two,
            CollisionData *data
            );

        static unsigned boxAndPoint(
            const CollisionBox &box,
            const Vector3 &point,
            CollisionData *data
            );

        static unsigned boxAndSphere(
            const CollisionBox &box,
            const CollisionSphere &sphere,
            CollisionData *data
            );
    };



} // namespace cyclone

#endif // CYCLONE_COLLISION_FINE_H
