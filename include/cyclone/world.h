/*
 * Interface file for the rigid body world structure.
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
 * This file contains the definitions for a structure to hold any
 * number of rigid bodies, and to manage their simulation.
 */
#ifndef CYCLONE_WORLD_H
#define CYCLONE_WORLD_H

#include "body.h"
#include "contacts.h"

namespace cyclone {
    // > WorldStructureIntro
    /**
     * The world represents an independent simulation of physics.  It
     * keeps track of a set of rigid bodies, and provides the means to
     * update them all.
     */
    // > WorldStructure
    class World
    {
        // < WorldStructureIntro
        // ... other World data as before ...

        // < WorldStructure
        // > WorldStructureIntro
    public:
        typedef std::vector<RigidBody*> RigidBodies;
        
    protected:
        /**
         * Holds the rigid bodies being simulated.
         */
        RigidBodies bodies;

        // < WorldStructureIntro
        /**
         * True if the world should calculate the number of iterations
         * to give the contact resolver at each frame.
         */
        bool calculateIterations;

        /**
         * Holds the resolver for sets of contacts.
         */
        ContactResolver resolver;

        /**
         * Holds one contact generators in a linked list.
         */
        struct ContactGenRegistration
        {
            ContactGenerator *gen;
            ContactGenRegistration *next;
        };

        /**
         * Holds the head of the list of contact generators.
         */
        ContactGenRegistration *firstContactGen;

        /**
         * Holds an array of contacts, for filling by the contact
         * generators.
         */
        Contact *contacts;

        /**
         * Holds the maximum number of contacts allowed (i.e. the size
         * of the contacts array).
         */
        unsigned maxContacts;

    public:
        /**
         * Creates a new simulator that can handle up to the given
         * number of contacts per frame. You can also optionally give
         * a number of contact-resolution iterations to use. If you
         * don't give a number of iterations, then four times the
         * number of detected contacts will be used for each frame.
         */
        World(unsigned maxContacts, unsigned iterations=0);
        ~World();

        /**
         * Calls each of the registered contact generators to report
         * their contacts. Returns the number of generated contacts.
         */
        unsigned generateContacts();

        // > WorldRunPhysics
        /**
         * Processes all the physics for the world.
         */
        void runPhysics(real duration);
        // < WorldRunPhysics

        // > WorldStartFrame
        /**
         * Initialises the world for a simulation frame. This clears
         * the force and torque accumulators for bodies in the
         * world. After calling this, the bodies can have their forces
         * and torques for this frame added.
         */
        void startFrame();
        // < WorldStartFrame
        // > WorldStructure; WorldStructureIntro
    };
    // < WorldStructure; WorldStructureIntro

} // namespace cyclone

#endif // CYCLONE_PWORLD_H
