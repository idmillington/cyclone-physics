/*
 * Interface file for the particle links.
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
 * This file contains classes representing the connections between
 * particles.
 */
#ifndef CYCLONE_PLINKS_H
#define CYCLONE_PLINKS_H

#include "pcontacts.h"

namespace cyclone {

    /**
     * Links connect two particles together, generating a contact if
     * they violate the constraints of their link. It is used as a
     * base class for cables and rods, and could be used as a base
     * class for springs with a limit to their extension..
     */
    class ParticleLink : public ParticleContactGenerator
    {
    public:
        /**
         * Holds the pair of particles that are connected by this link.
         */
        Particle* particle[2];

    protected:
        /**
         * Returns the current length of the link.
         */
        real currentLength() const;

    public:
        /**
         * Geneates the contacts to keep this link from being
         * violated. This class can only ever generate a single
         * contact, so the pointer can be a pointer to a single
         * element, the limit parameter is assumed to be at least one
         * (zero isn't valid) and the return value is either 0, if the
         * cable wasn't over-extended, or one if a contact was needed.
         *
         * NB: This method is declared in the same way (as pure
         * virtual) in the parent class, but is replicated here for
         * documentation purposes.
         */
        virtual unsigned addContact(ParticleContact *contact,
                                    unsigned limit) const = 0;
    };

    /**
     * Cables link a pair of particles, generating a contact if they
     * stray too far apart.
     */
    class ParticleCable : public ParticleLink
    {
    public:
        /**
         * Holds the maximum length of the cable.
         */
        real maxLength;

        /**
         * Holds the restitution (bounciness) of the cable.
         */
        real restitution;

    public:
        /**
         * Fills the given contact structure with the contact needed
         * to keep the cable from over-extending.
         */
        virtual unsigned addContact(ParticleContact *contact,
                                    unsigned limit) const;
    };

    /**
     * Rods link a pair of particles, generating a contact if they
     * stray too far apart or too close.
     */
    class ParticleRod : public ParticleLink
    {
    public:
        /**
         * Holds the length of the rod.
         */
        real length;

    public:
        /**
         * Fills the given contact structure with the contact needed
         * to keep the rod from extending or compressing.
         */
        virtual unsigned addContact(ParticleContact *contact,
                                     unsigned limit) const;
    };

    /**
    * Constraints are just like links, except they connect a particle to
    * an immovable anchor point.
    */
    class ParticleConstraint : public ParticleContactGenerator
    {
    public:
        /**
        * Holds the particles connected by this constraint.
        */
        Particle* particle;

        /**
         * The point to which the particle is anchored.
         */
        Vector3 anchor;

    protected:
        /**
        * Returns the current length of the link.
        */
        real currentLength() const;

    public:
        /**
        * Geneates the contacts to keep this link from being
        * violated. This class can only ever generate a single
        * contact, so the pointer can be a pointer to a single
        * element, the limit parameter is assumed to be at least one
        * (zero isn't valid) and the return value is either 0, if the
        * cable wasn't over-extended, or one if a contact was needed.
        *
        * NB: This method is declared in the same way (as pure
        * virtual) in the parent class, but is replicated here for
        * documentation purposes.
        */
        virtual unsigned addContact(ParticleContact *contact,
            unsigned limit) const = 0;
    };

    /**
    * Cables link a particle to an anchor point, generating a contact if they
    * stray too far apart.
    */
    class ParticleCableConstraint : public ParticleConstraint
    {
    public:
        /**
        * Holds the maximum length of the cable.
        */
        real maxLength;

        /**
        * Holds the restitution (bounciness) of the cable.
        */
        real restitution;

    public:
        /**
        * Fills the given contact structure with the contact needed
        * to keep the cable from over-extending.
        */
        virtual unsigned addContact(ParticleContact *contact,
            unsigned limit) const;
    };

    /**
    * Rods link a particle to an anchor point, generating a contact if they
    * stray too far apart or too close.
    */
    class ParticleRodConstraint : public ParticleConstraint
    {
    public:
        /**
        * Holds the length of the rod.
        */
        real length;

    public:
        /**
        * Fills the given contact structure with the contact needed
        * to keep the rod from extending or compressing.
        */
        virtual unsigned addContact(ParticleContact *contact,
            unsigned limit) const;
    };
} // namespace cyclone

#endif // CYCLONE_CONTACTS_H