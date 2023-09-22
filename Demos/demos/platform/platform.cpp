/*
 * The platform demo.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

#include <cyclone/cyclone.h>
#include "../ogl_headers.h"
#include "../app.h"
#include "../timing.h"

#include <stdio.h>
#include <cassert>

#define ROD_COUNT 15

#define BASE_MASS 1
#define EXTRA_MASS 10

/**
 * The main demo class definition.
 */
class PlatformDemo : public MassAggregateApplication
{
    cyclone::ParticleRod *rods;

    cyclone::Vector3 massPos;
    cyclone::Vector3 massDisplayPos;

    /**
     * Updates particle masses to take into account the mass
     * that's on the platform.
     */
    void updateAdditionalMass();

public:
    /** Creates a new demo object. */
    PlatformDemo();
    virtual ~PlatformDemo();

    /** Returns the window title for the demo. */
    virtual const char* getTitle();

    /** Display the particles. */
    virtual void display();

    /** Update the particle positions. */
    virtual void update();

    /** Handle a key press. */
    virtual void key(unsigned char key);
};

// Method definitions
PlatformDemo::PlatformDemo()
:
MassAggregateApplication(6), rods(0),
massPos(0,0,0.5f)
{
    // Create the masses and connections.
    particleArray[0].setPosition(0,0,1);
    particleArray[1].setPosition(0,0,-1);
    particleArray[2].setPosition(-3,2,1);
    particleArray[3].setPosition(-3,2,-1);
    particleArray[4].setPosition(4,2,1);
    particleArray[5].setPosition(4,2,-1);
    for (unsigned i = 0; i < 6; i++)
    {
        particleArray[i].setMass(BASE_MASS);
        particleArray[i].setVelocity(0,0,0);
        particleArray[i].setDamping(0.9f);
        particleArray[i].setAcceleration(cyclone::Vector3::GRAVITY);
        particleArray[i].clearAccumulator();
    }

    rods = new cyclone::ParticleRod[ROD_COUNT];

    rods[0].particle[0] = &particleArray[0];
    rods[0].particle[1] = &particleArray[1];
    rods[0].length = 2;
    rods[1].particle[0] = &particleArray[2];
    rods[1].particle[1] = &particleArray[3];
    rods[1].length = 2;
    rods[2].particle[0] = &particleArray[4];
    rods[2].particle[1] = &particleArray[5];
    rods[2].length = 2;

    rods[3].particle[0] = &particleArray[2];
    rods[3].particle[1] = &particleArray[4];
    rods[3].length = 7;
    rods[4].particle[0] = &particleArray[3];
    rods[4].particle[1] = &particleArray[5];
    rods[4].length = 7;

    rods[5].particle[0] = &particleArray[0];
    rods[5].particle[1] = &particleArray[2];
    rods[5].length = 3.606;
    rods[6].particle[0] = &particleArray[1];
    rods[6].particle[1] = &particleArray[3];
    rods[6].length = 3.606;

    rods[7].particle[0] = &particleArray[0];
    rods[7].particle[1] = &particleArray[4];
    rods[7].length = 4.472;
    rods[8].particle[0] = &particleArray[1];
    rods[8].particle[1] = &particleArray[5];
    rods[8].length = 4.472;

    rods[9].particle[0] = &particleArray[0];
    rods[9].particle[1] = &particleArray[3];
    rods[9].length = 4.123;
    rods[10].particle[0] = &particleArray[2];
    rods[10].particle[1] = &particleArray[5];
    rods[10].length = 7.28;
    rods[11].particle[0] = &particleArray[4];
    rods[11].particle[1] = &particleArray[1];
    rods[11].length = 4.899;
    rods[12].particle[0] = &particleArray[1];
    rods[12].particle[1] = &particleArray[2];
    rods[12].length = 4.123;
    rods[13].particle[0] = &particleArray[3];
    rods[13].particle[1] = &particleArray[4];
    rods[13].length = 7.28;
    rods[14].particle[0] = &particleArray[5];
    rods[14].particle[1] = &particleArray[0];
    rods[14].length = 4.899;

    for (unsigned i = 0; i < ROD_COUNT; i++)
    {
        world.getContactGenerators().push_back(&rods[i]);
    }

    updateAdditionalMass();
}

PlatformDemo::~PlatformDemo()
{
    if (rods) delete[] rods;
}

void PlatformDemo::updateAdditionalMass()
{
    for (unsigned i = 2; i < 6; i++)
    {
        particleArray[i].setMass(BASE_MASS);
    }

    // Find the coordinates of the mass as an index and proportion
    cyclone::real xp = massPos.x;
    if (xp < 0) xp = 0;
    if (xp > 1) xp = 1;

    cyclone::real zp = massPos.z;
    if (zp < 0) zp = 0;
    if (zp > 1) zp = 1;

    // Calculate where to draw the mass
    massDisplayPos.clear();

    // Add the proportion to the correct masses
    particleArray[2].setMass(BASE_MASS + EXTRA_MASS*(1-xp)*(1-zp));
    massDisplayPos.addScaledVector(
        particleArray[2].getPosition(), (1-xp)*(1-zp)
        );

    if (xp > 0)
    {
        particleArray[4].setMass(BASE_MASS + EXTRA_MASS*xp*(1-zp));
        massDisplayPos.addScaledVector(
            particleArray[4].getPosition(), xp*(1-zp)
            );

        if (zp > 0)
        {
            particleArray[5].setMass(BASE_MASS + EXTRA_MASS*xp*zp);
            massDisplayPos.addScaledVector(
                particleArray[5].getPosition(), xp*zp
                );
        }
    }
    if (zp > 0)
    {
        particleArray[3].setMass(BASE_MASS + EXTRA_MASS*(1-xp)*zp);
        massDisplayPos.addScaledVector(
            particleArray[3].getPosition(), (1-xp)*zp
            );
    }
}

void PlatformDemo::display()
{
    MassAggregateApplication::display();

    glBegin(GL_LINES);
    glColor3f(0,0,1);
    for (unsigned i = 0; i < ROD_COUNT; i++)
    {
        cyclone::Particle **particles = rods[i].particle;
        const cyclone::Vector3 &p0 = particles[0]->getPosition();
        const cyclone::Vector3 &p1 = particles[1]->getPosition();
        glVertex3f(p0.x, p0.y, p0.z);
        glVertex3f(p1.x, p1.y, p1.z);
    }
    glEnd();

    glColor3f(1,0,0);
    glPushMatrix();
    glTranslatef(massDisplayPos.x, massDisplayPos.y+0.25f, massDisplayPos.z);
    glutSolidSphere(0.25f, 20, 10);
    glPopMatrix();
}

void PlatformDemo::update()
{
    MassAggregateApplication::update();

    updateAdditionalMass();
}

const char* PlatformDemo::getTitle()
{
    return "Cyclone > Platform Demo";
}

void PlatformDemo::key(unsigned char key)
{
    switch(key)
    {
    case 'w': case 'W':
        massPos.z += 0.1f;
        if (massPos.z > 1.0f) massPos.z = 1.0f;
        break;
    case 's': case 'S':
        massPos.z -= 0.1f;
        if (massPos.z < 0.0f) massPos.z = 0.0f;
        break;
    case 'a': case 'A':
        massPos.x -= 0.1f;
        if (massPos.x < 0.0f) massPos.x = 0.0f;
        break;
    case 'd': case 'D':
        massPos.x += 0.1f;
        if (massPos.x > 1.0f) massPos.x = 1.0f;
        break;

    default:
        MassAggregateApplication::key(key);
    }
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
    return new PlatformDemo();
}