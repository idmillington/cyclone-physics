/*
 * The fracture demo.
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

#define MAX_BLOCKS 9

cyclone::Random global_random;

class Block : public cyclone::CollisionBox
{
public:
    bool exists;

    Block()
    :
    exists(false)
    {
        body = new cyclone::RigidBody();
    }

    ~Block()
    {
        delete body;
    }

    /** Draws the block. */
    void render()
    {
        // Get the OpenGL transformation
        GLfloat mat[16];
        body->getGLTransform(mat);

        if (body->getAwake()) glColor3f(1.0f,0.7f,0.7f);
        else glColor3f(0.7f,0.7f,1.0f);

        glPushMatrix();
        glMultMatrixf(mat);
        glScalef(halfSize.x*2, halfSize.y*2, halfSize.z*2);
        glutSolidCube(1.0f);
        glPopMatrix();
    }

    /** Sets the block to a specific location. */
    void setState(const cyclone::Vector3 &position,
                  const cyclone::Quaternion &orientation,
                  const cyclone::Vector3 &extents,
                  const cyclone::Vector3 &velocity)
    {
        body->setPosition(position);
        body->setOrientation(orientation);
        body->setVelocity(velocity);
        body->setRotation(cyclone::Vector3(0,0,0));
        halfSize = extents;

        cyclone::real mass = halfSize.x * halfSize.y * halfSize.z * 8.0f;
        body->setMass(mass);

        cyclone::Matrix3 tensor;
        tensor.setBlockInertiaTensor(halfSize, mass);
        body->setInertiaTensor(tensor);

        body->setLinearDamping(0.95f);
        body->setAngularDamping(0.8f);
        body->clearAccumulators();
        body->setAcceleration(0,-10.0f,0);

        //body->setCanSleep(false);
        body->setAwake();

        body->calculateDerivedData();
    }

    /**
     * Calculates and sets the mass and inertia tensor of this block,
     * assuming it has the given constant density.
     */
    void calculateMassProperties(cyclone::real invDensity)
    {
        // Check for infinite mass
        if (invDensity <= 0)
        {
            // Just set zeros for both mass and inertia tensor
            body->setInverseMass(0);
            body->setInverseInertiaTensor(cyclone::Matrix3());
        }
        else
        {
            // Otherwise we need to calculate the mass
            cyclone::real volume = halfSize.magnitude() * 2.0;
            cyclone::real mass = volume / invDensity;

            body->setMass(mass);

            // And calculate the inertia tensor from the mass and size
            mass *= 0.333f;
            cyclone::Matrix3 tensor;
            tensor.setInertiaTensorCoeffs(
                mass * halfSize.y*halfSize.y + halfSize.z*halfSize.z,
                mass * halfSize.y*halfSize.x + halfSize.z*halfSize.z,
                mass * halfSize.y*halfSize.x + halfSize.z*halfSize.y
                );
            body->setInertiaTensor(tensor);
        }

    }

    /**
     * Performs the division of the given block into four, writing the
     * eight new blocks into the given blocks array. The blocks array can be
     * a pointer to the same location as the target pointer: since the
     * original block is always deleted, this effectively reuses its storage.
     * The algorithm is structured to allow this reuse.
     */
    void divideBlock(const cyclone::Contact& contact,
        Block* target, Block* blocks)
    {
        // Find out if we're block one or two in the contact structure, and
        // therefore what the contact normal is.
        cyclone::Vector3 normal = contact.contactNormal;
        cyclone::RigidBody *body = contact.body[0];
        if (body != target->body)
        {
            normal.invert();
            body = contact.body[1];
        }

        // Work out where on the body (in body coordinates) the contact is
        // and its direction.
        cyclone::Vector3 point = body->getPointInLocalSpace(contact.contactPoint);
        normal = body->getDirectionInLocalSpace(normal);

        // Work out the centre of the split: this is the point coordinates
        // for each of the axes perpendicular to the normal, and 0 for the
        // axis along the normal.
        point = point - normal * (point * normal);

        // Take a copy of the half size, so we can create the new blocks.
        cyclone::Vector3 size = target->halfSize;

        // Take a copy also of the body's other data.
        cyclone::RigidBody tempBody;
        tempBody.setPosition(body->getPosition());
        tempBody.setOrientation(body->getOrientation());
        tempBody.setVelocity(body->getVelocity());
        tempBody.setRotation(body->getRotation());
        tempBody.setLinearDamping(body->getLinearDamping());
        tempBody.setAngularDamping(body->getAngularDamping());
        tempBody.setInverseInertiaTensor(body->getInverseInertiaTensor());
        tempBody.calculateDerivedData();

        // Remove the old block
        target->exists = false;

        // Work out the inverse density of the old block
        cyclone::real invDensity =
            halfSize.magnitude()*8 * body->getInverseMass();

        // Now split the block into eight.
        for (unsigned i = 0; i < 8; i++)
        {
            // Find the minimum and maximum extents of the new block
            // in old-block coordinates
            cyclone::Vector3 min, max;
            if ((i & 1) == 0) {
                min.x = -size.x;
                max.x = point.x;
            } else {
                min.x = point.x;
                max.x = size.x;
            }
            if ((i & 2) == 0) {
                min.y = -size.y;
                max.y = point.y;
            } else {
                min.y = point.y;
                max.y = size.y;
            }
            if ((i & 4) == 0) {
                min.z = -size.z;
                max.z = point.z;
            } else {
                min.z = point.z;
                max.z = size.z;
            }

            // Get the origin and half size of the block, in old-body
            // local coordinates.
            cyclone::Vector3 halfSize = (max - min) * 0.5f;
            cyclone::Vector3 newPos = halfSize + min;

            // Convert the origin to world coordinates.
            newPos = tempBody.getPointInWorldSpace(newPos);

            // Work out the direction to the impact.
            cyclone::Vector3 direction = newPos - contact.contactPoint;
            direction.normalise();

            // Set the body's properties (we assume the block has a body
            // already that we're going to overwrite).
            blocks[i].body->setPosition(newPos);
            blocks[i].body->setVelocity(tempBody.getVelocity() + direction * 10.0f);
            blocks[i].body->setOrientation(tempBody.getOrientation());
            blocks[i].body->setRotation(tempBody.getRotation());
            blocks[i].body->setLinearDamping(tempBody.getLinearDamping());
            blocks[i].body->setAngularDamping(tempBody.getAngularDamping());
            blocks[i].body->setAwake(true);
            blocks[i].body->setAcceleration(cyclone::Vector3::GRAVITY);
            blocks[i].body->clearAccumulators();
            blocks[i].body->calculateDerivedData();
            blocks[i].offset = cyclone::Matrix4();
            blocks[i].exists = true;
            blocks[i].halfSize = halfSize;

            // Finally calculate the mass and inertia tensor of the new block
            blocks[i].calculateMassProperties(invDensity);
        }
    }
};

/**
 * The main demo class definition.
 */
class FractureDemo : public RigidBodyApplication
{
    /** Tracks if a block has been hit. */
    bool hit;
    bool ball_active;
    unsigned fracture_contact;

    /** Handle random numbers. */
    cyclone::Random random;

    /** Holds the bodies. */
    Block blocks[MAX_BLOCKS];

    /** Holds the projectile. */
    cyclone::CollisionSphere ball;

    /** Processes the contact generation code. */
    virtual void generateContacts();

    /** Processes the objects in the simulation forward in time. */
    virtual void updateObjects(cyclone::real duration);

    /** Resets the position of all the blocks. */
    virtual void reset();

    /** Processes the physics. */
    virtual void update();

public:
    /** Creates a new demo object. */
    FractureDemo();

    /** Returns the window title for the demo. */
    virtual const char* getTitle();

    /** Display the particle positions. */
    virtual void display();
};

// Method definitions
FractureDemo::FractureDemo()
    :
    RigidBodyApplication()
{
    // Create the ball.
    ball.body = new cyclone::RigidBody();
    ball.radius = 0.25f;
    ball.body->setMass(5.0f);
    ball.body->setDamping(0.9f, 0.9f);
    cyclone::Matrix3 it;
    it.setDiagonal(5.0f, 5.0f, 5.0f);
    ball.body->setInertiaTensor(it);
    ball.body->setAcceleration(cyclone::Vector3::GRAVITY);

    ball.body->setCanSleep(false);
    ball.body->setAwake(true);

    // Set up the initial block
    reset();
}

const char* FractureDemo::getTitle()
{
    return "Cyclone > Fracture Demo";
}

void FractureDemo::generateContacts()
{
    hit = false;

    // Create the ground plane data
    cyclone::CollisionPlane plane;
    plane.direction = cyclone::Vector3(0,1,0);
    plane.offset = 0;

    // Set up the collision data structure
    cData.reset(maxContacts);
    cData.friction = (cyclone::real)0.9;
    cData.restitution = (cyclone::real)0.2;
    cData.tolerance = (cyclone::real)0.1;

    // Perform collision detection
    cyclone::Matrix4 transform, otherTransform;
    cyclone::Vector3 position, otherPosition;
    for (Block *block = blocks; block < blocks+MAX_BLOCKS; block++)
    {
        if (!block->exists) continue;

        // Check for collisions with the ground plane
        if (!cData.hasMoreContacts()) return;
        cyclone::CollisionDetector::boxAndHalfSpace(*block, plane, &cData);

        if (ball_active)
        {
            // And with the sphere
            if (!cData.hasMoreContacts()) return;
            if (cyclone::CollisionDetector::boxAndSphere(*block, ball, &cData))
            {
                hit = true;
                fracture_contact = cData.contactCount-1;
            }
        }

        // Check for collisions with each other box
        for (Block *other = block+1; other < blocks+MAX_BLOCKS; other++)
        {
            if (!other->exists) continue;

            if (!cData.hasMoreContacts()) return;
            cyclone::CollisionDetector::boxAndBox(*block, *other, &cData);
        }
    }

    // Check for sphere ground collisions
    if (ball_active)
    {
        if (!cData.hasMoreContacts()) return;
        cyclone::CollisionDetector::sphereAndHalfSpace(ball, plane, &cData);
    }
}

void FractureDemo::reset()
{
    // Only the first block exists
    blocks[0].exists = true;
    for (Block *block = blocks+1; block < blocks+MAX_BLOCKS; block++)
    {
        block->exists = false;
    }

    // Set the first block
    blocks[0].halfSize = cyclone::Vector3(4,4,4);
    blocks[0].body->setPosition(0, 7, 0);
    blocks[0].body->setOrientation(1,0,0,0);
    blocks[0].body->setVelocity(0,0,0);
    blocks[0].body->setRotation(0,0,0);
    blocks[0].body->setMass(100.0f);
    cyclone::Matrix3 it;
    it.setBlockInertiaTensor(blocks[0].halfSize, 100.0f);
    blocks[0].body->setInertiaTensor(it);
    blocks[0].body->setDamping(0.9f, 0.9f);
    blocks[0].body->calculateDerivedData();
    blocks[0].calculateInternals();

    blocks[0].body->setAcceleration(cyclone::Vector3::GRAVITY);
    blocks[0].body->setAwake(true);
    blocks[0].body->setCanSleep(true);


    ball_active = true;

    // Set up the ball
    ball.body->setPosition(0,5.0f,20.0f);
    ball.body->setOrientation(1,0,0,0);
    ball.body->setVelocity(
        random.randomBinomial(4.0f),
        random.randomReal(1.0f, 6.0f),
        -20.0f
        );
    ball.body->setRotation(0,0,0);
    ball.body->calculateDerivedData();
    ball.body->setAwake(true);
    ball.calculateInternals();

    hit = false;

    // Reset the contacts
    cData.contactCount = 0;
}

void FractureDemo::update()
{
    RigidBodyApplication::update();

    // Handle fractures.
    if (hit)
    {
        blocks[0].divideBlock(
            cData.contactArray[fracture_contact],
            blocks,
            blocks+1
            );
        ball_active = false;
    }
}

void FractureDemo::updateObjects(cyclone::real duration)
{
    for (Block *block = blocks; block < blocks+MAX_BLOCKS; block++)
    {
        if (block->exists)
        {
            block->body->integrate(duration);
            block->calculateInternals();
        }
    }

    if (ball_active)
    {
        ball.body->integrate(duration);
        ball.calculateInternals();
    }
}

void FractureDemo::display()
{
    const static GLfloat lightPosition[] = {0.7f,1,0.4f,0};

    RigidBodyApplication::display();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

    glEnable(GL_NORMALIZE);
    for (Block *block = blocks; block < blocks+MAX_BLOCKS; block++)
    {
        if (block->exists) block->render();
    }
    glDisable(GL_NORMALIZE);

    if (ball_active)
    {
        glColor3f(0.4f, 0.7f, 0.4f);
        glPushMatrix();
        cyclone::Vector3 pos = ball.body->getPosition();
        glTranslatef(pos.x, pos.y, pos.z);
        glutSolidSphere(0.25f, 16, 8);
        glPopMatrix();
    }

    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

    // Draw some scale circles
    glColor3f(0.75, 0.75, 0.75);
    for (unsigned i = 1; i < 20; i++)
    {
        glBegin(GL_LINE_LOOP);
        for (unsigned j = 0; j < 32; j++)
        {
            float theta = 3.1415926f * j / 16.0f;
            glVertex3f(i*cosf(theta),0.0f,i*sinf(theta));
        }
        glEnd();
    }
    glBegin(GL_LINES);
    glVertex3f(-20,0,0);
    glVertex3f(20,0,0);
    glVertex3f(0,0,-20);
    glVertex3f(0,0,20);
    glEnd();

    RigidBodyApplication::drawDebug();
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
    return new FractureDemo();
}