/*
 * The ragdoll demo.
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

#define NUM_BONES 12
#define NUM_JOINTS 11

class Bone : public cyclone::CollisionBox
{
public:
    Bone()
    {
        body = new cyclone::RigidBody();
    }

    ~Bone()
    {
        delete body;
    }

    /**
     * We use a sphere to collide bone on bone to allow some limited
     * interpenetration.
     */
    cyclone::CollisionSphere getCollisionSphere() const
    {
        cyclone::CollisionSphere sphere;
        sphere.body = body;
        sphere.radius = halfSize.x;
        sphere.offset = cyclone::Matrix4();
        if (halfSize.y < sphere.radius) sphere.radius = halfSize.y;
        if (halfSize.z < sphere.radius) sphere.radius = halfSize.z;
        sphere.calculateInternals();
        return sphere;
    }

    /** Draws the bone. */
    void render()
    {
        // Get the OpenGL transformation
        GLfloat mat[16];
        body->getGLTransform(mat);

        if (body->getAwake()) glColor3f(0.5f, 0.3f, 0.3f);
        else glColor3f(0.3f, 0.3f, 0.5f);

        glPushMatrix();
        glMultMatrixf(mat);
        glScalef(halfSize.x*2, halfSize.y*2, halfSize.z*2);
        glutSolidCube(1.0f);
        glPopMatrix();
    }

    /** Sets the bone to a specific location. */
    void setState(const cyclone::Vector3 &position,
                  const cyclone::Vector3 &extents)
    {
        body->setPosition(position);
        body->setOrientation(cyclone::Quaternion());
        body->setVelocity(cyclone::Vector3());
        body->setRotation(cyclone::Vector3());
        halfSize = extents;

        cyclone::real mass = halfSize.x * halfSize.y * halfSize.z * 8.0f;
        body->setMass(mass);

        cyclone::Matrix3 tensor;
        tensor.setBlockInertiaTensor(halfSize, mass);
        body->setInertiaTensor(tensor);

        body->setLinearDamping(0.95f);
        body->setAngularDamping(0.8f);
        body->clearAccumulators();
        body->setAcceleration(cyclone::Vector3::GRAVITY);

        body->setCanSleep(false);
        body->setAwake();

        body->calculateDerivedData();
        calculateInternals();
    }

};

/**
 * The main demo class definition.
 */
class RagdollDemo : public RigidBodyApplication
{
    cyclone::Random random;

    /** Holds the bone bodies. */
    Bone bones[NUM_BONES];

    /** Holds the joints. */
    cyclone::Joint joints[NUM_JOINTS];

    /** Processes the contact generation code. */
    virtual void generateContacts();

    /** Processes the objects in the simulation forward in time. */
    virtual void updateObjects(cyclone::real duration);

    /** Resets the position of all the bones. */
    virtual void reset();

public:
    /** Creates a new demo object. */
    RagdollDemo();

    /** Sets up the rendering. */
    virtual void initGraphics();

    /** Returns the window title for the demo. */
    virtual const char* getTitle();

    /** Display the particle positions. */
    virtual void display();
};

// Method definitions
RagdollDemo::RagdollDemo()
    :
    RigidBodyApplication()
{
    // Set up the bone hierarchy.

    // Right Knee
    joints[0].set(
        bones[0].body, cyclone::Vector3(0, 1.07f, 0),
        bones[1].body, cyclone::Vector3(0, -1.07f, 0),
        0.15f
        );

    // Left Knee
    joints[1].set(
        bones[2].body, cyclone::Vector3(0, 1.07f, 0),
        bones[3].body, cyclone::Vector3(0, -1.07f, 0),
        0.15f
        );

    // Right elbow
    joints[2].set(
        bones[9].body, cyclone::Vector3(0, 0.96f, 0),
        bones[8].body, cyclone::Vector3(0, -0.96f, 0),
        0.15f
        );

    // Left elbow
    joints[3].set(
        bones[11].body, cyclone::Vector3(0, 0.96f, 0),
        bones[10].body, cyclone::Vector3(0, -0.96f, 0),
        0.15f
        );

    // Stomach to Waist
    joints[4].set(
        bones[4].body, cyclone::Vector3(0.054f, 0.50f, 0),
        bones[5].body, cyclone::Vector3(-0.043f, -0.45f, 0),
        0.15f
        );

    joints[5].set(
        bones[5].body, cyclone::Vector3(-0.043f, 0.411f, 0),
        bones[6].body, cyclone::Vector3(0, -0.411f, 0),
        0.15f
        );

    joints[6].set(
        bones[6].body, cyclone::Vector3(0, 0.521f, 0),
        bones[7].body, cyclone::Vector3(0, -0.752f, 0),
        0.15f
        );

    // Right hip
    joints[7].set(
        bones[1].body, cyclone::Vector3(0, 1.066f, 0),
        bones[4].body, cyclone::Vector3(0, -0.458f, -0.5f),
        0.15f
        );

    // Left Hip
    joints[8].set(
        bones[3].body, cyclone::Vector3(0, 1.066f, 0),
        bones[4].body, cyclone::Vector3(0, -0.458f, 0.5f),
        0.105f
        );

    // Right shoulder
    joints[9].set(
        bones[6].body, cyclone::Vector3(0, 0.367f, -0.8f),
        bones[8].body, cyclone::Vector3(0, 0.888f, 0.32f),
        0.15f
        );

    // Left shoulder
    joints[10].set(
        bones[6].body, cyclone::Vector3(0, 0.367f, 0.8f),
        bones[10].body, cyclone::Vector3(0, 0.888f, -0.32f),
        0.15f
        );

    // Set up the initial positions
    reset();
}

const char* RagdollDemo::getTitle()
{
    return "Cyclone > Ragdoll Demo";
}

void RagdollDemo::generateContacts()
{
    // Create the ground plane data
    cyclone::CollisionPlane plane;
    plane.direction = cyclone::Vector3(0,1,0);
    plane.offset = 0;

    // Set up the collision data structure
    cData.reset(maxContacts);
    cData.friction = (cyclone::real)0.9;
    cData.restitution = (cyclone::real)0.6;
    cData.tolerance = (cyclone::real)0.1;

    // Perform exhaustive collision detection on the ground plane
    cyclone::Matrix4 transform, otherTransform;
    cyclone::Vector3 position, otherPosition;
    for (Bone *bone = bones; bone < bones+NUM_BONES; bone++)
    {
        // Check for collisions with the ground plane
        if (!cData.hasMoreContacts()) return;
        cyclone::CollisionDetector::boxAndHalfSpace(*bone, plane, &cData);

        cyclone::CollisionSphere boneSphere = bone->getCollisionSphere();

        // Check for collisions with each other box
        for (Bone *other = bone+1; other < bones+NUM_BONES; other++)
        {
            if (!cData.hasMoreContacts()) return;

            cyclone::CollisionSphere otherSphere = other->getCollisionSphere();

            cyclone::CollisionDetector::sphereAndSphere(
                boneSphere,
                otherSphere,
                &cData
                );
        }
    }

    // Check for joint violation
    for (cyclone::Joint *joint = joints; joint < joints+NUM_JOINTS; joint++)
    {
        if (!cData.hasMoreContacts()) return;
        unsigned added = joint->addContact(cData.contacts, cData.contactsLeft);
        cData.addContacts(added);
    }
}

void RagdollDemo::reset()
{
    bones[0].setState(
        cyclone::Vector3(0, 0.993, -0.5),
        cyclone::Vector3(0.301, 1.0, 0.234));
    bones[1].setState(
        cyclone::Vector3(0, 3.159, -0.56),
        cyclone::Vector3(0.301, 1.0, 0.234));
    bones[2].setState(
        cyclone::Vector3(0, 0.993, 0.5),
        cyclone::Vector3(0.301, 1.0, 0.234));
    bones[3].setState(
        cyclone::Vector3(0, 3.15, 0.56),
        cyclone::Vector3(0.301, 1.0, 0.234));
    bones[4].setState(
        cyclone::Vector3(-0.054, 4.683, 0.013),
        cyclone::Vector3(0.415, 0.392, 0.690));
    bones[5].setState(
        cyclone::Vector3(0.043, 5.603, 0.013),
        cyclone::Vector3(0.301, 0.367, 0.693));
    bones[6].setState(
        cyclone::Vector3(0, 6.485, 0.013),
        cyclone::Vector3(0.435, 0.367, 0.786));
    bones[7].setState(
        cyclone::Vector3(0, 7.759, 0.013),
        cyclone::Vector3(0.45, 0.598, 0.421));
    bones[8].setState(
        cyclone::Vector3(0, 5.946, -1.066),
        cyclone::Vector3(0.267, 0.888, 0.207));
    bones[9].setState(
        cyclone::Vector3(0, 4.024, -1.066),
        cyclone::Vector3(0.267, 0.888, 0.207));
    bones[10].setState(
        cyclone::Vector3(0, 5.946, 1.066),
        cyclone::Vector3(0.267, 0.888, 0.207));
    bones[11].setState(
        cyclone::Vector3(0, 4.024, 1.066),
        cyclone::Vector3(0.267, 0.888, 0.207));

    cyclone::real strength = -random.randomReal(500.0f, 1000.0f);
    for (unsigned i = 0; i < NUM_BONES; i++)
    {
        bones[i].body->addForceAtBodyPoint(
            cyclone::Vector3(strength, 0, 0), cyclone::Vector3()
            );
    }
    bones[6].body->addForceAtBodyPoint(
        cyclone::Vector3(strength, 0, random.randomBinomial(1000.0f)),
        cyclone::Vector3(random.randomBinomial(4.0f), random.randomBinomial(3.0f), 0)
        );

    // Reset the contacts
    cData.contactCount = 0;
}

void RagdollDemo::updateObjects(cyclone::real duration)
{
    for (Bone *bone = bones; bone < bones+NUM_BONES; bone++)
    {
        bone->body->integrate(duration);
        bone->calculateInternals();
    }
}

void RagdollDemo::initGraphics()
{
    GLfloat lightAmbient[] = {0.8f,0.8f,0.8f,1.0f};
    GLfloat lightDiffuse[] = {0.9f,0.95f,1.0f,1.0f};

    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);

    glEnable(GL_LIGHT0);

    Application::initGraphics();
}

void RagdollDemo::display()
{
    const static GLfloat lightPosition[] = {0.7f,-1,0.4f,0};
    const static GLfloat lightPositionMirror[] = {0.7f,1,0.4f,0};

    RigidBodyApplication::display();

    // Render the bones
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

    glEnable(GL_NORMALIZE);
    glColor3f(1,0,0);
    for (unsigned i = 0; i < NUM_BONES; i++)
    {
        bones[i].render();
    }
    glDisable(GL_NORMALIZE);

    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

    glDisable(GL_DEPTH_TEST);
    glBegin(GL_LINES);
    for (unsigned i = 0; i < NUM_JOINTS; i++)
    {
        cyclone::Joint *joint = joints + i;
        cyclone::Vector3 a_pos = joint->body[0]->getPointInWorldSpace(joint->position[0]);
        cyclone::Vector3 b_pos = joint->body[1]->getPointInWorldSpace(joint->position[1]);
        cyclone::real length = (b_pos - a_pos).magnitude();

        if (length > joint->error) glColor3f(1,0,0);
        else glColor3f(0,1,0);

        glVertex3f(a_pos.x, a_pos.y, a_pos.z);
        glVertex3f(b_pos.x, b_pos.y, b_pos.z);
    }
    glEnd();
    glEnable(GL_DEPTH_TEST);

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
    return new RagdollDemo();
}