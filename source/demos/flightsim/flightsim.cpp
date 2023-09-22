/*
 * The flightsim demo.
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

/**
 * The main demo class definition.
 */
class FlightSimDemo : public Application
{
    cyclone::AeroControl left_wing;
    cyclone::AeroControl right_wing;
    cyclone::AeroControl rudder;
    cyclone::Aero tail;
    cyclone::RigidBody aircraft;
    cyclone::ForceRegistry registry;

    cyclone::Vector3 windspeed;

    float left_wing_control;
    float right_wing_control;
    float rudder_control;

    void resetPlane();

public:
    /** Creates a new demo object. */
    FlightSimDemo();
    virtual ~FlightSimDemo();

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
FlightSimDemo::FlightSimDemo()
:
Application(),

right_wing(cyclone::Matrix3(0,0,0, -1,-0.5f,0, 0,0,0),
           cyclone::Matrix3(0,0,0, -0.995f,-0.5f,0, 0,0,0),
           cyclone::Matrix3(0,0,0, -1.005f,-0.5f,0, 0,0,0),
           cyclone::Vector3(-1.0f, 0.0f, 2.0f), &windspeed),

left_wing(cyclone::Matrix3(0,0,0, -1,-0.5f,0, 0,0,0),
          cyclone::Matrix3(0,0,0, -0.995f,-0.5f,0, 0,0,0),
          cyclone::Matrix3(0,0,0, -1.005f,-0.5f,0, 0,0,0),
          cyclone::Vector3(-1.0f, 0.0f, -2.0f), &windspeed),

rudder(cyclone::Matrix3(0,0,0, 0,0,0, 0,0,0),
       cyclone::Matrix3(0,0,0, 0,0,0, 0.01f,0,0),
       cyclone::Matrix3(0,0,0, 0,0,0, -0.01f,0,0),
       cyclone::Vector3(2.0f, 0.5f, 0), &windspeed),

tail(cyclone::Matrix3(0,0,0, -1,-0.5f,0, 0,0,-0.1f),
     cyclone::Vector3(2.0f, 0, 0), &windspeed),

left_wing_control(0), right_wing_control(0), rudder_control(0),

windspeed(0,0,0)
{
    // Set up the aircraft rigid body.
    resetPlane();

    aircraft.setMass(2.5f);
    cyclone::Matrix3 it;
    it.setBlockInertiaTensor(cyclone::Vector3(2,1,1), 1);
    aircraft.setInertiaTensor(it);

    aircraft.setDamping(0.8f, 0.8f);

    aircraft.setAcceleration(cyclone::Vector3::GRAVITY);
    aircraft.calculateDerivedData();

    aircraft.setAwake();
    aircraft.setCanSleep(false);

    registry.add(&aircraft, &left_wing);
    registry.add(&aircraft, &right_wing);
    registry.add(&aircraft, &rudder);
    registry.add(&aircraft, &tail);
}

FlightSimDemo::~FlightSimDemo()
{
}

void FlightSimDemo::resetPlane()
{
    aircraft.setPosition(0, 0, 0);
    aircraft.setOrientation(1,0,0,0);

    aircraft.setVelocity(0,0,0);
    aircraft.setRotation(0,0,0);
}

static void drawAircraft()
{
    // Fuselage
    glPushMatrix();
    glTranslatef(-0.5f, 0, 0);
    glScalef(2.0f, 0.8f, 1.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Rear Fuselage
    glPushMatrix();
    glTranslatef(1.0f, 0.15f, 0);
    glScalef(2.75f, 0.5f, 0.5f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Wings
    glPushMatrix();
    glTranslatef(0, 0.3f, 0);
    glScalef(0.8f, 0.1f, 6.0f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Rudder
    glPushMatrix();
    glTranslatef(2.0f, 0.775f, 0);
    glScalef(0.75f, 1.15f, 0.1f);
    glutSolidCube(1.0f);
    glPopMatrix();

    // Tail-plane
    glPushMatrix();
    glTranslatef(1.9f, 0, 0);
    glScalef(0.85f, 0.1f, 2.0f);
    glutSolidCube(1.0f);
    glPopMatrix();
}

void FlightSimDemo::display()
{
    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    cyclone::Vector3 pos = aircraft.getPosition();
    cyclone::Vector3 offset(4.0f+aircraft.getVelocity().magnitude(), 0, 0);
    offset = aircraft.getTransform().transformDirection(offset);
    gluLookAt(pos.x+offset.x, pos.y+5.0f, pos.z+offset.z,
              pos.x, pos.y, pos.z,
              0.0, 1.0, 0.0);

    glColor3f(0.6f,0.6f,0.6f);
    int bx = int(pos.x);
    int bz = int(pos.z);
    glBegin(GL_QUADS);
    for (int x = -20; x <= 20; x++) for (int z = -20; z <= 20; z++)
    {
        glVertex3f(bx+x-0.1f, 0, bz+z-0.1f);
        glVertex3f(bx+x-0.1f, 0, bz+z+0.1f);
        glVertex3f(bx+x+0.1f, 0, bz+z+0.1f);
        glVertex3f(bx+x+0.1f, 0, bz+z-0.1f);
    }
    glEnd();

    // Set the transform matrix for the aircraft
    cyclone::Matrix4 transform = aircraft.getTransform();
    GLfloat gl_transform[16];
    transform.fillGLArray(gl_transform);
    glPushMatrix();
    glMultMatrixf(gl_transform);

    // Draw the aircraft
    glColor3f(0,0,0);
    drawAircraft();
    glPopMatrix();

    glColor3f(0.8f, 0.8f, 0.8f);
    glPushMatrix();
    glTranslatef(0, -1.0f - pos.y, 0);
    glScalef(1.0f, 0.001f, 1.0f);
    glMultMatrixf(gl_transform);
    drawAircraft();
    glPopMatrix();

    char buffer[256];
    sprintf(
        buffer,
        "Altitude: %.1f | Speed %.1f",
        aircraft.getPosition().y,
        aircraft.getVelocity().magnitude()
        );
    glColor3f(0,0,0);
    renderText(10.0f, 24.0f, buffer);

    sprintf(
        buffer,
        "Left Wing: %.1f | Right Wing: %.1f | Rudder %.1f",
        left_wing_control, right_wing_control, rudder_control
        );
    renderText(10.0f, 10.0f, buffer);
}

void FlightSimDemo::update()
{
    // Find the duration of the last frame in seconds
    float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
    if (duration <= 0.0f) return;

    // Start with no forces or acceleration.
    aircraft.clearAccumulators();

    // Add the propeller force
    cyclone::Vector3 propulsion(-10.0f, 0, 0);
    propulsion = aircraft.getTransform().transformDirection(propulsion);
    aircraft.addForce(propulsion);

    // Add the forces acting on the aircraft.
    registry.updateForces(duration);

    // Update the aircraft's physics.
    aircraft.integrate(duration);

    // Do a very basic collision detection and response with the ground.
    cyclone::Vector3 pos = aircraft.getPosition();
    if (pos.y < 0.0f)
    {
        pos.y = 0.0f;
        aircraft.setPosition(pos);

        if (aircraft.getVelocity().y < -10.0f)
        {
            resetPlane();
        }
    }

    Application::update();
}

const char* FlightSimDemo::getTitle()
{
    return "Cyclone > Flight Sim Demo";
}

void FlightSimDemo::key(unsigned char key)
{
    switch(key)
    {
    case 'q': case 'Q':
        rudder_control += 0.1f;
        break;

    case 'e': case 'E':
        rudder_control -= 0.1f;
        break;

    case 'w': case 'W':
        left_wing_control -= 0.1f;
        right_wing_control -= 0.1f;
        break;

    case 's': case 'S':
        left_wing_control += 0.1f;
        right_wing_control += 0.1f;
        break;

    case 'd': case 'D':
        left_wing_control -= 0.1f;
        right_wing_control += 0.1f;
        break;

    case 'a': case 'A':
        left_wing_control += 0.1f;
        right_wing_control -= 0.1f;
        break;

    case 'x': case 'X':
        left_wing_control = 0.0f;
        right_wing_control = 0.0f;
        rudder_control = 0.0f;
        break;

    case 'r': case 'R':
        resetPlane();
        break;

    default:
        Application::key(key);
    }

    // Make sure the controls are in range
    if (left_wing_control < -1.0f) left_wing_control = -1.0f;
    else if (left_wing_control > 1.0f) left_wing_control = 1.0f;
    if (right_wing_control < -1.0f) right_wing_control = -1.0f;
    else if (right_wing_control > 1.0f) right_wing_control = 1.0f;
    if (rudder_control < -1.0f) rudder_control = -1.0f;
    else if (rudder_control > 1.0f) rudder_control = 1.0f;

    // Update the control surfaces
    left_wing.setControl(left_wing_control);
    right_wing.setControl(right_wing_control);
    rudder.setControl(rudder_control);
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
    return new FlightSimDemo();
}