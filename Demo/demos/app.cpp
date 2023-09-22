/*
 * The definition file for the default application object.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */
#include <cstring>
#include "ogl_headers.h"
#include "app.h"
#include "timing.h"

void Application::initGraphics()
{
    glClearColor(0.9f, 0.95f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);

    setView();
}

void Application::setView()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (double)width/(double)height, 1.0, 500.0);
    glMatrixMode(GL_MODELVIEW);
}

void Application::display()
{
    glClear(GL_COLOR_BUFFER_BIT);

    glBegin(GL_LINES);
    glVertex2i(1, 1);
    glVertex2i(639, 319);
    glEnd();
}

const char* Application::getTitle()
{
    return "Cyclone Demo";
}

void Application::deinit()
{
}

void Application::update()
{
    glutPostRedisplay();
}

void Application::key(unsigned char key)
{
}


void Application::resize(int width, int height)
{
    // Avoid the divide by zero.
    if (height <= 0) height = 1;

    // Set the internal variables and update the view
    Application::width = width;
    Application::height = height;
    glViewport(0, 0, width, height);
    setView();
}

void Application::mouse(int button, int state, int x, int y)
{
}

void Application::mouseDrag(int x, int y)
{
}

// The following methods aren't intended to be overloaded
void Application::renderText(float x, float y, const char *text, void *font)
{
    glDisable(GL_DEPTH_TEST);

    // Temporarily set up the view in orthographic projection.
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, (double)width, 0.0, (double)height, -1.0, 1.0);

    // Move to modelview mode.
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Ensure we have a font
    if (font == NULL) {
        font = GLUT_BITMAP_HELVETICA_10;
    }

    // Loop through characters displaying them.
    size_t len = strlen(text);

    glRasterPos2f(x, y);
    for (const char *letter = text; letter < text+len; letter++) {

        // If we meet a newline, then move down by the line-height
        // TODO: Make the line-height a function of the font
        if (*letter == '\n') {
            y -= 12.0f;
            glRasterPos2f(x, y);
        }
        glutBitmapCharacter(font, *letter);
    }

    // Pop the matrices to return to how we were before.
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glEnable(GL_DEPTH_TEST);
}


MassAggregateApplication::MassAggregateApplication(unsigned int particleCount)
:
world(particleCount*10)
{
    particleArray = new cyclone::Particle[particleCount];
    for (unsigned i = 0; i < particleCount; i++)
    {
        world.getParticles().push_back(particleArray + i);
    }

    groundContactGenerator.init(&world.getParticles());
    world.getContactGenerators().push_back(&groundContactGenerator);
}

MassAggregateApplication::~MassAggregateApplication()
{
    delete[] particleArray;
}

void MassAggregateApplication::initGraphics()
{
    // Call the superclass
    Application::initGraphics();
}

void MassAggregateApplication::display()
{
    // Clear the view port and set the camera direction
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(0.0, 3.5, 8.0,  0.0, 3.5, 0.0,  0.0, 1.0, 0.0);

    glColor3f(0,0,0);

    cyclone::ParticleWorld::Particles &particles = world.getParticles();
    for (cyclone::ParticleWorld::Particles::iterator p = particles.begin();
        p != particles.end();
        p++)
    {
        cyclone::Particle *particle = *p;
        const cyclone::Vector3 &pos = particle->getPosition();
        glPushMatrix();
        glTranslatef(pos.x, pos.y, pos.z);
        glutSolidSphere(0.1f, 20, 10);
        glPopMatrix();
    }
}

void MassAggregateApplication::update()
{
    // Clear accumulators
    world.startFrame();

    // Find the duration of the last frame in seconds
    float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
    if (duration <= 0.0f) return;

    // Run the simulation
    world.runPhysics(duration);

    Application::update();
}

RigidBodyApplication::RigidBodyApplication()
:
    theta(0.0f),
    phi(15.0f),
    resolver(maxContacts*8),

    renderDebugInfo(false),
    pauseSimulation(true),
    autoPauseSimulation(false)
{
    cData.contactArray = contacts;
}

void RigidBodyApplication::update()
{
    // Find the duration of the last frame in seconds
    float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
    if (duration <= 0.0f) return;
    else if (duration > 0.05f) duration = 0.05f;

    // Exit immediately if we aren't running the simulation
    if (pauseSimulation)
    {
        Application::update();
        return;
    }
    else if (autoPauseSimulation)
    {
        pauseSimulation = true;
        autoPauseSimulation = false;
    }

    // Update the objects
    updateObjects(duration);

    // Perform the contact generation
    generateContacts();

    // Resolve detected contacts
    resolver.resolveContacts(
        cData.contactArray,
        cData.contactCount,
        duration
        );

    Application::update();
}

void RigidBodyApplication::display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    gluLookAt(18.0f, 0, 0,  0, 0, 0,  0, 1.0f, 0);
    glRotatef(-phi, 0, 0, 1);
    glRotatef(theta, 0, 1, 0);
    glTranslatef(0, -5.0f, 0);
}

void RigidBodyApplication::drawDebug()
{
    if (!renderDebugInfo) return;

    // Recalculate the contacts, so they are current (in case we're
    // paused, for example).
    generateContacts();

    // Render the contacts, if required
    glBegin(GL_LINES);
    for (unsigned i = 0; i < cData.contactCount; i++)
    {
        // Interbody contacts are in green, floor contacts are red.
        if (contacts[i].body[1]) {
            glColor3f(0,1,0);
        } else {
            glColor3f(1,0,0);
        }

        cyclone::Vector3 vec = contacts[i].contactPoint;
        glVertex3f(vec.x, vec.y, vec.z);

        vec += contacts[i].contactNormal;
        glVertex3f(vec.x, vec.y, vec.z);
    }

    glEnd();
}

void RigidBodyApplication::mouse(int button, int state, int x, int y)
{
    // Set the position
    last_x = x;
    last_y = y;
}

void RigidBodyApplication::mouseDrag(int x, int y)
{
    // Update the camera
    theta += (x - last_x)*0.25f;
    phi += (y - last_y)*0.25f;

    // Keep it in bounds
    if (phi < -20.0f) phi = -20.0f;
    else if (phi > 80.0f) phi = 80.0f;

    // Remember the position
    last_x = x;
    last_y = y;
}

void RigidBodyApplication::key(unsigned char key)
{
    switch(key)
    {
    case 'R': case 'r':
        // Reset the simulation
        reset();
        return;

    case 'C': case 'c':
        // Toggle rendering of contacts
        renderDebugInfo = !renderDebugInfo;
        return;

    case 'P': case 'p':
        // Toggle running the simulation
        pauseSimulation = !pauseSimulation;
        return;

    case ' ':
        // Advance one frame
        autoPauseSimulation = true;
        pauseSimulation = false;
    }

    Application::key(key);
}
