/*
 * The base application class for all demos.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */
#include <cstdlib>

#include <cyclone/cyclone.h>

/**
 * An application is the base class for all demonstration progams.
 * GLUT is a c-style API, which calls bare functions. This makes
 * it more difficult to provide default services for all demos and
 * only override them when needed.
 *
 * To solve this, the GLUT API is translated into calls on a
 * generic application object. Each demonstration will create a
 * concrete subclass of Application, providing the behaviours it
 * needs. The common code for all demos manages dispatch of
 * requests to the appropriate application object.
 *
 * To provide a correct application object of the right type without
 * the core code needing to know which subclass is being used, each
 * demonstration will supply a getApplication function which creates
 * (with new) and returns a pointer to a new Application instance.
 *
 * Even though subclasses will have to implement most of the methods
 * in this class, I have not made them pure virtual. This saves the
 * annoying need to implement an empty function that isn't needed.
 */
class Application
{
protected:
    /**
     * Holds the height of the application window.
     */
    int height;

    /**
     * Holds the current width of the application window.
     */
    int width;

public:
    /**
     * Gets the title of the demo for the title bar of the window.
     *
     * The default implementation returns a generic title.
     */
    virtual const char* getTitle();

    /**
     * Sets up the graphics, and allows the application to acquire
     * graphical resources. Guaranteed to be called after OpenGL is
     * set up.
     *
     * The default implementation sets up a basic view, and calls
     * setView to set up the camera projection.
     */
    virtual void initGraphics();

    /**
     * Called to set the projection characteristics of the camera.
     *
     * The default implementation uses a 60 degree field of view camera
     * with a range from 1-500 units.
     */
    virtual void setView();

    /**
     * Called just before the application is destroyed. Clear up can
     * be performed here or in the application destructor.
     *
     * The default implementation does nothing.
     */
    virtual void deinit();

    /**
     * Called each frame to display the current scene. The common code
     * will automatically flush the graphics pipe and swap the render
     * buffers after calling this so glFlush doesn't need to be called.
     *
     * The default
     * implementation draws a simple diagonal line across the surface
     * (as a sanity check to make sure GL is working).
     */
    virtual void display();

    /**
     * Called each frame to update the current state of the scene.
     *
     * The default implementation requests that the display be refreshed.
     * It should probably be called from any subclass update as the last
     * command.
     */
    virtual void update();

    /**
     * Called when a keypress is detected.
     *
     * The default implementation does nothing.
     *
     * @param key The ascii code of the key that has been pressed.
     */
    virtual void key(unsigned char key);


    /**
     * Notifies the application that the window has changed size.
     * The new size is given.
     *
     * The default implementation sets the internal height and width
     * parameters and changes the gl viewport. These are steps you'll
     * almost always need, so its worth calling the base class version
     * of this method even if you override it in a demo class.
     */
    virtual void resize(int width, int height);

    /**
     * Called when GLUT detects a mouse button press.
     *
     * The default implementation does nothing.
     */
    virtual void mouse(int button, int state, int x, int y);

    /**
     * Called when GLUT detects a mouse drag.
     *
     * The default implementation does nothing.
     */
    virtual void mouseDrag(int x, int y);

    // These are helper functions that can be used by an application
    // to render things.

    /**
     * Renders the given text to the given x,y location (in screen space)
     * on the window. This is used to pass status information to the
     * application.
     */
    void renderText(float x, float y, const char *text, void* font=NULL);
};

/**
 * This application adds additional functionality used in the mass-aggregate demos.
 */
class MassAggregateApplication : public Application
{
protected:
    cyclone::ParticleWorld world;
    cyclone::Particle *particleArray;
    cyclone::GroundContacts groundContactGenerator;

public:
    MassAggregateApplication(unsigned int particleCount);
    virtual ~MassAggregateApplication();

    /** Update the particle positions. */
    virtual void update();

    /** Sets up the graphic rendering. */
    virtual void initGraphics();

    /** Display the particles. */
    virtual void display();
};

/**
 * This application adds additional functionality used in many of the
 * demos. This includes the ability to track contacts (for rigid bodies)
 * and move the camera around.
 */
 class RigidBodyApplication : public Application
 {
 protected:
    /** Holds the maximum number of contacts. */
    const static unsigned maxContacts = 256;

    /** Holds the array of contacts. */
    cyclone::Contact contacts[maxContacts];

    /** Holds the collision data structure for collision detection. */
    cyclone::CollisionData cData;

    /** Holds the contact resolver. */
    cyclone::ContactResolver resolver;

    /** Holds the camera angle. */
    float theta;

    /** Holds the camera elevation. */
    float phi;

    /** Holds the position of the mouse at the last frame of a drag. */
    int last_x, last_y;

    /** True if the contacts should be rendered. */
    bool renderDebugInfo;

    /** True if the simulation is paused. */
    bool pauseSimulation;

    /** Pauses the simulation after the next frame automatically */
    bool autoPauseSimulation;

    /** Processes the contact generation code. */
    virtual void generateContacts() = 0;

    /** Processes the objects in the simulation forward in time. */
    virtual void updateObjects(cyclone::real duration) = 0;

    /**
     * Finishes drawing the frame, adding debugging information
     * as needed.
     */
    void drawDebug();

    /** Resets the simulation. */
    virtual void reset() = 0;

public:
    /**
     * Creates a new application object.
     */
    RigidBodyApplication();

    /** Display the application. */
    virtual void display();

    /** Update the objects. */
    virtual void update();

    /** Handle a mouse click. */
    virtual void mouse(int button, int state, int x, int y);

    /** Handle a mouse drag */
    virtual void mouseDrag(int x, int y);

    /** Handles a key press. */
    virtual void key(unsigned char key);
 };
