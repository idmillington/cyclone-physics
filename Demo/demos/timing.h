/*
 * Timing functions.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Ian Millington 2003-2006. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

/**
 * @file
 *
 * Holds the timing system for the physics demos.
 */
#ifndef CYCLONE_DEMO_TIMING_H
#define CYCLONE_DEMO_TIMING_H

/**
 * Represents all the information that the demo might need about the
 * timing of the game: current time, fps, frame number, and so on.
 */
struct TimingData
{
    /** The current render frame. This simply increments. */
    unsigned frameNumber;

    /**
     * The timestamp when the last frame ended. Times are
     * given in milliseconds since some undefined time.
     */
    unsigned lastFrameTimestamp;

    /**
     * The duration of the last frame in milliseconds.
     */
    unsigned lastFrameDuration;

    /**
     * The clockstamp of the end of the last frame.
     */
    unsigned long lastFrameClockstamp;

    /**
     * The duration of the last frame in clock ticks.
     */
    unsigned long lastFrameClockTicks;

    /**
     * Keeps track of whether the rendering is paused.
     */
    bool isPaused;

    // Calculated data

    /**
     * This is a recency weighted average of the frame time, calculated
     * from frame durations.
     */
    double averageFrameDuration;

    /**
     * The reciprocal of the average frame duration giving the mean
     * fps over a recency weighted average.
     */
    float fps;

    /**
     * Gets the global timing data object.
     */
    static TimingData& get();

    /**
     * Updates the timing system, should be called once per frame.
     */
    static void update();

    /**
     * Initialises the frame information system. Use the overall
     * init function to set up all modules.
     */
    static void init();

    /**
     * Deinitialises the frame information system.
     */
    static void deinit();

    /**
     * Gets the global system time, in the best resolution possible.
     * Timing is in milliseconds.
     */
    static unsigned getTime();

    /**
     * Gets the clock ticks since process start.
     */
    static unsigned long getClock();


private:
    // These are private to stop instances being created: use get().
    TimingData() {}
    TimingData(const TimingData &) {}
    TimingData& operator=(const TimingData &);
};


#endif // CYCLONE_DEMO_TIMING_H


