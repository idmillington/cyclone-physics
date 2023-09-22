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
    unsigned frameNumber = 0;

    /**
     * The timestamp when the last frame ended. Times are
     * given in milliseconds since some undefined time.
     */
    uint64_t lastFrameTimestamp = 0;

    /**
     * The duration of the last frame in milliseconds.
     */
    uint64_t lastFrameDuration = 0;

    /**
     * The clockstamp of the end of the last frame.
     */
    uint64_t lastFrameClockstamp = 0;

    /**
     * The duration of the last frame in clock ticks.
     */
    uint64_t lastFrameClockTicks = 0;

    /**
     * Keeps track of whether the rendering is paused.
     */
    bool isPaused = false;

    // Calculated data

    /**
     * This is a recency weighted average of the frame time, calculated
     * from frame durations.
     */
    double averageFrameDuration = 0.0;

    /**
     * The reciprocal of the average frame duration giving the mean
     * fps over a recency weighted average.
     */
    float fps = 0.f;

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
     * Gets the global system time in milliseconds.
     */
    static uint64_t getTime();

    /**
     * Gets the time in milliseconds since timer init.
     */
    static uint64_t getClock();


private:
    // These are private to stop instances being created: use get().
    TimingData() = delete;
    TimingData(const TimingData &) = delete;
    TimingData& operator=(const TimingData&) = delete;
};


#endif // CYCLONE_DEMO_TIMING_H


