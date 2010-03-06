/*
 * Timing functions, frame management and profiling.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Ian Millington 2003-2006. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

#include "timing.h"

// Import the high performance timer (c. 4ms).
#include <windows.h>
#include <mmsystem.h>

// Hold internal timing data for the performance counter.
static bool qpcFlag;
static double qpcFrequency;

// Internal time and clock access functions
unsigned systemTime()
{
    if(qpcFlag)
    {
        static LONGLONG qpcMillisPerTick;
        QueryPerformanceCounter((LARGE_INTEGER*)&qpcMillisPerTick);
        return (unsigned)(qpcMillisPerTick * qpcFrequency);
    }
    else
    {
        return unsigned(timeGetTime());
    }
}

unsigned TimingData::getTime()
{
    return systemTime();
}

unsigned long systemClock()
{
    __asm {
        rdtsc;
    }
}

unsigned long TimingData::getClock()
{
    return systemClock();
}

// Sets up the timing system and registers the performance timer.
void initTime()
{
    LONGLONG time;

    qpcFlag = (QueryPerformanceFrequency((LARGE_INTEGER*)&time) > 0);

    // Check if we have access to the performance counter at this
    // resolution.
    if (qpcFlag) qpcFrequency = 1000.0 / time;
}


// Holds the global frame time that is passed around
static TimingData *timingData = NULL;

// Retrieves the global frame info instance
TimingData& TimingData::get()
{
    return (TimingData&)*timingData;
}

// Updates the global frame information. Should be called once per frame.
void TimingData::update()
{
    if (!timingData) return;

    // Advance the frame number.
    if (!timingData->isPaused)
    {
        timingData->frameNumber++;
    }

    // Update the timing information.
    unsigned thisTime = systemTime();
    timingData->lastFrameDuration = thisTime -
        timingData->lastFrameTimestamp;
    timingData->lastFrameTimestamp = thisTime;

    // Update the tick information.
    unsigned long thisClock = systemClock();
    timingData->lastFrameClockTicks =
    thisClock - timingData->lastFrameClockstamp;
    timingData->lastFrameClockstamp = thisClock;

    // Update the RWA frame rate if we are able to.
    if (timingData->frameNumber > 1) {
        if (timingData->averageFrameDuration <= 0)
        {
            timingData->averageFrameDuration =
                (double)timingData->lastFrameDuration;
        }
        else
        {
            // RWA over 100 frames.
            timingData->averageFrameDuration *= 0.99;
            timingData->averageFrameDuration +=
                0.01 * (double)timingData->lastFrameDuration;

            // Invert to get FPS
            timingData->fps =
                (float)(1000.0/timingData->averageFrameDuration);
        }
    }
}

void TimingData::init()
{
    // Set up the timing system.
    initTime();

    // Create the frame info object
    if (!timingData) timingData = new TimingData();

    // Set up the frame info structure.
    timingData->frameNumber = 0;

    timingData->lastFrameTimestamp = systemTime();
    timingData->lastFrameDuration = 0;

    timingData->lastFrameClockstamp = systemClock();
    timingData->lastFrameClockTicks = 0;

    timingData->isPaused = false;

    timingData->averageFrameDuration = 0;
    timingData->fps = 0;
}

void TimingData::deinit()
{
        delete timingData;
        timingData = NULL;
}