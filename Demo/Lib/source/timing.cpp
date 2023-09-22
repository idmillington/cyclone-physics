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

#include <chrono>
#include "timing.h"

static std::chrono::high_resolution_clock::time_point initTimePoint;

uint64_t TimingData::getTime()
{
	auto now = std::chrono::high_resolution_clock::now();
	auto duration = now.time_since_epoch();
	return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

uint64_t TimingData::getClock()
{
	auto now = std::chrono::high_resolution_clock::now();
	auto duration = now - initTimePoint;
	return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

// Sets up the timing system and registers the performance timer.
void initTime()
{
	initTimePoint = std::chrono::high_resolution_clock::now();
}


// Holds the global frame time that is passed around
static TimingData* timingData = NULL;

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
	auto thisTime = getTime();
	timingData->lastFrameDuration = thisTime - timingData->lastFrameTimestamp;
	timingData->lastFrameTimestamp = thisTime;

	// Update the tick information.
	auto thisClock = getClock();
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
				(float)(1000.0 / timingData->averageFrameDuration);
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

	timingData->lastFrameTimestamp = getTime();
	timingData->lastFrameDuration = 0;

	timingData->lastFrameClockstamp = getClock();
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
