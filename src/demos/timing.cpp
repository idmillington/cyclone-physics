#include "timing.h"

static TimingData *timingData = NULL;

TimingData& TimingData::get()
{
    return (TimingData&)*timingData;
}

double TimingData::getTime() {

	std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double,std::milli> dtn = tp.time_since_epoch();
	return dtn.count();

}

void TimingData::update()
{
	if (!timingData) return;

	// Advance the frame number.
	if (!timingData->isPaused)
	{
		timingData->frameNumber++;
	}
	double thisTime = getTime();
	timingData->lastFrameDuration = thisTime -
		timingData->lastFrameTimestamp;
	timingData->lastFrameTimestamp = thisTime;

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


void TimingData::init() {

    // Create the frame info object
    if (!timingData) timingData = new TimingData();

    // Set up the frame info structure.
    timingData->frameNumber = 0;

    timingData->lastFrameTimestamp = getTime();
    timingData->lastFrameDuration = 0;

    timingData->isPaused = false;

    timingData->averageFrameDuration = 0;
    timingData->fps = 0;


}

void TimingData::deinit()
{
        delete timingData;
        timingData = NULL;
}
