#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <memory.h>
#include "SampleInterfaces.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////

BuildContext::BuildContext() 
{
}

// Virtual functions for custom implementations.
void BuildContext::doResetLog()
{
}

void BuildContext::doLog(const rcLogCategory category, const char* msg, const int len)
{
}

void BuildContext::doResetTimers()
{
}

void BuildContext::doStartTimer(const rcTimerLabel label)
{
}

void BuildContext::doStopTimer(const rcTimerLabel label)
{
}

int BuildContext::doGetAccumulatedTime(const rcTimerLabel label) const
{
	return 0;
}

void BuildContext::dumpLog(const char* format, ...)
{
}

int BuildContext::getLogCount() const
{
	return 0;
}

const char* BuildContext::getLogText(const int i) const
{
	return "";
}
