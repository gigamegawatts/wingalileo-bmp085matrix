#include "DateTime.h"
#include <time.h>

DateTime::DateTime()
{
}


DateTime::~DateTime()
{
}

void DateTime::getCurrentTime()
{
	GetLocalTime(&lt);
	timeinfo.tm_hour = lt.wHour;
	timeinfo.tm_mday = lt.wDay;
	timeinfo.tm_min = lt.wMinute;
	timeinfo.tm_mon = lt.wMonth;
	timeinfo.tm_sec = lt.wSecond;
	timeinfo.tm_wday = lt.wDayOfWeek;
	timeinfo.tm_year = lt.wYear;
}

// Writes date in m/d/y format to buffer.  Returns # of chars written
int DateTime::GetShortDate(char *buffer, int bufferlen)
{
	int outlen = 0;
	
	//outlen = sprintf(buffer, "%d/%d/%d", lt.wMonth, lt.wDay, lt.wYear);
	getCurrentTime();
	outlen = strftime(buffer, bufferlen, "%b %e, %Y", &timeinfo);
	return outlen;
}

int DateTime::GetDate(char *buffer, int bufferlen)
{
	int outlen = 0;
	//GetLocalTime(&lt);
	//outlen = sprintf(buffer, "%s %d, %d", months[lt.wMonth-1], lt.wDay, lt.wYear);
	getCurrentTime();
	//HACK - need to reduce month # by 1 (convert 1-indexed to 0-indexed) for strftime to get the right month
	timeinfo.tm_mon -= 1;
	// date formatted as <weekday> <month> <day> - e.g. Mon Oct 6.  The # before the day strips the leading zero
	outlen = strftime(buffer, bufferlen, "%a %b %#d", &timeinfo);
	return outlen;
}

int DateTime::GetTime(char *buffer, int bufferlen, bool bln24Hr)
{
	int outlen = 0;
	getCurrentTime();
	if (bln24Hr)
	{
		outlen = strftime(buffer, bufferlen, "%H:%M", &timeinfo);
	}
	else
	{
		outlen = strftime(buffer, bufferlen, "%#I:%M %p", &timeinfo);
	}
	return outlen;
}
