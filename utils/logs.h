// logs.h

#ifndef __LOGS_H__
#define __LOGS_H__
#include <stdio.h>      /* Standard input/output definitions */
#include <stdarg.h>     // Needed for variable argument handling __VA_ARGS__, etc.

#include "timestamp.h"

#define LOG_LEVEL_INFO  3
#define LOG_LEVEL_DEBUG 2
#define LOG_LEVEL_WARN  1
#define LOG_LEVEL_ERROR 0

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_ERROR
#endif  // LOG_LEVEL

#define LOCAL_LOG_LEVEL(level, tag, format, ...) do {\
    if(level == LOG_LEVEL_ERROR)      { fprintf(stderr, "%lu: ERROR: %s: %s() line %i: %s: " format "\n", nanos(), __FILE__, __FUNCTION__, __LINE__, tag __VA_OPT__(,) __VA_ARGS__); } \
    else if(level == LOG_LEVEL_WARN)  { fprintf(stdout, "%lu:  WARN: %s: %s() line %i: %s: " format "\n", nanos(), __FILE__, __FUNCTION__, __LINE__, tag __VA_OPT__(,) __VA_ARGS__); } \
    else if(level == LOG_LEVEL_DEBUG) { fprintf(stdout, "%lu: DEBUG: %s: %s() line %i: %s: " format "\n", nanos(), __FILE__, __FUNCTION__, __LINE__, tag __VA_OPT__(,) __VA_ARGS__); } \
    else if(level == LOG_LEVEL_INFO)  { fprintf(stdout, "%lu:  INFO: %s: %s() line %i: %s: " format "\n", nanos(), __FILE__, __FUNCTION__, __LINE__, tag __VA_OPT__(,) __VA_ARGS__); } \
  } while (0)

#define LOG_LEVEL_LOCAL(level, tag, format, ...) do {\
                if(LOG_LEVEL >= level) LOCAL_LOG_LEVEL(level, tag, format __VA_OPT__(,) __VA_ARGS__); \
        } while (0)

#define LOGI(tag, format, ...)\
        LOG_LEVEL_LOCAL(LOG_LEVEL_INFO, tag, format __VA_OPT__(,) __VA_ARGS__)

#define LOGD(tag, format, ...)\
        LOG_LEVEL_LOCAL(LOG_LEVEL_DEBUG, tag, format __VA_OPT__(,) __VA_ARGS__)

#define LOGW(tag, format, ...)\
        LOG_LEVEL_LOCAL(LOG_LEVEL_WARN, tag, format __VA_OPT__(,) __VA_ARGS__)

#define LOGE(tag, format, ...)\
        LOG_LEVEL_LOCAL(LOG_LEVEL_ERROR, tag, format __VA_OPT__(,) __VA_ARGS__)

#endif // __LOGS_H__