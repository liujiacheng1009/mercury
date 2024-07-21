#pragma once

#include <sys/time.h>
#include <inttypes.h>
#include <string>
#include <string.h>
#include "logging.h"
#include "log_thread.h"

#define __STDC_FORMAT_MACROS

#define PATH_SEPARATOR '/'

#define __FNAME__ (strrchr(__FILE__, PATH_SEPARATOR) ? strrchr(__FILE__, PATH_SEPARATOR) + 1 : __FILE__)

#define LOG(module, level, format, ...)                                \
  do                                                                     \
  {                                                                      \
    if (should_log_on_module_level(#module, LOG_##level))  \
    {                                                                    \
      write_log(#module, LOG_##level, __FNAME__, __LINE__, \
                            format, ##__VA_ARGS__);                      \
    }                                                                    \
  } while (0)

#define LOG_MODULE_LEVEL(module, level, format, ...)              \
  do                                                                \
  {                                                                 \
    struct timeval __tv                                             \
    {                                                               \
    };                                                              \
    gettimeofday(&__tv, NULL);                                      \
    uint64_t __ts = __tv.tv_usec + (uint64_t)__tv.tv_sec * 1000000; \
    LOG(module, level,                                            \
          "p[%s:%d:%" PRIu64 ":" format "]",                        \
          __FNAME__, __LINE__, __ts, ##__VA_ARGS__);                \
  } while (0)


#define LOG_LEVEL(module, level, format, ...)               \
  do                                                          \
  {                                                           \
    LOG_MODULE_LEVEL(module, level, format, ##__VA_ARGS__); \
  } while (0)


#define APROF_INIT(name, enable)                            \
  static struct timeval in_timeval_profile_##name##_point;  \
  static struct timeval out_timeval_profile_##name##_point; \
  static long sum_time_profile_##name##_point = 0;          \
  static long total_processed_frames_##name##_point = 0;    \
  static bool enable_profile_##name = true

#define APROF_IN(name)                                      \
  do                                                        \
  {                                                         \
    gettimeofday(&in_timeval_profile_##name##_point, NULL); \
  } while (0)

#define APROF_OUT_SUM_LEVEL(module, level, name, sum)                                              \
  do                                                                                               \
  {                                                                                                \
    gettimeofday(&out_timeval_profile_##name##_point, NULL);                                       \
    sum_time_profile_##name##_point += (out_timeval_profile_##name##_point.tv_usec -               \
                                        in_timeval_profile_##name##_point.tv_usec) +               \
                                       1000000 *                                                   \
                                           (out_timeval_profile_##name##_point.tv_sec -            \
                                            in_timeval_profile_##name##_point.tv_sec);             \
    total_processed_frames_##name##_point++;                                                       \
    if (enable_profile_##name == true && sum <= total_processed_frames_##name##_point)             \
    {                                                                                              \
      LOG_MODULE_LEVEL(module, level, "%s %d", #name,                                            \
                         sum_time_profile_##name##_point / total_processed_frames_##name##_point); \
      sum_time_profile_##name##_point = 0;                                                         \
      total_processed_frames_##name##_point = 0;                                                   \
    }                                                                                              \
  } while (0)

#define APROF_OUT_MODULE_LEVEL(module, level, name) APROF_OUT_SUM_LEVEL(module, level, name, 1)
#define APROF_OUT_LEVEL(level, name) APROF_OUT_SUM_LEVEL(APROF, level, name, 1)
#define APROF_OUT(name) APROF_OUT_LEVEL(INFO, name)

#define IS_VALID_MLOG_LEVEL(level) \
  ((level >= LOG_VERBOSE) &&     \
   (level < LOG_NUM_MLOG_LEVEL))

const char *log_level_enum_to_string(log_level_e level);

log_level_e log_level_string_to_enum(const char *str);

int init_logging(const char *app_name, bool log_to_file = true, std::string log_file_dir = "", log_level_e min_log_level = log_level_e::LOG_INFO);

int flush();

int sync_to_disk();

void dump_mlog_settings();

bool should_log_on_module_level(const char *module, log_level_e level);

int write_log(const char *module, log_level_e level, const char *filename,
              int line, const char *format, ...);

std::string get_current_mlog_file();

std::string get_current_log_dir();
