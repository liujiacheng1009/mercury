#include "mlog.h"
#include <stdarg.h>

const char *log_level_enum_to_string(log_level_e level)
{
  switch (level)
  {
  case LOG_VERBOSE:
    return "VERBOSE";
  case LOG_INFO:
    return "INFO";
  case LOG_WARN:
    return "WARN";
  case LOG_ERROR:
    return "ERROR";
  case LOG_FATAL:
    return "FATAL";
  default:
    return "UNDEFINED";
  }
}

log_level_e log_level_string_to_enum(const char *level)
{
  if (strcmp(level, "VERBOSE") == 0)
  {
    return LOG_VERBOSE;
  }
  else if (strcmp(level, "INFO") == 0)
  {
    return LOG_INFO;
  }
  else if (strcmp(level, "WARN") == 0)
  {
    return LOG_WARN;
  }
  else if (strcmp(level, "ERROR") == 0)
  {
    return LOG_ERROR;
  }
  else if (strcmp(level, "FATAL") == 0)
  {
    return LOG_FATAL;
  }
  return LOG_NUM_MLOG_LEVEL;
}

int init_logging(const char *app_name, bool log_to_file, std::string log_file_dir, log_level_e min_log_level)
{
  return Logging::Instance()->init(app_name, log_to_file, log_file_dir, min_log_level);
}

int flush()
{
  return Logging::Instance()->flush();
}

int sync_to_disk()
{
  return Logging::Instance()->sync_to_disk();
}

void dump_mlog_settings()
{
  Logging::Instance()->dump_logging_config();
}

bool should_log_on_module_level(const char *module, log_level_e level)
{
  if (!Logging::Instance()->initialized())
  {
    return false;
  }

  return Logging::Instance()->should_log_on_module_level(module, level);
}

int write_log(const char *module, log_level_e level, const char *filename,
              int line, const char *format, ...)
{

  char buf[LOG_BUF_LEN];

  // quickly return if we really do not need to log
  if (!Logging::Instance()->should_log_to_file())
  {
    return 0;
  }

  char *message_body = &buf[0];
  size_t available = LOG_BUF_LEN;

  va_list ap;
  va_start(ap, format);
  int body_len = vsnprintf(message_body, available, format, ap);
  va_end(ap);
  if (body_len < 0)
  {
    ERROR_DUMP("Error on vsnprintf: return %d, errno %d\n", body_len, errno);
    return -1;
  }

  // we don't care whether the buffer is null-terminated
  return Logging::Instance()->write_log(module, level, filename, line, buf, body_len);
}

std::string get_current_mlog_file()
{
  return Logging::Instance()->get_current_mlog_file();
}

std::string get_current_log_dir()
{
  std::string current_log_dir;
  int ret = Logging::Instance()->get_current_log_dir(current_log_dir);
  (void)ret;
  return current_log_dir;
}
