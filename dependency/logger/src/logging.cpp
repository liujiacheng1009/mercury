#include <sys/time.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <filesystem>

#include "mlog.h"
#include "logging.h"

#define DEFAULT_LOG_FILE_DIR "log"

namespace fs = std::filesystem;

Logging Logging::logging_instance_;

Logging::Logging()
    : initialized_(false),
      log_dir_prefix_(""),
      log_file_dir_(DEFAULT_LOG_FILE_DIR),
      log_to_file_(true),
      log_to_file_async_(true),
      log_min_level_(LOG_INFO)
{
}

Logging::~Logging()
{
  deinit_loggers();
}

void Logging::dump_logging_config()
{
  INFO_DUMP("---- mjlog configurations ----\n");
  INFO_DUMP("log file directory: %s\n", log_file_dir_.c_str());
  INFO_DUMP("log file name: %s\n", log_file_name_.c_str());
  INFO_DUMP("log to file: %s, mode: %s\n", log_to_file_ ? "yes" : "no", log_to_file_async_ ? "async" : "sync");
  INFO_DUMP("min log level: %s\n", log_level_enum_to_string(log_min_level_));

  std::lock_guard<std::mutex> lk(module_level_mutex_);
  for (auto &it : module_level_)
  {
    INFO_DUMP("%s\t %s\n", it.first.c_str(), log_level_enum_to_string(it.second));
  }
}

// Single-threaded initialization procedure for mlog
int Logging::init(const std::string &app_name, bool log_to_file, std::string &log_file_dir, log_level_e min_log_level)
{
  std::lock_guard<std::mutex> lk(logging_mutex_);
  if (initialized_)
  {
    return 0;
  }
  set_log_to_file(log_to_file);
  set_min_log_level(min_log_level);
  set_log_file_dir(log_file_dir);
  return init_unlocked(app_name);
}

int Logging::init_unlocked(const std::string &app_name)
{
  init_loggers(get_base_name(app_name.c_str()));
  initialized_ = true;
  return 0;
}

const char *Logging::get_base_name(const char *filepath) const
{
  const char *p = strrchr(filepath, PATH_SEPARATOR);
  return (p != nullptr) ? (p + 1) : filepath;
}

int Logging::init_loggers(const std::string &app_name)
{
  if (!log_to_file_)
  {
    return -1;
  }
  if (fs::path(log_file_dir_).is_relative())
  {
    log_file_dir_ = fs::path(log_dir_prefix_).append(log_file_dir_);
  }
  std::error_code ec;
  bool ret = fs::create_directories(log_file_dir_, ec);
  (void)ret;
  std::string filepath = log_file_dir_;
  if (!filepath.empty() && filepath[filepath.length() - 1] != PATH_SEPARATOR)
  {
    filepath.append(1, PATH_SEPARATOR);
  }

  struct timeval tv;
  gettimeofday(&tv, NULL);
  struct tm tm_time;
  localtime_r(&tv.tv_sec, &tm_time);
  char dt_buf[32];
  snprintf(dt_buf, sizeof(dt_buf) / sizeof(dt_buf[0]),
           "%04d%02d%02dT%02d%02d%02d", tm_time.tm_year + 1900,
           tm_time.tm_mon + 1, tm_time.tm_mday, tm_time.tm_hour,
           tm_time.tm_min, tm_time.tm_sec);

  log_file_name_ = get_base_name(app_name.c_str());
  log_file_name_ += "_";
  log_file_name_ += dt_buf;
  log_file_name_ += ".log";
  filepath += log_file_name_;
  auto logger = std::make_shared<FileLogger>(filepath);
  if (log_to_file_async_)
    async_log_thread_.add_logger(logger);
  else
    loggers_.push_back(logger);

  if (async_log_thread_.logger_size())
  {
    async_log_thread_.start();
  }

  return 0;
}

int Logging::deinit_loggers()
{
  // force flush all remaining logs
  flush();

  loggers_.clear();

  return 0;
}

int Logging::flush()
{
  std::lock_guard<std::mutex> lk(logging_mutex_);
  return flush_unlocked();
}

int Logging::flush_unlocked()
{
  if (async_log_thread_.logger_size())
  {
    AsyncLogMsg new_log(async_log_msg_type::flush);
    async_log_thread_.post_async_msg(std::move(new_log));
  }

  for (auto it : loggers_)
  {
    it->flush();
  }

  return 0;
}

int Logging::sync_to_disk()
{
  std::lock_guard<std::mutex> lk(logging_mutex_);
  return sync_to_disk_unlocked();
}

int Logging::sync_to_disk_unlocked()
{

  if (async_log_thread_.logger_size())
  {
    AsyncLogMsg new_log(async_log_msg_type::sync_to_disk);
    async_log_thread_.post_async_msg(std::move(new_log));
  }

  for (auto it : loggers_)
  {
    it->sync_to_disk();
  }

  return 0;
}

// Query whether we should log or not, according to the specified level and
// module name. This will also add a new entry with default values in case
// it does not exist.
bool Logging::should_log_on_module_level(const std::string &module,
                                         log_level_e level)
{
  std::lock_guard<std::mutex> lk(module_level_mutex_);

  // NOTE: Fast return for logging checking, to avoid unnecessary overhead.
  // TODO: Currently, we support two type of loggers. If there're more types,
  // this condition should be updated accrodingly.
  if (!log_to_file_)
  {
    return false;
  }

  if (module_level_.find(module) == module_level_.end())
  {
    module_level_[module] = log_min_level_;
  }
  return level >= module_level_[module];
}

int Logging::write_log(const char *module, log_level_e level, const char *filename,
                       int line, char *body_buf, int body_len)
{
  std::lock_guard<std::mutex> lk(logging_mutex_);

  // In case the `init_logging()` not been called, we just provide
  // a simple way to ensure the file logger initialized with default values:
  // write log contents to a file named program name plus date-time string in
  // `/tmp`
  if (!initialized_)
  {
    ERROR_DUMP("write log message without calling init_logging(). "
               "Initialize with defaults.\n");
    exit(-1);
  }

  std::shared_ptr<std::string> log_string_ptr = std::make_shared<std::string>(LOG_BUF_LEN, 0x00);
  char *buf = const_cast<char *>(log_string_ptr->data());

  // Firstly, print the message header
  // This buffer is big enough, and not likely to be truncated
  // struct timeval tv;
  // gettimeofday(&tv, NULL);
  struct timespec tv;
  clock_gettime(CLOCK_REALTIME, &tv);
  struct tm local_tm;
  localtime_r(&tv.tv_sec, &local_tm);
  int header_len =
      snprintf(buf, LOG_BUF_LEN,
               "%04d/%02d/%02d %02d:%02d:%02d.%03d %s %s [%s:%d] ",
               local_tm.tm_year + 1900, local_tm.tm_mon + 1, local_tm.tm_mday,
               local_tm.tm_hour, local_tm.tm_min, local_tm.tm_sec,
               static_cast<int>(tv.tv_nsec / 1000000),
               log_level_enum_to_string(level), module, filename, line);
  if (header_len < 0)
  {
    ERROR_DUMP("Error on snprintf: return %d, errno %d\n", header_len, errno);
    return -1;
  }

  int bytes_printed = header_len + body_len;

  // ensure '\n' is at the end of log message
  if (bytes_printed >= LOG_BUF_LEN)
  { // truncated
    memcpy(&buf[header_len], body_buf, LOG_BUF_LEN - header_len);
    bytes_printed = LOG_BUF_LEN;
    buf[LOG_BUF_LEN - 1] = '\n'; // overwrite the last byte to '\n'
  }
  else
  {
    memcpy(&buf[header_len], body_buf, body_len);
    if (buf[bytes_printed - 1] != '\n')
    {
      buf[bytes_printed++] = '\n'; // append '\n'
    }
  }

  log_string_ptr->resize(bytes_printed);

  if (async_log_thread_.logger_size())
  {
    AsyncLogMsg new_log(log_string_ptr, async_log_msg_type::log);
    async_log_thread_.post_async_msg(std::move(new_log));
  }

  for (auto &it : loggers_)
  {
    it->write(buf, bytes_printed);
  }

  return 0;
}

int Logging::get_current_log_dir(std::string &log_dir)
{
  int ret = -1;
  if (initialized_)
  {
    log_dir = log_file_dir_;
    ret = 0;
  }
  return ret;
}

std::string Logging::get_current_mlog_file()
{
  std::string filepath = log_file_dir_;
  if (!filepath.empty() && filepath[filepath.length() - 1] != PATH_SEPARATOR)
  {
    filepath.append(1, PATH_SEPARATOR);
  }

  filepath += log_file_name_;
  return filepath;
}

FileLogger::FileLogger(const std::string &filepath)
    : _file(nullptr)
{
  open(filepath);
}

FileLogger::~FileLogger()
{
  close();
}

int FileLogger::open(const std::string &filepath)
{
  const char *full_filename = filepath.c_str();
  _file = std::fopen(full_filename, "w+");
  if (_file == nullptr)
  {
    ERROR_DUMP("Failed to create log file: %s, %d\n", full_filename, errno);
    return -1;
  }
  return 0;
}

int FileLogger::close()
{
  if (_file != nullptr)
  {
    std::fclose(_file);
    _file = nullptr;
  }
  return 0;
}

int FileLogger::write(const char *message, int len)
{
  if (_file == nullptr)
  {
    return -1;
  }

  int written_len = std::fwrite(message, 1, len, _file);
  if (written_len < len)
  {
    if (errno == ENOSPC)
    {
      // TODO: prevent logging any subsequent messages,
      // or just print out an error message.
      ERROR_DUMP("Failed to write file due to disk full.\n");
      return written_len;
    }
  }

  return written_len;
}

int FileLogger::flush()
{
  return (_file == nullptr) ? -1 : std::fflush(_file);
}

int FileLogger::sync_to_disk()
{
  return (_file == nullptr) ? -1 : fsync(fileno(_file));
}
