#pragma once

#include <cstdio>
#include <string>
#include <vector>
#include <map>
#include <mutex>

#include "log_thread.h"

typedef enum
{
  LOG_VERBOSE = 0,
  LOG_INFO = 1,
  LOG_WARN = 2,
  LOG_ERROR = 3,
  LOG_FATAL = 4,
  LOG_NUM_MLOG_LEVEL // always be the last one for counting purpose
} log_level_e;

#define LOG_BUF_LEN 1024

#define INFO_DUMP(...) fprintf(stderr, __VA_ARGS__)
#define ERROR_DUMP(...) fprintf(stderr, __VA_ARGS__)

class Logging;
class Logger;
class FileLogger;
class LogThread;

class Logging
{
public:
  Logging();
  virtual ~Logging();
  static Logging *Instance() { return &logging_instance_; }

  bool initialized() { return initialized_; }

public:
  int init(const std::string &app_name, bool log_to_file, std::string &log_file_dir, log_level_e min_log_level);
  int flush();
  int flush_unlocked();
  int sync_to_disk();
  int sync_to_disk_unlocked();
  void dump_logging_config();
  bool should_log_on_module_level(const std::string &module, log_level_e level);
  bool should_log_to_file() const { return log_to_file_; }
  void set_log_to_file(bool log_to_file) { log_to_file_ = log_to_file; }
  void set_min_log_level(log_level_e min_log_level) { log_min_level_ = min_log_level; }
  void set_log_file_dir(std::string &log_file_dir) { log_dir_prefix_ = log_file_dir; }
  int write_log(const char *module, log_level_e level, const char *filename, int line, char *body_buf, int body_len);

  /**
   * @brief Get the current log dir
   *
   * @param log_dir
   * @return int
   */
  int get_current_log_dir(std::string &log_dir);

  /**
   * @brief Get the current mlog file full path
   *
   * @return std::string
   */
  std::string get_current_mlog_file();

private:
  int init_unlocked(const std::string &app_name);
  int init_loggers(const std::string &app_name);
  int deinit_loggers();
  const char *get_base_name(const char *filepath) const;

private:
  static Logging logging_instance_;
  bool initialized_;
  std::vector<std::shared_ptr<Logger>> loggers_;
  std::string log_dir_prefix_;
  std::string log_file_dir_;
  std::string log_file_name_;
  bool log_to_file_;
  bool log_to_file_async_;
  log_level_e log_min_level_;
  std::map<std::string, log_level_e> module_level_;
  std::mutex module_level_mutex_;
  std::mutex logging_mutex_;
  LogThread async_log_thread_;
};

// Logger defines some required interfaces which used for acutal logging task.
// We assume the `message` data provided here are already well formatted and
// should be ready for logging directly.
// Subclasses need do actual logging work only, i.e. writing to file, console,
// saving to database, etc
class Logger
{
public:
  Logger() {}
  virtual ~Logger() {}

public:
  // Do logging operation on final text message
  virtual int write(const char *message, int len) = 0;
  // flush to ensure all underlying buffered data are done
  virtual int flush() = 0;
  // sync all data to disk
  virtual int sync_to_disk() = 0;
};

// FileLogger for logging message to file.
class FileLogger : public Logger
{
public:
  FileLogger(const std::string &filepath);
  virtual ~FileLogger();

public:
  virtual int write(const char *message, int len) override;
  virtual int flush() override;
  virtual int sync_to_disk() override;

private:
  int open(const std::string &filepath);
  int close();

private:
  std::FILE *_file;
};
