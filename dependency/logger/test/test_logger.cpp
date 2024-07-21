#include <iostream>
#include <unistd.h>

#include "mlog.h"

int main(int argc, char *argv[])
{
  std::string log_path = "";
  init_logging(argv[0], true, log_path, log_level_e::LOG_INFO);

  const float kPI = 3.1415926f;
  LOG(module1, INFO, "%s: Hello, there. [PI = %f?]", argv[0], kPI);
  LOG(module2, WARN, "Bye!");
  LOG(module3, VERBOSE, "Can you see this log?");

  float x_pos = 1.0f, y_pos = 2.0f, z_pos = 3.14f;

  APROF_INIT(dump_alog_settings, true);
  APROF_IN(dump_alog_settings);
  dump_mlog_settings();
  APROF_OUT(dump_alog_settings);

  for (size_t i = 0; i < 100; i++)
  {
    LOG_LEVEL(vio, INFO, "x_pos_est %f, y_pos_est %f, z_pos_est %f", x_pos + i * 0.1, y_pos + i * 0.1, z_pos + i * 0.1);
    LOG_LEVEL(vio, INFO, "x_pos %f, y_pos %f, z_pos %f", x_pos + i * 0.1, y_pos + i * 0.1, z_pos + i * 0.1);
    usleep(10 * 1000);
  }

  return 0;
}