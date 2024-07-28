/**
@file
@brief Assertions used in the project
*/

#pragma once

#include <iostream>

#define UNUSED(x) (void)(x)

#define BASALT_ATTRIBUTE_NORETURN __attribute__((noreturn))

inline BASALT_ATTRIBUTE_NORETURN void assertionFailed(char const* expr,
                                                      char const* function,
                                                      char const* file,
                                                      long line) {
  std::cerr << "***** Assertion (" << expr << ") failed in " << function
            << ":\n"
            << file << ':' << line << ":" << std::endl;
  std::abort();
}

inline BASALT_ATTRIBUTE_NORETURN void assertionFailedMsg(char const* expr,
                                                         char const* msg,
                                                         char const* function,
                                                         char const* file,
                                                         long line) {
  std::cerr << "***** Assertion (" << expr << ") failed in " << function
            << ":\n"
            << file << ':' << line << ": " << msg << std::endl;
  std::abort();
}

inline BASALT_ATTRIBUTE_NORETURN void logFatal(char const* function,
                                               char const* file, long line) {
  std::cerr << "***** Fatal error in " << function << ":\n"
            << file << ':' << line << ":" << std::endl;
  std::abort();
}

inline BASALT_ATTRIBUTE_NORETURN void logFatalMsg(char const* msg,
                                                  char const* function,
                                                  char const* file, long line) {
  std::cerr << "***** Fatal error in " << function << ":\n"
            << file << ':' << line << ": " << msg << std::endl;
  std::abort();
}

#define BASALT_LIKELY(x) __builtin_expect(x, 1)

#if defined(BASALT_DISABLE_ASSERTS)

#define BASALT_ASSERT(expr) ((void)0)

#define BASALT_ASSERT_MSG(expr, msg) ((void)0)

#define BASALT_ASSERT_STREAM(expr, msg) ((void)0)

#else

#define BASALT_ASSERT(expr)                                              \
  (BASALT_LIKELY(!!(expr))                                               \
       ? ((void)0)                                                       \
       : ::assertionFailed(#expr, __PRETTY_FUNCTION__, __FILE__, \
                                   __LINE__))

#define BASALT_ASSERT_MSG(expr, msg)                                   \
  (BASALT_LIKELY(!!(expr))                                             \
       ? ((void)0)                                                     \
       : ::assertionFailedMsg(#expr, msg, __PRETTY_FUNCTION__, \
                                      __FILE__, __LINE__))

#define BASALT_ASSERT_STREAM(expr, msg)                                   \
  (BASALT_LIKELY(!!(expr))                                                \
       ? ((void)0)                                                        \
       : (std::cerr << msg << std::endl,                                  \
          ::assertionFailed(#expr, __PRETTY_FUNCTION__, __FILE__, \
                                    __LINE__)))

#endif

#define BASALT_LOG_FATAL(msg) \
  ::logFatalMsg(msg, __PRETTY_FUNCTION__, __FILE__, __LINE__)

#define BASALT_LOG_FATAL_STREAM(msg) \
  (std::cerr << msg << std::endl,    \
   ::logFatal(__PRETTY_FUNCTION__, __FILE__, __LINE__))
