#ifndef RR_UDP_SERVER__VISIBILITY_CONTROL_H_
#define RR_UDP_SERVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RR_UDP_SERVER_EXPORT __attribute__ ((dllexport))
    #define RR_UDP_SERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define RR_UDP_SERVER_EXPORT __declspec(dllexport)
    #define RR_UDP_SERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef RR_UDP_SERVER_BUILDING_LIBRARY
    #define RR_UDP_SERVER_PUBLIC RR_UDP_SERVER_EXPORT
  #else
    #define RR_UDP_SERVER_PUBLIC RR_UDP_SERVER_IMPORT
  #endif
  #define RR_UDP_SERVER_PUBLIC_TYPE RR_UDP_SERVER_PUBLIC
  #define RR_UDP_SERVER_LOCAL
#else
  #define RR_UDP_SERVER_EXPORT __attribute__ ((visibility("default")))
  #define RR_UDP_SERVER_IMPORT
  #if __GNUC__ >= 4
    #define RR_UDP_SERVER_PUBLIC __attribute__ ((visibility("default")))
    #define RR_UDP_SERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RR_UDP_SERVER_PUBLIC
    #define RR_UDP_SERVER_LOCAL
  #endif
  #define RR_UDP_SERVER_PUBLIC_TYPE
#endif

#endif  // RR_UDP_SERVER__VISIBILITY_CONTROL_H_
