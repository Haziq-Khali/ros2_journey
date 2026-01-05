#ifndef ORDER_HAZIQ_PLUGINS__VISIBILITY_CONTROL_H_
#define ORDER_HAZIQ_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ORDER_HAZIQ_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define ORDER_HAZIQ_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define ORDER_HAZIQ_PLUGINS_EXPORT __declspec(dllexport)
    #define ORDER_HAZIQ_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ORDER_HAZIQ_PLUGINS_BUILDING_LIBRARY
    #define ORDER_HAZIQ_PLUGINS_PUBLIC ORDER_HAZIQ_PLUGINS_EXPORT
  #else
    #define ORDER_HAZIQ_PLUGINS_PUBLIC ORDER_HAZIQ_PLUGINS_IMPORT
  #endif
  #define ORDER_HAZIQ_PLUGINS_PUBLIC_TYPE ORDER_HAZIQ_PLUGINS_PUBLIC
  #define ORDER_HAZIQ_PLUGINS_LOCAL
#else
  #define ORDER_HAZIQ_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define ORDER_HAZIQ_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define ORDER_HAZIQ_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define ORDER_HAZIQ_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ORDER_HAZIQ_PLUGINS_PUBLIC
    #define ORDER_HAZIQ_PLUGINS_LOCAL
  #endif
  #define ORDER_HAZIQ_PLUGINS_PUBLIC_TYPE
#endif

#endif  // ORDER_HAZIQ_PLUGINS__VISIBILITY_CONTROL_H_
