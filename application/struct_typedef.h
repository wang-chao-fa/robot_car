#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

// 优先使用标准库的精确宽度整数类型，避免重定义冲突
#if defined(__ARMCC_VERSION) || defined(__GNUC__) || defined(__ICCARM__)
  #include <stdint.h>
#else
  // 仅在无标准库可用时才手动定义
  typedef signed char int8_t;
  typedef signed short int int16_t;
  typedef signed int int32_t;
  typedef signed long long int64_t;

  typedef unsigned char uint8_t;
  typedef unsigned short int uint16_t;
  typedef unsigned int uint32_t;
  typedef unsigned long long uint64_t;
#endif

// 自定义类型 (标准库中没有，保留)
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;


#endif



