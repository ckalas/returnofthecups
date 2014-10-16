#ifndef my_linux_types_h
#define my_linux_types_h

#ifdef __linux__
# include "linux/types.h"
#else
# include <stdint.h>
typedef int32_t __s32;
typedef uint8_t __u8;
typedef uint16_t __u16;
[... and so on for whatever other types your program uses ...]
#endif

#endif