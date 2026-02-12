/****************************************************************************
 * nuttx-config/include/board.h
 ****************************************************************************/

#pragma once

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

void sg2000_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
