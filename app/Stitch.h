#ifndef STITCH_H
#define STITCH_H

#include <stdbool.h>
#include <stddef.h>
#include "cJSON.h"

typedef void (*stitch_callback)(cJSON*);

int  STICH_Init(stitch_callback cb);
int  STICH_Settings(cJSON* settings);
void STICH_Path(cJSON* path);

#ifdef __cplusplus
}
#endif

#endif // VOD_H