#ifndef STITCH_H
#define STITCH_H

#include <stdbool.h>
#include <stddef.h>
#include "cJSON.h"

typedef void (*stitch_callback)(cJSON*);

int  Stitch_Init(stitch_callback cb);
int  Stitch_Settings(cJSON* settings);
void Stitch_Path(cJSON* path);

#ifdef __cplusplus
}
#endif

#endif // VOD_H