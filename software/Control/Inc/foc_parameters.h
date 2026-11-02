#ifndef GIMBAL_FOC_PARAMETERS_H
#define GIMBAL_FOC_PARAMETERS_H

#include "foc.h"
#include <stddef.h>

/* Wire IDs, field mapping and bounds all come from the shared catalog. */
typedef enum {
#define FOC_PARAMETER(symbol,id,field,default_macro,minimum,maximum,invalidates_zero) \
    FOC_PARAMETER_##symbol = id,
#include "parameter_catalog.def"
#undef FOC_PARAMETER
    FOC_PARAMETER_COUNT = 0
#define FOC_PARAMETER(symbol,id,field,default_macro,minimum,maximum,invalidates_zero) + 1
#include "parameter_catalog.def"
#undef FOC_PARAMETER
} FocParameterId;

typedef struct {
    FocParameterId id;
    size_t field_offset;
    float default_value;
    float minimum;
    float maximum;
    bool invalidates_zero;
} FocParameterDescriptor;

extern const FocParameterDescriptor FocParameterTable[FOC_PARAMETER_COUNT];

const FocParameterDescriptor *FocParameters_Find(unsigned id);
bool FocParameters_Read(const FocConfig *config, unsigned id, float *value);
bool FocParameters_Validate(unsigned id, float value);
/* Only changes the selected field in a caller-owned config/copy. Application
 * STOP/EN/fault gates, coupled safety checks and controller reinit stay in App. */
bool FocParameters_WriteConfig(FocConfig *config, unsigned id, float value);
void FocParameters_SetDefaults(FocConfig *config);

#endif
