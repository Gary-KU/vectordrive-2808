#include "foc_parameters.h"
#include "motor_config.h"

#include <math.h>
#include <string.h>

const FocParameterDescriptor FocParameterTable[FOC_PARAMETER_COUNT] = {
#define FOC_PARAMETER(symbol,id,field,default_macro,minimum,maximum,invalidates_zero) \
    {FOC_PARAMETER_##symbol, offsetof(FocConfig,field), default_macro, minimum, maximum, invalidates_zero != 0},
#include "parameter_catalog.def"
#undef FOC_PARAMETER
};

typedef char FocParameterWireCountMustRemainNine[(FOC_PARAMETER_COUNT == 9) ? 1 : -1];

const FocParameterDescriptor *FocParameters_Find(unsigned id)
{
    for (unsigned index = 0u; index < FOC_PARAMETER_COUNT; ++index)
        if ((unsigned)FocParameterTable[index].id == id) return &FocParameterTable[index];
    return NULL;
}

bool FocParameters_Read(const FocConfig *config, unsigned id, float *value)
{
    const FocParameterDescriptor *parameter = FocParameters_Find(id);
    if (config == NULL || value == NULL || parameter == NULL) return false;
    memcpy(value, (const unsigned char *)config + parameter->field_offset, sizeof(*value));
    return true;
}

bool FocParameters_Validate(unsigned id, float value)
{
    const FocParameterDescriptor *parameter = FocParameters_Find(id);
    return parameter != NULL && isfinite(value) &&
        value >= parameter->minimum && value <= parameter->maximum;
}

bool FocParameters_WriteConfig(FocConfig *config, unsigned id, float value)
{
    const FocParameterDescriptor *parameter = FocParameters_Find(id);
    if (config == NULL || parameter == NULL || !FocParameters_Validate(id,value)) return false;
    memcpy((unsigned char *)config + parameter->field_offset, &value, sizeof(value));
    return true;
}

void FocParameters_SetDefaults(FocConfig *config)
{
    if (config == NULL) return;
    for (unsigned index = 0u; index < FOC_PARAMETER_COUNT; ++index) {
        const FocParameterDescriptor *parameter = &FocParameterTable[index];
        memcpy((unsigned char *)config + parameter->field_offset,
               &parameter->default_value, sizeof(parameter->default_value));
    }
}
