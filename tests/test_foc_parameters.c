/* Parameter catalog/config-copy contract; no board, IRQ or CAN dependency. */
#include "foc_parameters.h"
#include "motor_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    FocConfig config, expected, before;
    float value;
    const FocParameterDescriptor *parameter;
    memset(&config,0,sizeof(config));
    config.fast_dt_s = 0.0005f;
    config.pole_pairs = 11u;
    config.overcurrent_trip_a = 1.5f;
    FocParameters_SetDefaults(&config);
    assert(config.fast_dt_s == 0.0005f && config.pole_pairs == 11u);
    assert(config.overcurrent_trip_a == 1.5f); /* Not in writable/default catalog. */
    assert(config.current_kp_v_per_a == MOTOR_CURRENT_KP);
    assert(config.current_ki_v_per_a_s == MOTOR_CURRENT_KI);
    assert(config.iq_limit_a == MOTOR_CURRENT_LIMIT_A);
    assert(config.voltage_limit_v == MOTOR_VOLTAGE_LIMIT_V);
    assert(config.vbus_v == MOTOR_BUS_VOLTAGE_V);
    assert(FOC_PARAMETER_COUNT == 9);
    for (unsigned id = 1u; id <= 9u; ++id) {
        parameter = FocParameters_Find(id);
        assert(parameter != NULL && (unsigned)parameter->id == id);
        assert(parameter->invalidates_zero == (id == 9u));
        assert(FocParameters_Read(&config,id,&value));
        assert(value == parameter->default_value);
        assert(FocParameters_Validate(id,parameter->minimum));
        assert(FocParameters_Validate(id,parameter->maximum));
    }
    parameter = FocParameters_Find(1u);
    assert(parameter->minimum == 0.0f && parameter->maximum == 5.0f);
    parameter = FocParameters_Find(3u);
    assert(parameter->maximum == MOTOR_IQ_REQUEST_MAX_A);
    assert(FocParameters_WriteConfig(&config,3u,MOTOR_IQ_REQUEST_MAX_A));
    assert(config.iq_limit_a == MOTOR_IQ_REQUEST_MAX_A);
    assert(config.overcurrent_trip_a == 1.5f);
    assert(!FocParameters_WriteConfig(&config,3u,MOTOR_IQ_REQUEST_MAX_A + 0.0001f));
    assert(!FocParameters_WriteConfig(&config,3u,3.3001f));
    assert(!FocParameters_WriteConfig(&config,3u,-3.3f));
    /* Both profiles share the 3.3 A ceiling, so 3.3 A is writable. */
    assert(FocParameters_WriteConfig(&config,3u,3.3f));
    assert(config.iq_limit_a == 3.3f);
    parameter = FocParameters_Find(9u);
    assert(parameter->minimum == 8.0f && parameter->maximum == 30.0f);
    expected = config;
    expected.current_kp_v_per_a = 2.25f;
    assert(FocParameters_WriteConfig(&config,1u,2.25f));
    assert(memcmp(&config,&expected,sizeof(config)) == 0);
    assert(FocParameters_Read(&config,1u,&value) && value == 2.25f);
    before = config;
    assert(!FocParameters_WriteConfig(&config,1u,5.01f));
    assert(!FocParameters_WriteConfig(&config,1u,-0.01f));
    assert(!FocParameters_WriteConfig(&config,1u,NAN));
    assert(!FocParameters_WriteConfig(&config,1u,INFINITY));
    assert(!FocParameters_WriteConfig(&config,0u,1.0f));
    assert(!FocParameters_WriteConfig(&config,10u,1.0f));
    assert(memcmp(&config,&before,sizeof(config)) == 0);
    assert(!FocParameters_WriteConfig(NULL,1u,1.0f));
    assert(!FocParameters_Read(NULL,1u,&value));
    assert(!FocParameters_Read(&config,1u,NULL));
    assert(!FocParameters_Read(&config,10u,&value));
    assert(!FocParameters_Validate(0u,1.0f));
    assert(FocParameters_Find(10u) == NULL);
    FocParameters_SetDefaults(NULL);
    puts("FOC parameter catalog/config-copy tests passed (profile defaults preserved).");
    return 0;
}
