@{
    # Shared GCC / Keil project layout. Paths are relative to software/.
    Name = 'gimbal_foc'
    IncludeDirectories = @(
        'Config'
        'App/Inc'
        'Communication/Inc'
        'BSP/Inc'
        'Control/Inc'
        'Drivers/CMSIS/Include'
        'Drivers/CMSIS/Device/ST/STM32F1xx/Include'
    )
    SourceGroups = @(
        @{ Name = 'Core'; Path = 'Core/Src'; Roles = @{ Main = 'main.c'; System = 'system_stm32f1xx.c' } }
        @{ Name = 'App'; Path = 'App/Src'; Roles = @{ App = 'app.c' } }
        @{ Name = 'Communication'; Path = 'Communication/Src'; Roles = @{ Console = 'console.c'; CanProtocol = 'can_protocol.c'; CanService = 'can_service.c' } }
        @{ Name = 'BSP'; Path = 'BSP/Src' }
        @{ Name = 'Control'; Path = 'Control/Src'; Roles = @{ Foc = 'foc.c'; FocParameters = 'foc_parameters.c' } }
    )
    HeaderGroups = @(
        @{ Name = 'Config'; Path = 'Config' }
    )
    Startup = @{
        Gcc = 'Core/Startup/startup_stm32f103xb_gcc.s'
        Armcc = 'Core/Startup/startup_stm32f103xb_arm.s'
    }
    Linker = @{
        Gcc = 'Core/Linker/STM32F103C8TX_FLASH.ld'
        Armcc = 'Core/Linker/STM32F103C8TX_FLASH.sct'
    }
}
