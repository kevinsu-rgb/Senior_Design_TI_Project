################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../linker.cmd 

SYSCFG_SRCS += \
../example.syscfg 

C_SRCS += \
../ADC_testbuf.c \
../dpc.c \
./syscfg/ti_dpl_config.c \
./syscfg/ti_drivers_config.c \
./syscfg/ti_drivers_open_close.c \
./syscfg/ti_pinmux_config.c \
./syscfg/ti_power_clock_config.c \
./syscfg/ti_board_config.c \
./syscfg/ti_board_open_close.c \
../factory_cal.c \
../interrupts.c \
../main.c \
../mmw_cli.c \
../mmw_demo_utils.c \
../mmw_flash_cal.c \
../mmwave_control_config.c \
../monitors.c \
../motion_detect.c \
../power_management.c \
../range_phase_bias_measurement.c \
../tracker_utils.c 

GEN_FILES += \
./syscfg/ti_dpl_config.c \
./syscfg/ti_drivers_config.c \
./syscfg/ti_drivers_open_close.c \
./syscfg/ti_pinmux_config.c \
./syscfg/ti_power_clock_config.c \
./syscfg/ti_board_config.c \
./syscfg/ti_board_open_close.c 

GEN_MISC_DIRS += \
./syscfg 

C_DEPS += \
./ADC_testbuf.d \
./dpc.d \
./syscfg/ti_dpl_config.d \
./syscfg/ti_drivers_config.d \
./syscfg/ti_drivers_open_close.d \
./syscfg/ti_pinmux_config.d \
./syscfg/ti_power_clock_config.d \
./syscfg/ti_board_config.d \
./syscfg/ti_board_open_close.d \
./factory_cal.d \
./interrupts.d \
./main.d \
./mmw_cli.d \
./mmw_demo_utils.d \
./mmw_flash_cal.d \
./mmwave_control_config.d \
./monitors.d \
./motion_detect.d \
./power_management.d \
./range_phase_bias_measurement.d \
./tracker_utils.d 

OBJS += \
./ADC_testbuf.o \
./dpc.o \
./syscfg/ti_dpl_config.o \
./syscfg/ti_drivers_config.o \
./syscfg/ti_drivers_open_close.o \
./syscfg/ti_pinmux_config.o \
./syscfg/ti_power_clock_config.o \
./syscfg/ti_board_config.o \
./syscfg/ti_board_open_close.o \
./factory_cal.o \
./interrupts.o \
./main.o \
./mmw_cli.o \
./mmw_demo_utils.o \
./mmw_flash_cal.o \
./mmwave_control_config.o \
./monitors.o \
./motion_detect.o \
./power_management.o \
./range_phase_bias_measurement.o \
./tracker_utils.o 

GEN_MISC_FILES += \
./syscfg/ti_dpl_config.h \
./syscfg/ti_drivers_config.h \
./syscfg/ti_drivers_open_close.h \
./syscfg/ti_board_config.h \
./syscfg/ti_board_open_close.h \
./syscfg/ti_cli_mpd_demo_config.h 

GEN_MISC_DIRS__QUOTED += \
"syscfg" 

OBJS__QUOTED += \
"ADC_testbuf.o" \
"dpc.o" \
"syscfg\ti_dpl_config.o" \
"syscfg\ti_drivers_config.o" \
"syscfg\ti_drivers_open_close.o" \
"syscfg\ti_pinmux_config.o" \
"syscfg\ti_power_clock_config.o" \
"syscfg\ti_board_config.o" \
"syscfg\ti_board_open_close.o" \
"factory_cal.o" \
"interrupts.o" \
"main.o" \
"mmw_cli.o" \
"mmw_demo_utils.o" \
"mmw_flash_cal.o" \
"mmwave_control_config.o" \
"monitors.o" \
"motion_detect.o" \
"power_management.o" \
"range_phase_bias_measurement.o" \
"tracker_utils.o" 

GEN_MISC_FILES__QUOTED += \
"syscfg\ti_dpl_config.h" \
"syscfg\ti_drivers_config.h" \
"syscfg\ti_drivers_open_close.h" \
"syscfg\ti_board_config.h" \
"syscfg\ti_board_open_close.h" \
"syscfg\ti_cli_mpd_demo_config.h" 

C_DEPS__QUOTED += \
"ADC_testbuf.d" \
"dpc.d" \
"syscfg\ti_dpl_config.d" \
"syscfg\ti_drivers_config.d" \
"syscfg\ti_drivers_open_close.d" \
"syscfg\ti_pinmux_config.d" \
"syscfg\ti_power_clock_config.d" \
"syscfg\ti_board_config.d" \
"syscfg\ti_board_open_close.d" \
"factory_cal.d" \
"interrupts.d" \
"main.d" \
"mmw_cli.d" \
"mmw_demo_utils.d" \
"mmw_flash_cal.d" \
"mmwave_control_config.d" \
"monitors.d" \
"motion_detect.d" \
"power_management.d" \
"range_phase_bias_measurement.d" \
"tracker_utils.d" 

GEN_FILES__QUOTED += \
"syscfg\ti_dpl_config.c" \
"syscfg\ti_drivers_config.c" \
"syscfg\ti_drivers_open_close.c" \
"syscfg\ti_pinmux_config.c" \
"syscfg\ti_power_clock_config.c" \
"syscfg\ti_board_config.c" \
"syscfg\ti_board_open_close.c" 

C_SRCS__QUOTED += \
"../ADC_testbuf.c" \
"../dpc.c" \
"./syscfg/ti_dpl_config.c" \
"./syscfg/ti_drivers_config.c" \
"./syscfg/ti_drivers_open_close.c" \
"./syscfg/ti_pinmux_config.c" \
"./syscfg/ti_power_clock_config.c" \
"./syscfg/ti_board_config.c" \
"./syscfg/ti_board_open_close.c" \
"../factory_cal.c" \
"../interrupts.c" \
"../main.c" \
"../mmw_cli.c" \
"../mmw_demo_utils.c" \
"../mmw_flash_cal.c" \
"../mmwave_control_config.c" \
"../monitors.c" \
"../motion_detect.c" \
"../power_management.c" \
"../range_phase_bias_measurement.c" \
"../tracker_utils.c" 

SYSCFG_SRCS__QUOTED += \
"../example.syscfg" 


