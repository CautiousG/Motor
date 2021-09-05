################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
f28335_SCI.obj: ../SCI/f28335_SCI.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -O2 --include_path="E:/CCS_Pro/HVPM_CCS_MPC_Speed/CCS_MPC" --include_path="E:/CCS_Pro/HVPM_CCS_MPC_Speed/SCI" --include_path="E:/CCS_Pro/HVPM_CCS_MPC_Speed" --include_path="E:/CCS_Pro/HVPM_CCS_MPC_Speed/SC16IS750" --include_path="D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="E:/CCS_Pro/HVPM_CCS_MPC_Speed/SPI" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2833x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --advice:performance=all --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" -g --diag_warning=225 --quiet --preproc_with_compile --preproc_dependency="SCI/f28335_SCI.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


