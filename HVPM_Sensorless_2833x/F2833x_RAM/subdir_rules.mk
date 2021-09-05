################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
DLOG4CHC.obj: ../DLOG4CHC.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2833x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" -g --diag_warning=225 --quiet --preproc_with_compile --preproc_dependency="DLOG4CHC.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2833x_ADC_cal.obj: D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/source/DSP2833x_ADC_cal.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2833x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" -g --diag_warning=225 --quiet --preproc_with_compile --preproc_dependency="DSP2833x_ADC_cal.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2833x_CodeStartBranch.obj: D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/source/DSP2833x_CodeStartBranch.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2833x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" -g --diag_warning=225 --quiet --preproc_with_compile --preproc_dependency="DSP2833x_CodeStartBranch.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2833x_GlobalVariableDefs.obj: D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/source/DSP2833x_GlobalVariableDefs.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2833x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" -g --diag_warning=225 --quiet --preproc_with_compile --preproc_dependency="DSP2833x_GlobalVariableDefs.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2833x_usDelay.obj: ../DSP2833x_usDelay.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2833x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" -g --diag_warning=225 --quiet --preproc_with_compile --preproc_dependency="DSP2833x_usDelay.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

HVPM_Sensorless-DevInit_F2833x.obj: ../HVPM_Sensorless-DevInit_F2833x.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2833x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" -g --diag_warning=225 --quiet --preproc_with_compile --preproc_dependency="HVPM_Sensorless-DevInit_F2833x.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

HVPM_Sensorless.obj: ../HVPM_Sensorless.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 --include_path="D:/ti/ccsv7/tools/compiler/ti-cgt-c2000_16.9.4.LTS/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2833x/v132/DSP2833x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2833x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2833x_v2.0" --define="_DEBUG" --define="LARGE_MODEL" --define="FLOATING_MATH" -g --diag_warning=225 --quiet --preproc_with_compile --preproc_dependency="HVPM_Sensorless.d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


