################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/TMX_ENVSET/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"E:/DIANSAI resources/26DIANSAI/DIANSAI_26" -I"E:/DIANSAI resources/26DIANSAI/DIANSAI_26/Debug" -I"D:/TMX_ENVSET/mspm0_sdk_2_11_00_07/source/third_party/CMSIS/Core/Include" -I"D:/TMX_ENVSET/mspm0_sdk_2_11_00_07/source" -g -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1595323523: ../empty.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/TI/sysconfig_1.26.2/sysconfig_cli.bat" -s "D:/TMX_ENVSET/mspm0_sdk_2_11_00_07/.metadata/product.json" --script "E:/DIANSAI resources/26DIANSAI/DIANSAI_26/empty.syscfg" -o "." --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-1595323523 ../empty.syscfg
device.opt: build-1595323523
device.cmd.genlibs: build-1595323523
ti_msp_dl_config.c: build-1595323523
ti_msp_dl_config.h: build-1595323523
Event.dot: build-1595323523

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/TMX_ENVSET/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"E:/DIANSAI resources/26DIANSAI/DIANSAI_26" -I"E:/DIANSAI resources/26DIANSAI/DIANSAI_26/Debug" -I"D:/TMX_ENVSET/mspm0_sdk_2_11_00_07/source/third_party/CMSIS/Core/Include" -I"D:/TMX_ENVSET/mspm0_sdk_2_11_00_07/source" -g -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: D:/TMX_ENVSET/mspm0_sdk_2_11_00_07/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/TMX_ENVSET/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"E:/DIANSAI resources/26DIANSAI/DIANSAI_26" -I"E:/DIANSAI resources/26DIANSAI/DIANSAI_26/Debug" -I"D:/TMX_ENVSET/mspm0_sdk_2_11_00_07/source/third_party/CMSIS/Core/Include" -I"D:/TMX_ENVSET/mspm0_sdk_2_11_00_07/source" -g -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


