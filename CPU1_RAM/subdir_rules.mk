################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1010/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --include_path="C:/ME461jl/ME461/workspace/finalproject" --include_path="C:/ME461jl/ME461/C2000Ware_3_02_00_00_F28379D/device_support/f2837xd/common/include" --include_path="C:/ME461jl/ME461/C2000Ware_3_02_00_00_F28379D/device_support/f2837xd/headers/include" --include_path="C:/ME461jl/ME461/workspace/finalproject/device" --include_path="C:/ME461jl/ME461/C2000Ware_3_02_00_00_F28379D/driverlib/f2837xd/driverlib" --include_path="C:/ME461jl/ME461/C2000Ware_3_02_00_00_F28379D/libraries/dsp/FPU/c28/include" --include_path="C:/ti/ccs1010/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/include" --define=DEBUG --define=_DUAL_HEADERS --define=CPU1 --define=_LAUNCHXL_F28379D --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1010/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --include_path="C:/ME461jl/ME461/workspace/finalproject" --include_path="C:/ME461jl/ME461/C2000Ware_3_02_00_00_F28379D/device_support/f2837xd/common/include" --include_path="C:/ME461jl/ME461/C2000Ware_3_02_00_00_F28379D/device_support/f2837xd/headers/include" --include_path="C:/ME461jl/ME461/workspace/finalproject/device" --include_path="C:/ME461jl/ME461/C2000Ware_3_02_00_00_F28379D/driverlib/f2837xd/driverlib" --include_path="C:/ME461jl/ME461/C2000Ware_3_02_00_00_F28379D/libraries/dsp/FPU/c28/include" --include_path="C:/ti/ccs1010/ccs/tools/compiler/ti-cgt-c2000_20.2.2.LTS/include" --define=DEBUG --define=_DUAL_HEADERS --define=CPU1 --define=_LAUNCHXL_F28379D --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


