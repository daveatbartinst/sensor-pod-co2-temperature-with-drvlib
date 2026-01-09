################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
driverlib/MSP430FR5xx_6xx/%.obj: ../driverlib/MSP430FR5xx_6xx/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: MSP430 Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-msp430_21.6.1.LTS/bin/cl430" -vmspx --data_model=restricted -O3 --use_hw_mpy=F5 --include_path="C:/Users/Dave/workspace_v12/sensor-pod-co2-temperature-with-drvlib" --include_path="C:/Users/Dave/workspace_v12/sensor-pod-co2-temperature-with-drvlib/driverlib" --include_path="C:/Users/Dave/workspace_v12/sensor-pod-co2-temperature-with-drvlib/driverlib/MSP430FR5xx_6xx" --include_path="C:/Users/Dave/workspace_v12/sensor-pod-co2-temperature-with-drvlib/driverlib/MSP430FR5xx_6xx/deprecated" --include_path="C:/Users/Dave/workspace_v12/sensor-pod-co2-temperature-with-drvlib/GrLib/grlib" --include_path="C:/Users/Dave/workspace_v12/sensor-pod-co2-temperature-with-drvlib/GrLib/fonts" --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-msp430_21.6.1.LTS/include" --advice:power="none" --advice:hw_config=all --define=__MSP430FR5964__ --define=_MPU_ENABLE -g --gcc --printf_support=minimal --diag_warning=225 --diag_wrap=off --display_error_number --large_memory_model --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU40 --preproc_with_compile --preproc_dependency="driverlib/MSP430FR5xx_6xx/$(basename $(<F)).d_raw" --obj_directory="driverlib/MSP430FR5xx_6xx" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


