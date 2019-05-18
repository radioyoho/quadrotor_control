################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../interfaces/register_io_i2c.c \
../interfaces/register_io_spi.c \
../interfaces/sensor_io_i2c.c \
../interfaces/sensor_io_spi.c 

OBJS += \
./interfaces/register_io_i2c.o \
./interfaces/register_io_spi.o \
./interfaces/sensor_io_i2c.o \
./interfaces/sensor_io_spi.o 

C_DEPS += \
./interfaces/register_io_i2c.d \
./interfaces/register_io_spi.d \
./interfaces/sensor_io_i2c.d \
./interfaces/sensor_io_spi.d 


# Each subdirectory must supply rules for building sources it contributes
interfaces/%.o: ../interfaces/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_MK64FN1M0VDC12 -DCPU_MK64FN1M0VDC12_cm4 -DCPU_MK64FN1M0VLL12 -DFRDM_K64F -DFREEDOM -DPRINTF_ADVANCED_ENABLE -DSDK_DEBUGCONSOLE=0 -DPRINTF_FLOAT_ENABLE=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__REDLIB__ -I../board -I../source -I../ -I../drivers -I../device -I../CMSIS -I../CMSIS_driver -I../utilities -I../gpio_driver -I../interfaces -I../sensors -I../component/serial_manager -I../component/uart -I../component/lists -O0 -fno-common -g3 -Wall -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


