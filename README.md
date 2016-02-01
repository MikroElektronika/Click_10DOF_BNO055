> ![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)
> #[10 DOF Click](http://www.mikroe.com/click/10dof/)#
> ##By [MikroElektronika](http://www.mikroe.com)
---

## Installation
Use the [package manager](http://www.mikroe.com/package-manager/) to install the library for your architecture.

###Example on STM32
```

#include "charger.h"
#include "bmp180.h"
#include "bno055.h"
#include <stdint.h>
#include "DOF10_objects.h"

struct bmp180_t bmp180;
struct bno055_t bno055;
uint8_t temp = 3;

float accel_x;
float accel_y;
float accel_z;
float gyro_x;
float gyro_y;
float gyro_z;
float mag_x;
float mag_y;
float mag_z;
int int_gyro_x;
int int_gyro_y;
int int_gyro_z;
uint8_t range;
char buffer[1];
char txt[10];

int16_t euler_h;
int16_t euler_p;
int16_t euler_r;


int32_t temperature_int;
float temperature_float;
int32_t pressure;

void system_init()
{
   I2C1_Init_Advanced(400000, &_GPIO_MODULE_I2C1_PB67 );
   Delay_ms(200);
   bmp180.dev_addr = BMP180_I2C_ADDR;
   bno055.dev_addr = BNO055_I2C_ADDR1;

   bmp180_init( &bmp180 );
   temp =  bno055_init( &bno055 );

   GPIO_Digital_Input(&GPIOA_IDR, _GPIO_PINMASK_0);         // Set PA0 as digital input

}

process_pressure()
{
  uint32_t temp_press;
  temp_press = bmp180_get_uncomp_pressure();
  pressure = bmp180_get_pressure(temp_press);
}

process_temperature()
{
    uint16_t temp;
    temp = bmp180_get_uncomp_temperature();
    temperature_int = bmp180_get_temperature(temp);
    temperature_float = (float) temperature_int;
    temperature_float *= 0.1f;
    
}

process_magnet()
{
     bno055_convert_float_mag_x_uT(&mag_x);
     bno055_convert_float_mag_y_uT(&mag_y);
     bno055_convert_float_mag_z_uT(&mag_z);
}

void process_accel()
{
  bno055_convert_float_accel_x_mg( &accel_x );
  bno055_convert_float_accel_y_mg( &accel_y );
  bno055_convert_float_accel_z_mg( &accel_z );
}

void process_gyro()
{
  bno055_read_euler_h(&euler_h);
  bno055_read_euler_p(&euler_p);
  bno055_read_euler_r(&euler_r);  
}



```

```
void main()
{
    system_init();
    bno055_set_operation_mode(0x0C);
    bno055_set_accel_range(0x01);
    bno055_get_accel_range(&range);
    bno055_get_operation_mode(&buffer);
    bno055_get_accel_unit(&temp);
    bno055_set_gyro_range(0x04);
    bno055_get_gyro_range(&range);
    
    while( 1 )
    {
          
        process_gyro();
        process_accel();
        process_temperature();
        process_pressure();
        process_magnet();
    }
       
      delay_ms(250);
    }
}


```








