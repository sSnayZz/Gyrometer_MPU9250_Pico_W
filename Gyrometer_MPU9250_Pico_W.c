#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// CONFIGURATION

#define I2C_PORT i2c0
#define SDA_PIN 4
#define SCL_PIN 5

#define MPU_ADDR 0x68

#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CFG   0x1C
#define REG_GYRO_CFG    0x1B
#define REG_CONFIG      0x1A
#define REG_ACCEL_XOUT  0x3B

// Sensibilités
#define ACCEL_LSB_PER_G 16384.0f   // ±2g
#define GYRO_LSB_PER_DPS 131.0f    // ±250 dps
#define G_SI 9.80665f              // m/s²

// CALIBRATION 

int32_t ax_off = 0, ay_off = 0, az_off = 0;
int32_t gx_off = 0, gy_off = 0, gz_off = 0;



void mpu_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, MPU_ADDR, buf, 2, false);
}

void mpu_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t reg = REG_ACCEL_XOUT;
    uint8_t buf[14];

    i2c_write_blocking(I2C_PORT, MPU_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDR, buf, 14, false);

    *ax = (buf[0] << 8) | buf[1];
    *ay = (buf[2] << 8) | buf[3];
    *az = (buf[4] << 8) | buf[5];
    *temp = (buf[6] << 8) | buf[7];
    *gx = (buf[8] << 8) | buf[9];
    *gy = (buf[10] << 8) | buf[11];
    *gz = (buf[12] << 8) | buf[13];
}

// INITIALISATION MPU 

void mpu_init() {
    mpu_write(REG_PWR_MGMT_1, 0x00);   // Wake up
    sleep_ms(100);

    mpu_write(REG_ACCEL_CFG, 0x00);    // Accel ±2g
    mpu_write(REG_GYRO_CFG,  0x00);    // Gyro ±250 dps
    mpu_write(REG_CONFIG,    0x02);    // DLPF ~94 Hz
}

// CALIBRATION AU DEBUT

void mpu_calibrate(int samples) {
    int64_t ax_sum=0, ay_sum=0, az_sum=0;
    int64_t gx_sum=0, gy_sum=0, gz_sum=0;

    printf("Calibration en cours (%d échantillons)...\n", samples);
    sleep_ms(500);

    for(int i=0;i<samples;i++) {
        int16_t ax,ay,az,temp,gx,gy,gz;
        mpu_read_raw(&ax,&ay,&az,&temp,&gx,&gy,&gz);

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;

        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        sleep_ms(5);
    }

    ax_off = ax_sum / samples;
    ay_off = ay_sum / samples;
    az_off = (az_sum / samples) - (int32_t)ACCEL_LSB_PER_G; // enlever la gravité

    gx_off = gx_sum / samples;
    gy_off = gy_sum / samples;
    gz_off = gz_sum / samples;

    printf("Offsets accel : %ld %ld %ld\n", ax_off, ay_off, az_off);
    printf("Offsets gyro  : %ld %ld %ld\n", gx_off, gy_off, gz_off);
}

// MAIN 

int main() {
    stdio_init_all();
    sleep_ms(2000);

    /* I2C */
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);


    mpu_init();
    mpu_calibrate(500);

    while (1) {
        int16_t axr, ayr, azr, temp, gxr, gyr, gzr;
        mpu_read_raw(&axr,&ayr,&azr,&temp,&gxr,&gyr,&gzr);

        // ACCEL 
        float ax_g = (axr - ax_off) / ACCEL_LSB_PER_G;
        float ay_g = (ayr - ay_off) / ACCEL_LSB_PER_G;
        float az_g = (azr - az_off) / ACCEL_LSB_PER_G;

        float ax_ms2 = ax_g * G_SI;
        float ay_ms2 = ay_g * G_SI;
        float az_ms2 = az_g * G_SI;

        //GYRO 
        float gx_dps = (gxr - gx_off) / GYRO_LSB_PER_DPS;
        float gy_dps = (gyr - gy_off) / GYRO_LSB_PER_DPS;
        float gz_dps = (gzr - gz_off) / GYRO_LSB_PER_DPS;

        float gx_rads = gx_dps * (M_PI / 180.0f);
        float gy_rads = gy_dps * (M_PI / 180.0f);
        float gz_rads = gz_dps * (M_PI / 180.0f);

        //TEMP 
        float temp_c = (temp / 340.0f) + 36.53f;

        printf(
            "ACC[m/s²]: %.2f %.2f %.2f | "
            "GYRO[rad/s]: %.2f %.2f %.2f | T=%.1f\n",
            ax_ms2, ay_ms2, az_ms2,
            gx_rads, gy_rads, gz_rads,
            temp_c
        );

       
        sleep_ms(50);
    }
}
