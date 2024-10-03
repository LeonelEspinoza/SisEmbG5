#include <float.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <inttypes.h>


#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_sleep.h"

#define CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))

#define BUF_SIZE (128) // buffer size
#define TXD_PIN 1  // UART TX pin
#define RXD_PIN 3  // UART RX pin
#define UART_NUM UART_NUM_0   // UART port number
#define BAUD_RATE 115200   // Baud rate
#define M_PI 3.14159265358979323846

//especificas de bme688.c
#define I2C_MASTER_SCL_IO GPIO_NUM_22  // GPIO pin
#define I2C_MASTER_SDA_IO GPIO_NUM_21  // GPIO pin
#define I2C_MASTER_FREQ_HZ 10000
#define BME_ESP_SLAVE_ADDR 0x76
#define WRITE_BIT 0x0
#define READ_BIT 0x1
#define ACK_CHECK_EN 0x0
#define EXAMPLE_I2C_ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

#define REDIRECT_LOGS 1 // if redirect ESP log to another UART

esp_err_t ret = ESP_OK;
esp_err_t ret2 = ESP_OK;

uint16_t val0[6];

float task_delay_ms = 1000;

int wSize_default = 3;

int wSize = 3;

int32_t NVS_wsize = 0;

int t_fine = 0;

esp_err_t sensor_init(void) {
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;  // 0
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

esp_err_t bme_i2c_read(i2c_port_t i2c_num, uint8_t *data_addres, uint8_t *data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t bme_i2c_write(i2c_port_t i2c_num, uint8_t *data_addres, uint8_t *data_wr, size_t size) {
    uint8_t size1 = 1;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME_ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_addres, size1, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ------------ BME 688 ------------- //
uint8_t calc_gas_wait(uint16_t dur) {
    // Fuente: BME688 API
    // https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c#L1176
    uint8_t factor = 0;
    uint8_t durval;

    if (dur >= 0xfc0) {
        durval = 0xff; /* Max duration*/
    } else {
        while (dur > 0x3F) {
            dur = dur >> 2;
            factor += 1;
        }

        durval = (uint8_t)(dur + (factor * 64));
    }

    return durval;
}

uint8_t calc_res_heat(uint16_t temp) {
    // Fuente: BME688 API
    // https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c#L1145
    uint8_t heatr_res;
    uint8_t amb_temp = 25;

    uint8_t reg_par_g1 = 0xED;
    uint8_t par_g1;
    bme_i2c_read(I2C_NUM_0, &reg_par_g1, &par_g1, 1);

    uint8_t reg_par_g2_lsb = 0xEB;
    uint8_t par_g2_lsb;
    bme_i2c_read(I2C_NUM_0, &reg_par_g2_lsb, &par_g2_lsb, 1);
    uint8_t reg_par_g2_msb = 0xEC;
    uint8_t par_g2_msb;
    bme_i2c_read(I2C_NUM_0, &reg_par_g2_msb, &par_g2_msb, 1);
    uint16_t par_g2 = (int16_t)(CONCAT_BYTES(par_g2_msb, par_g2_lsb));

    uint8_t reg_par_g3 = 0xEE;
    uint8_t par_g3;
    bme_i2c_read(I2C_NUM_0, &reg_par_g3, &par_g3, 1);

    uint8_t reg_res_heat_range = 0x02;
    uint8_t res_heat_range;
    uint8_t mask_res_heat_range = (0x3 << 4);
    uint8_t tmp_res_heat_range;

    uint8_t reg_res_heat_val = 0x00;
    uint8_t res_heat_val;

    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t heatr_res_x100;

    if (temp > 400) {
        temp = 400;
    }

    bme_i2c_read(I2C_NUM_0, &reg_res_heat_range, &tmp_res_heat_range, 1);
    bme_i2c_read(I2C_NUM_0, &reg_res_heat_val, &res_heat_val, 1);
    res_heat_range = (mask_res_heat_range & tmp_res_heat_range) >> 4;

    var1 = (((int32_t)amb_temp * par_g3) / 1000) * 256;
    var2 = (par_g1 + 784) * (((((par_g2 + 154009) * temp * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 / 2);
    var4 = (var3 / (res_heat_range + 4));
    var5 = (131 * res_heat_val) + 65536;
    heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);
    heatr_res = (uint8_t)((heatr_res_x100 + 50) / 100);

    return heatr_res;
}

int bme_get_chipid(void) {
    uint8_t reg_id = 0xd0;
    uint8_t tmp;

    bme_i2c_read(I2C_NUM_0, &reg_id, &tmp, 1);
    //printf("Valor de CHIPID: %2X \n\n", tmp);

    if (tmp == 0x61) {
        //printf("Chip BME688 reconocido.\n\n");
        return 0;
    } else {
        //printf("Chip BME688 no reconocido. \nCHIP ID: %2x\n\n", tmp);  // %2X
    }

    return 1;
}

int bme_softreset(void) {
    uint8_t reg_softreset = 0xE0, val_softreset = 0xB6;

    ret = bme_i2c_write(I2C_NUM_0, &reg_softreset, &val_softreset, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        printf("\nError en softreset: %s \n", esp_err_to_name(ret));
        return 1;
    //} else {
        //printf("\nSoftreset: OK\n\n");
    }
    return 0;
}

void bme_forced_mode(void) {
    /*
    Fuente: Datasheet[19]
    https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=19

    Para configurar el BME688 en forced mode los pasos son:

    1. Set humidity oversampling to 1x     |-| 0b001 to osrs_h<2:0>
    2. Set temperature oversampling to 2x  |-| 0b010 to osrs_t<2:0>
    3. Set pressure oversampling to 16x    |-| 0b101 to osrs_p<2:0>

    4. Set gas duration to 100 ms          |-| 0x59 to gas_wait_0
    5. Set heater step size to 0           |-| 0x00 to res_heat_0
    6. Set number of conversion to 0       |-| 0b0000 to nb_conv<3:0> and enable gas measurements
    7. Set run_gas to 1                    |-| 0b1 to run_gas<5>

    8. Set operation mode                  |-| 0b01  to mode<1:0>

    */

    // Datasheet[33]
    uint8_t ctrl_hum = 0x72;
    uint8_t ctrl_meas = 0x74;
    uint8_t gas_wait_0 = 0x64;
    uint8_t res_heat_0 = 0x5A;
    uint8_t ctrl_gas_1 = 0x71;

    uint8_t mask;
    uint8_t prev;
    // Configuramos el oversampling (Datasheet[36])

    // 1. osrs_h esta en ctrl_hum (LSB) -> seteamos 001 en bits 2:0
    uint8_t osrs_h = 0b001;
    mask = 0b00000111;
    bme_i2c_read(I2C_NUM_0, &ctrl_hum, &prev, 1);
    osrs_h = (prev & ~mask) | osrs_h;

    // 2. osrs_t esta en ctrl_meas MSB -> seteamos 010 en bits 7:5
    uint8_t osrs_t = 0b01000000;
    // 3. osrs_p esta en ctrl_meas LSB -> seteamos 101 en bits 4:2 [Datasheet:37]
    uint8_t osrs_p = 0b00010100;
    uint8_t osrs_t_p = osrs_t | osrs_p;
    // Se recomienda escribir hum, temp y pres en un solo write

    // Configuramos el sensor de gas

    // 4. Seteamos gas_wait_0 a 100ms
    uint8_t gas_duration = calc_gas_wait(100);

    // 5. Seteamos res_heat_0
    uint8_t heater_step = calc_res_heat(300);

    // 6. nb_conv esta en ctrl_gas_1 -> seteamos bits 3:0
    uint8_t nb_conv = 0b00000000;
    // 7. run_gas esta en ctrl_gas_1 -> seteamos bit 5
    uint8_t run_gas = 0b00100000;
    uint8_t gas_conf = nb_conv | run_gas;

    bme_i2c_write(I2C_NUM_0, &gas_wait_0, &gas_duration, 1);
    bme_i2c_write(I2C_NUM_0, &res_heat_0, &heater_step, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_hum, &osrs_h, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_meas, &osrs_t_p, 1);
    bme_i2c_write(I2C_NUM_0, &ctrl_gas_1, &gas_conf, 1);

    // Seteamos el modo
    // 8. Seteamos el modo a 01, pasando primero por sleep
    uint8_t mode = 0b00000001;
    uint8_t tmp_pow_mode;
    uint8_t pow_mode = 0;

    do {
        ret = bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);

        if (ret == ESP_OK) {
            // Se pone en sleep
            pow_mode = (tmp_pow_mode & 0x03);
            if (pow_mode != 0) {
                tmp_pow_mode &= ~0x03;
                ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);
            }
        }
    } while ((pow_mode != 0x0) && (ret == ESP_OK));

    tmp_pow_mode = (tmp_pow_mode & ~0x03) | mode;
    ret = bme_i2c_write(I2C_NUM_0, &ctrl_meas, &tmp_pow_mode, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

int bme_check_forced_mode(void) {
    uint8_t ctrl_hum = 0x72;
    uint8_t ctrl_meas = 0x74;
    uint8_t gas_wait_0 = 0x64;
    uint8_t res_heat_0 = 0x5A;
    uint8_t ctrl_gas_1 = 0x71;

    uint8_t tmp, tmp2, tmp3, tmp4, tmp5;

    ret = bme_i2c_read(I2C_NUM_0, &ctrl_hum, &tmp, 1);
    ret = bme_i2c_read(I2C_NUM_0, &gas_wait_0, &tmp2, 1);
    ret = bme_i2c_read(I2C_NUM_0, &res_heat_0, &tmp3, 1);
    ret = bme_i2c_read(I2C_NUM_0, &ctrl_gas_1, &tmp4, 1);
    ret = bme_i2c_read(I2C_NUM_0, &ctrl_meas, &tmp5, 1);
    vTaskDelay(task_delay_ms / portTICK_PERIOD_MS);
    return (tmp == 0b001 && tmp2 == 0x59 && tmp3 == 0x00 && tmp4 == 0b100000 && tmp5 == 0b01010101);
}

int bme_temp_celsius(uint32_t temp_adc) {
    // Datasheet[23]
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=23

    // Se obtienen los parametros de calibracion
    uint8_t addr_par_t1_lsb = 0xE9, addr_par_t1_msb = 0xEA;
    uint8_t addr_par_t2_lsb = 0x8A, addr_par_t2_msb = 0x8B;
    uint8_t addr_par_t3_lsb = 0x8C;
    uint16_t par_t1;
    uint16_t par_t2;
    uint16_t par_t3;

    uint8_t par[5];
    bme_i2c_read(I2C_NUM_0, &addr_par_t1_lsb, par, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t1_msb, par + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t2_lsb, par + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t2_msb, par + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_t3_lsb, par + 4, 1);

    par_t1 = (par[1] << 8) | par[0];
    par_t2 = (par[3] << 8) | par[2];
    par_t3 = par[4];
    
    
    int64_t var1;
    int64_t var2;
    int64_t var3;
    //int t_fine;
    int calc_temp;

    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
    var2 = (var1 * (int32_t)par_t2) >> 11;
    var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    var3 = ((var3) * ((int32_t)par_t3 << 4)) >> 14;
    t_fine = (int32_t)(var2 + var3);
    calc_temp = (((t_fine * 5) + 128) >> 8);
    return calc_temp;
}

int bme_pressure_pascal(uint32_t press_adc){
    uint8_t addr_par_p1_lsb = 0x8E, addr_par_p1_msb = 0x8F;
    uint8_t addr_par_p2_lsb = 0x90, addr_par_p2_msb = 0x91;
    uint8_t addr_par_p3_lsb = 0x92;
    uint8_t addr_par_p4_lsb = 0x94, addr_par_p4_msb = 0x95;
    uint8_t addr_par_p5_lsb = 0x96, addr_par_p5_msb = 0x97;
    uint8_t addr_par_p6_lsb = 0x99;
    uint8_t addr_par_p7_lsb = 0x98;
    uint8_t addr_par_p8_lsb = 0x9C, addr_par_p8_msb = 0x9D;
    uint8_t addr_par_p9_lsb = 0x9E, addr_par_p9_msb = 0x9F;
    uint8_t addr_par_p10_lsb = 0xA0;
    uint16_t par_p1;
    uint16_t par_p2;
    uint16_t par_p3;
    uint16_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    uint16_t par_p7;
    uint16_t par_p8;
    uint16_t par_p9;
    uint16_t par_p10;

    uint8_t par[16];
    bme_i2c_read(I2C_NUM_0, &addr_par_p1_lsb, par, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p1_msb, par + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p2_lsb, par + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p2_msb, par + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p3_lsb, par + 4, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p4_lsb, par + 5, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p4_msb, par + 6, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p5_lsb, par + 7, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p5_msb, par + 8, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p6_lsb, par + 9, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p7_lsb, par + 10, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p8_lsb, par + 11, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p8_msb, par + 12, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p9_lsb, par + 13, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p9_msb, par + 14, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_p10_lsb, par + 15, 1);
    
    //par_px = (par[y+1] << 8) | par [y]
    //par_px = par[y]
    
    par_p1 = (par[1] << 8) | par[0];
    par_p2 = (par[3] << 8) | par[2];
    par_p3 = (par[4]);
    par_p4 = (par[6] << 8) | par[5];
    par_p5 = (par[8] << 8) | par[7];
    par_p6 = par[9];
    par_p7 = par[10];
    par_p8 = (par[12] << 8) | par[11];
    par_p9 = (par[14] << 8) | par[13];
    par_p10 = par[15];

    int64_t var1;
    int64_t var2;
    int64_t var3;
    int press_comp;

    // t_fine = 0 por ahora original -> var1 = ((int32_t)t_fine >> 1) - 64000;
    var1 = ((int32_t)0 >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t) par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)par_p5) << 1);
    var2 = (var2 >> 2) +  ((int32_t)par_p4 << 16);
    var1 = ((((( var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)par_p3 << 5)) >> 3) + (((int32_t) par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)par_p1) >> 15;
    press_comp = 1048576 - press_adc;
    press_comp = (uint32_t)((press_comp - (var2 >> 12)) * ((uint32_t)3125));

    if (press_comp >= (1 << 30)){
        press_comp = ((press_comp / (uint32_t)var1) << 1);
    } else {
        press_comp = ((press_comp << 1) / (uint32_t)var1);
    }

    var1 = ((int32_t)par_p9 * (int32_t)(((press_comp >> 3) * (press_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(press_comp >> 2) * (int32_t)par_p8) >> 13;
    var3 = ((int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) * (int32_t)par_p10) >> 17;
    press_comp = (int32_t)(press_comp) + ((var1 + var2 + var3 + ((int32_t)par_p7 << 7)) >> 4);

    return press_comp;
}

int bme_humidity_percentage(uint32_t hum_adc, float temp_comp){
    //76543210
    //<3:0> -> 3210
    //<7:4> -> 7654
    uint8_t addr_par_h1_lsb = 0xE2, addr_par_h1_msb = 0xE3;
    addr_par_h1_lsb = addr_par_h1_lsb & 0b00001111;
    
    uint8_t addr_par_h2_lsb = 0xE2, addr_par_h2_msb = 0xE1;
    addr_par_h2_lsb = addr_par_h2_lsb >> 4;
    
    uint8_t addr_par_h3_lsb = 0xE4;
    uint8_t addr_par_h4_lsb = 0xE5;
    uint8_t addr_par_h5_lsb = 0xE6;
    uint8_t addr_par_h6_lsb = 0xE7;
    uint8_t addr_par_h7_lsb = 0xE8;
    uint16_t par_h1;
    uint16_t par_h2;
    uint16_t par_h3;
    uint16_t par_h4;
    uint16_t par_h5;
    uint16_t par_h6;
    uint16_t par_h7;

    uint8_t par[9];
    bme_i2c_read(I2C_NUM_0, &addr_par_h1_lsb, par, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h1_msb, par + 1, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h2_lsb, par + 2, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h2_msb, par + 3, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h3_lsb, par + 4, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h4_lsb, par + 5, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h5_lsb, par + 6, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h6_lsb, par + 7, 1);
    bme_i2c_read(I2C_NUM_0, &addr_par_h7_lsb, par + 8, 1);

    par_h1 = (par[1] << 8) | par[0];
    par_h2 = (par[3] << 8) | par[2];
    par_h3 = par[4];
    par_h4 = par[5];
    par_h5 = par[6];
    par_h6 = par[7];
    par_h7 = par[8];

    int64_t var1;
    int64_t var2;
    int64_t var3;
    int64_t var4;
    int64_t var5;
    int64_t var6;
    int64_t temp_scaled;

    int hum_comp;

    temp_scaled = (int32_t)temp_comp;
    var1 = (int32_t)hum_adc - (int32_t)((int32_t)par_h1 << 4) - (((temp_scaled * (int32_t)par_h3)/((int32_t)100))>>1);
    var2 = ((int32_t)par_h2 * (((temp_scaled *
    (int32_t)par_h4) / ((int32_t)100)) +
    (((temp_scaled * ((temp_scaled * (int32_t)par_h5) /
    ((int32_t)100))) >> 6) / ((int32_t)100)) + ((int32_t)(1<<14)))) >> 10;
    var3 = var1 * var2;
    var4 = (((int32_t)par_h6 << 7) + 
    ((temp_scaled * (int32_t)par_h7) / ((int32_t)100))) >> 4;
    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;
    hum_comp = (var3 + var6) >> 12;
    hum_comp = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;

    return hum_comp;
}


void bme_get_mode(void) {
    uint8_t reg_mode = 0x74;
    uint8_t tmp;

    ret = bme_i2c_read(I2C_NUM_0, &reg_mode, &tmp, 1);

    tmp = tmp & 0x3;

    //printf("Valor de BME MODE: %2X \n\n", tmp);
}

float bme_read_data_temperature(void) {
    // Datasheet[23:41]
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=23

    uint8_t tmp;

    // Se obtienen los datos de temperatura
    uint8_t forced_temp_addr[] = {0x22, 0x23, 0x24};
    uint32_t temp_adc = 0;
    bme_forced_mode();
    // Datasheet[41]
    // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf#page=41

    bme_i2c_read(I2C_NUM_0, &forced_temp_addr[0], &tmp, 1);
    temp_adc = temp_adc | tmp << 12;
    bme_i2c_read(I2C_NUM_0, &forced_temp_addr[1], &tmp, 1);
    temp_adc = temp_adc | tmp << 4;
    bme_i2c_read(I2C_NUM_0, &forced_temp_addr[2], &tmp, 1);
    temp_adc = temp_adc | (tmp & 0xf0) >> 4;

    uint32_t temp = bme_temp_celsius(temp_adc);

    float ret = (float)temp / 100;

    //printf("Temperatura: %f\n", ret);

    return ret;
}

float bme_read_data_pressure(void){
    uint8_t tmp;

    uint8_t forced_press_addr[] = {0x1F, 0x20, 0x21};
    uint32_t press_adc = 0;
    bme_forced_mode();

    bme_i2c_read(I2C_NUM_0, &forced_press_addr[0], &tmp, 1);
    press_adc = press_adc | tmp << 12;
    bme_i2c_read(I2C_NUM_0, &forced_press_addr[1], &tmp, 1);
    press_adc = press_adc | tmp << 4;
    bme_i2c_read(I2C_NUM_0, &forced_press_addr[2], &tmp, 1);
    press_adc = press_adc | (tmp & 0xf0) >> 4;

    uint32_t press = bme_pressure_pascal(press_adc);

    float ret = (float)press / 100;

    return ret;
}

float bme_read_data_humidity(float temp){

    uint8_t tmp;

    uint8_t forced_hum_addr[] = {0x25, 0x26};
    uint32_t hum_adc = 0;
    bme_forced_mode();

    bme_i2c_read(I2C_NUM_0, &forced_hum_addr[0], &tmp, 1);
    hum_adc = hum_adc | tmp;
    bme_i2c_read(I2C_NUM_0, &forced_hum_addr[1], &tmp, 1);
    hum_adc = hum_adc | tmp;

    uint32_t hum = bme_humidity_percentage(hum_adc,temp);

    return hum;
}
/*
 This internal API is used to read a single data of the sensor 

static int8_t read_field_data(uint8_t index, struct bme68x_data *data, struct bme68x_dev *dev)
{
    int8_t rslt = BME68X_OK;
    uint8_t buff[BME68X_LEN_FIELD] = { 0 };
    uint8_t gas_range_l, gas_range_h;
    uint32_t adc_temp;
    uint32_t adc_pres;
    uint16_t adc_hum;
    uint16_t adc_gas_res_low, adc_gas_res_high;
    uint8_t tries = 5;

    while ((tries) && (rslt == BME68X_OK))
    {
        rslt = bme68x_get_regs(((uint8_t)(BME68X_REG_FIELD0 + (index * BME68X_LEN_FIELD_OFFSET))),
                               buff,
                               (uint16_t)BME68X_LEN_FIELD,
                               dev);
        if (!data)
        {
            rslt = BME68X_E_NULL_PTR;
            break;
        }

        data->status = buff[0] & BME68X_NEW_DATA_MSK;
        data->gas_index = buff[0] & BME68X_GAS_INDEX_MSK;
        data->meas_index = buff[1];

         read the raw data from the sensor 

        adc_pres = (uint32_t)(((uint32_t)buff[2] * 4096) | ((uint32_t)buff[3] * 16) | ((uint32_t)buff[4] / 16));
        adc_temp = (uint32_t)(((uint32_t)buff[5] * 4096) | ((uint32_t)buff[6] * 16) | ((uint32_t)buff[7] / 16));
        adc_hum = (uint16_t)(((uint32_t)buff[8] * 256) | (uint32_t)buff[9]);
        adc_gas_res_low = (uint16_t)((uint32_t)buff[13] * 4 | (((uint32_t)buff[14]) / 64));
        adc_gas_res_high = (uint16_t)((uint32_t)buff[15] * 4 | (((uint32_t)buff[16]) / 64));
        gas_range_l = buff[14] & BME68X_GAS_RANGE_MSK;
        gas_range_h = buff[16] & BME68X_GAS_RANGE_MSK;
        if (dev->variant_id == BME68X_VARIANT_GAS_HIGH)
        {
            data->status |= buff[16] & BME68X_GASM_VALID_MSK;
            data->status |= buff[16] & BME68X_HEAT_STAB_MSK;
        }
        else
        {
            data->status |= buff[14] & BME68X_GASM_VALID_MSK;
            data->status |= buff[14] & BME68X_HEAT_STAB_MSK;
        }

        if ((data->status & BME68X_NEW_DATA_MSK) && (rslt == BME68X_OK))
        {
            rslt = bme68x_get_regs(BME68X_REG_RES_HEAT0 + data->gas_index, &data->res_heat, 1, dev);
            if (rslt == BME68X_OK)
            {
                rslt = bme68x_get_regs(BME68X_REG_IDAC_HEAT0 + data->gas_index, &data->idac, 1, dev);
            }

            if (rslt == BME68X_OK)
            {
                rslt = bme68x_get_regs(BME68X_REG_GAS_WAIT0 + data->gas_index, &data->gas_wait, 1, dev);
            }

            if (rslt == BME68X_OK)
            {
                data->temperature = calc_temperature(adc_temp, dev);
                data->pressure = calc_pressure(adc_pres, dev);
                data->humidity = calc_humidity(adc_hum, dev);
                if (dev->variant_id == BME68X_VARIANT_GAS_HIGH)
                {
                    data->gas_resistance = calc_gas_resistance_high(adc_gas_res_high, gas_range_h);
                }
                else
                {
                    data->gas_resistance = calc_gas_resistance_low(adc_gas_res_low, gas_range_l, dev);
                }

                break;
            }
        }

        if (rslt == BME68X_OK)
        {
            dev->delay_us(BME68X_PERIOD_POLL, dev->intf_ptr);
        }

        tries--;
    }

    return rslt;
}
*/

float bme_read_data_CO(void){
    return 1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//

// Function for sending things to UART1
static int uart1_printf(const char *str, va_list ap) {
    char *buf;
    vasprintf(&buf, str, ap);
    uart_write_bytes(UART_NUM_1, buf, strlen(buf));
    free(buf);
    return 0;
}

// Setup of UART connections 0 and 1, and try to redirect logs to UART1 if asked
static void uart_setup() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Redirect ESP log to UART1
    if (REDIRECT_LOGS) {
        esp_log_set_vprintf(uart1_printf);
    }
}

// Read UART_num for input with timeout of 1 sec
int serial_read(char *buffer, int size){
    int len = uart_read_bytes(UART_NUM, (uint8_t*)buffer, size, pdMS_TO_TICKS(1000));
    return len;
}

float RMS(float *data, int wsize, int begin){
    //printf("<RMS>\n");
    float sum = 0;
    for (int i = begin; i < begin + wsize; i++){
        float num = data[i];
        sum = sum + pow(num,2);
    }
    float sumN = sum / wsize;
    float rms = sqrt(sumN);
    return rms;
}

/**
 * @brief Funcion que calcula la FFT de un arreglo y guarda el resultado inplace
 *
 * @param array Arreglo de elementos sobre los que se quiere calcular la FFT
 * @param size Tamano del arreglo
 * @param array_re Direccion del arreglo donde se guardara la parte real. Debe ser de tamano size
 * @param array_im Direccion del arreglo donde se guardara la parte imaginaria. Debe ser de tamano size
 */
void FFT(float *array, int size, float *array_re, float *array_im) {
    for (int k = 0; k < size; k++) {
        float real = 0;
        float imag = 0;

        for (int n = 0; n < size; n++) {
            float angulo = 2 * M_PI * k * n / size;
            float cos_angulo = cos(angulo);
            float sin_angulo = -sin(angulo);

            real += array[n] * cos_angulo;
            imag += array[n] * sin_angulo;
        }
        real /= size;
        imag /= size;
        array_re[k] = real;
        array_im[k] = imag;
    }
}

int MyCompare(const void *arg1, const void *arg2){
    return arg1>arg2? -1 : 1; 
}

void Process_Data() {

    //Use wSize to set data size
    int DataSize = sizeof(float)*(wSize*4 + 4); //Temp, Press, Hum, CO Meassure Data + each RMS
    float * data = malloc(DataSize);
    
    //Read and save data; wSize times each
    for (int i = 0; i < wSize; i++){

        //Read and save temperature data
        float temp = bme_read_data_temperature();
        data[i] = temp;

        //Read and save pressure data
        float press = bme_read_data_pressure();
        data[wSize + i] = press;

        //Read and save humidity data
        float hum = bme_read_data_humidity(temp);
        data[2*wSize + i] = hum;

        //Read and save CO data
        float CO = bme_read_data_CO();
        data[3*wSize + i] = CO;
    }
    
    //Calculate RMS
    float temp_RMS  =   RMS(data, wSize, 0*wSize);
    float press_RMS =   RMS(data, wSize, 1*wSize);
    float hum_RMS   =   RMS(data, wSize, 2*wSize);
    float CO_RMS    =   RMS(data, wSize, 3*wSize);

    //Save RMS
    data[wSize*4+0] = temp_RMS;
    data[wSize*4+1] = press_RMS;
    data[wSize*4+2] = hum_RMS;
    data[wSize*4+3] = CO_RMS; 
    
    //Send data
    const char* dataToSend = (const char*)data;
    uart_write_bytes(UART_NUM, dataToSend, DataSize);

    //Calculate 5 Peaks
    qsort(data+wSize*0,wSize,sizeof(float), MyCompare);
    qsort(data+wSize*1,wSize,sizeof(float), MyCompare);
    qsort(data+wSize*2,wSize,sizeof(float), MyCompare);
    qsort(data+wSize*3,wSize,sizeof(float), MyCompare);

    //Save 5 Peaks
    float peaks[5*4];
    if(wSize>=5){
        for(int i=0; i<5; i++){
            peaks[i+5*0]=data[i+wSize*0];
            peaks[i+5*1]=data[i+wSize*1];
            peaks[i+5*2]=data[i+wSize*2];
            peaks[i+5*3]=data[i+wSize*3];
        }
    }

    //Send 5 Peaks
    dataToSend = (const char*)peaks;
    uart_write_bytes(UART_NUM, dataToSend, sizeof(float)*5*4);

    int FFT_size = sizeof(float)*(wSize*2*4);   //4*wSize meassures; FFT have imaginary and real numbers;  
    float * FFT_data = malloc(FFT_size);

    //Calculate FFTs
    FFT(data, wSize, FFT_data+wSize*0, FFT_data+wSize*1);
    FFT(data, wSize, FFT_data+wSize*2, FFT_data+wSize*3);
    FFT(data, wSize, FFT_data+wSize*4, FFT_data+wSize*5);
    FFT(data, wSize, FFT_data+wSize*6, FFT_data+wSize*7);

    //Send FFTs
    dataToSend = (const char*)FFT_data;
    uart_write_bytes(UART_NUM, dataToSend, FFT_size);

    free(data);
    free(FFT_data);

    return;
}

// Setea el tamano de la ventana y lo actualiza en la NVS
void Set_wSize() {
    char windowSize[5];
    esp_err_t err = nvs_flash_init();
    
    while(1){
        int rLen = serial_read(windowSize, 4);
        if (rLen > 0){
            windowSize[4]=0;
            wSize = atoi(windowSize);
            break;
        }
    }

    //initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    //Open
    nvs_handle_t my_handle;
    ret = nvs_open("storage", NVS_READWRITE, &my_handle);

    if (err != ESP_OK) {
        printf("<Set_wSize> Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }

    //Write
    NVS_wsize = wSize;
    err = nvs_set_i32(my_handle, "NVS_wsize", NVS_wsize);
    if (err != ESP_OK){
        printf("Failed!\n");
    }
    
    //Comit write value
    err = nvs_commit(my_handle);
    if (err != ESP_OK){
        printf("Failed!\n");
    }

    //close
    nvs_close(my_handle);
}

void ResetESP() {
    esp_deep_sleep(5000);
}

// Obtiene el tamano de la ventana que esta en la NVS
void setup_wsize() {
    //initialize NVS
    printf("<setup_wsize> Obteniendo tamano de la NVS\n");
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    //Open
    printf("<setup_wsize> Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    ret = nvs_open("storage", NVS_READWRITE, &my_handle);

    if (ret != ESP_OK) {
        printf("<setup_wsize> Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
        return;
    }

    //read NVS
    esp_err_t err = nvs_get_i32(my_handle, "NVS_wsize", &NVS_wsize);
    switch (err){
        case ESP_OK:
            printf("<setup_wsize> Done, NVS_wsize: %ld\n", NVS_wsize);
            wSize = NVS_wsize;
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("<setup_wsize> The value is not initialized yet, using default: %d\n", wSize_default);
            wSize = wSize_default;
            break;
        default:
            printf("<setup_wsize> Error (%s) reading!\n", esp_err_to_name(err));
    }

    //close
    nvs_close(my_handle);
    
}

// Main
void app_main(){
    uart_setup(); // Uart setup
    setup_wsize(); // Window size from nvs
    
    char* dataToSend = "OK setup";
    uart_write_bytes(UART_NUM, (const char*)dataToSend ,strlen(dataToSend));

    char dataResponse1[6];
    while (1){
        int rLen = serial_read(dataResponse1, 6);
        if (rLen > 0){
            if (strcmp(dataResponse1, "BEGIN") == 0){
                break;
            }
        }
    }

    char dataResponse2[3];
    while (1){
        int ar[1];
        ar[0]=wSize;
        uart_write_bytes(UART_NUM, (const char*)ar, sizeof(int));
        vTaskDelay(pdMS_TO_TICKS(500));  // Delay for 0.5 second

        int rLen = serial_read(dataResponse2, 3);
        if (rLen > 0){
            if (strcmp(dataResponse2, "OK") == 0){
                break;
            }
        }
    }

    //Conectamos esp con bme
    ESP_ERROR_CHECK(sensor_init());
    bme_get_chipid();
    bme_softreset();
    bme_get_mode();
    bme_forced_mode();

    char selection[2];
    while(1){
        int rLen = serial_read(selection, 2);
        if (rLen > 0){
            //1 -> Solicitar Ventana
            if (strcmp(selection, "1") == 0){
                Process_Data();
            }
            //2 -> Cambiar tamaño
            if (strcmp(selection, "2") == 0){
                Set_wSize();
            }
            //3 -> Terminar conexión
            if (strcmp(selection, "3") == 0){
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second
    }

    ResetESP();
}

