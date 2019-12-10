//
// Created by rarvolt on 15.10.2019.
//

#include <esp_log.h>
#include "as3935.h"
#include "driver/i2c.h"
#include "freertos/task.h"

#define AS3935_I2C_PORT I2C_NUM_0
#define WRITE_BIT       I2C_MASTER_WRITE
#define READ_BIT        I2C_MASTER_READ
#define ACK_CHECK_EN    0x1
#define ACK_CHECK_DIS   0x0
#define ACK_VAL         0x0
#define NACK_VAL        0x1
#define LAST_NACK_VAL   0x2

const char* _AS_TAG = "AS3935";

uint8_t _ffsz(uint8_t mask)
{
    uint8_t i = 0;
    if (mask)
    {
        for (i = 1; ~mask & 1; i++)
        {
            mask >>= 1;
        }
    }
    return i;
}


/**
 * @brief Read data from AS3935
 *
 * 1. send reg address
 * +-------+---------------------------+----------------------+------+
 * | start | slave_addr + wr_bit + ack | write reg_addr + ack | stop |
 * +-------+---------------------------+----------------------+------+
 *
 * 2. read data
 * +-------+---------------------------+---------------------------------+------+
 * | start | slave_addr + wr_bit + ack | read data byte + ack(last nack) | stop |
 * +-------+---------------------------+---------------------------------+------+
 *
 * @param as AS3935 driver handle
 * @param reg
 * @param mask
 * @return
 */
esp_err_t _as3935_raw_register_read(AS3935_handle_t as, uint8_t reg, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, as->addr << 1 | WRITE_BIT, ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(AS3935_I2C_PORT, cmd, 1000 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, as->addr << 1 | READ_BIT, ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data, LAST_NACK_VAL))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(AS3935_I2C_PORT, cmd, 1000 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

/**
 * @brief Send data to AS3935
 *
 * 1. send data
 * +-------+---------------------------+----------------------+-----------------------+------+
 * | start | slave_addr + wr_bit + ack | write reg_addr + ack | write data byte + ack | stop |
 * +-------+---------------------------+----------------------+-----------------------+------+
 *
 * @param as AS3935 driver handle
 * @param reg slave reg address
 * @param data data byte
 */
esp_err_t _as3935_raw_register_write(AS3935_handle_t as, uint8_t reg, uint8_t data)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, as->addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(AS3935_I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t _as3935_register_read(AS3935_handle_t as, uint8_t reg, uint8_t mask, uint8_t *data)
{
    esp_err_t ret;
    uint8_t regval;

    ret = _as3935_raw_register_read(as, reg, &regval);
    if (ret != ESP_OK)
    {
        ESP_LOGE(_AS_TAG, "ESP ERROR: %d", ret);
        return ret;
    }

    regval = regval & mask;
    if (mask)
    {
        regval >>= (_ffsz(mask) - 1);
    }

    *data = regval;

    return ret;
}

esp_err_t _as3935_register_write(AS3935_handle_t as, uint8_t reg, uint8_t mask, uint8_t data)
{
    uint8_t regval;
    esp_err_t ret = _as3935_raw_register_read(as, reg, &regval);
    if (ret != ESP_OK)
    {
        return ret;
    }

    regval &= ~(mask);
    if (mask)
    {
        regval |= (data << (_ffsz(mask) - 1));
    }
    else
    {
        regval |= data;
    }

    ret = _as3935_raw_register_write(as, reg, regval);
    return ret;
}

AS3935_handle_t as3935_init(uint8_t address, int8_t irq_pin)
{
    AS3935_handle_t as = malloc(sizeof(struct AS3935_t));
    as->addr = address;
    as->irq_pin = irq_pin;

    if (irq_pin >= 0)
    {
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_POSEDGE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << irq_pin);
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;

        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }

    uint8_t nf = as3935_get_noise_floor(as);
    ESP_LOGI(_AS_TAG, "Noise floor is %x", nf);

    return as;
}

void as3935_destroy(AS3935_handle_t as)
{
    if (as == NULL)
    {
        return;
    }

    free(as);
}

void as3935_power_up(AS3935_handle_t as)
{
    ESP_ERROR_CHECK(_as3935_register_write(as, AS3935_PWD, 0));
    ESP_ERROR_CHECK(_as3935_raw_register_write(as, 0x3D, 0x96));
    vTaskDelay(3 / portTICK_RATE_MS);
}

void as3935_power_down(AS3935_handle_t as)
{
    ESP_ERROR_CHECK(_as3935_register_write(as, AS3935_PWD, 1));
}

void as3935_calibrate(AS3935_handle_t as)
{
    int32_t target = 3125, current_count = 0, best_diff = INT32_MAX, curr_diff = 0;
    uint8_t best_tune = 0, curr_tune = 0;
    uint64_t set_up_time;
    int curr_irq, prev_irq;

    ESP_LOGI(_AS_TAG, "Starting calibration...");

    _as3935_register_write(as, AS3935_LCO_FDIV, 0);
    _as3935_register_write(as, AS3935_DISP_LCO, 1);

    for (curr_tune = 0; curr_tune <= 0x0F; curr_tune++)
    {
        ESP_LOGI(_AS_TAG, "Trying tune: %x", curr_tune);
        _as3935_register_write(as, AS3935_TUN_CAP, curr_tune);
        vTaskDelay(2 / portTICK_RATE_MS);
        current_count = 0;
        prev_irq = gpio_get_level(as->irq_pin);
        set_up_time = (xTaskGetTickCount() * portTICK_RATE_MS) + 100;
        while ((long)((xTaskGetTickCount() * portTICK_RATE_MS) - set_up_time) < 0)
        {
            curr_irq = gpio_get_level(as->irq_pin);
            if (curr_irq > prev_irq)
            {
                current_count++;
            }
            prev_irq = curr_irq;
        }
        curr_diff = target - current_count;
        if (curr_diff < 0)
            curr_diff = -curr_diff;
        if (best_diff > curr_diff)
        {
            best_diff = curr_diff;
            best_tune = curr_tune;
        }
        ESP_LOGI(_AS_TAG, "Current diff: %d", curr_diff);
    }
    ESP_LOGI(_AS_TAG, "Found best tune = %x", best_tune);
    ESP_LOGI(_AS_TAG, "Found best diff = %d", best_diff);
    _as3935_register_write(as, AS3935_TUN_CAP, best_tune);
    vTaskDelay(2 / portTICK_RATE_MS);
    _as3935_register_write(as, AS3935_DISP_LCO, 0);
    as3935_power_up(as);

    if (best_diff > 109)
    {
        ESP_LOGE(_AS_TAG, "Diff out of spec!");
    }
    else
    {
        ESP_LOGI(_AS_TAG, "Diff in spec");
    }
}

void as3935_reset(AS3935_handle_t as)
{
    ESP_ERROR_CHECK(_as3935_raw_register_write(as, 0x3C, 0x96));
    vTaskDelay(2 / portTICK_RATE_MS);
}

uint8_t as3935_get_noise_floor(AS3935_handle_t as)
{
    uint8_t data;
    ESP_ERROR_CHECK(_as3935_register_read(as, AS3935_NF_LEV, &data));
    return data;
}

void as3935_set_noise_floor(AS3935_handle_t as, uint8_t data)
{
    ESP_ERROR_CHECK(_as3935_register_write(as, AS3935_NF_LEV, data));
}

uint8_t as3935_get_interrupt_source(AS3935_handle_t as)
{
    uint8_t data;
    ESP_ERROR_CHECK(_as3935_register_read(as, AS3935_INT, &data));
    return data;
}

void as3935_set_disturbers_mask(AS3935_handle_t as, bool enabled)
{
    ESP_ERROR_CHECK(_as3935_register_write(as, AS3935_MASK_DIST, (uint8_t)enabled));
}

uint8_t as3935_get_minimum_lightnings(AS3935_handle_t as)
{
    uint8_t data;
    ESP_ERROR_CHECK(_as3935_register_read(as, AS3935_MIN_NUM_LIGH, &data));
    return data;
}

uint8_t as3935_set_minimum_lightnings(AS3935_handle_t as, uint8_t min_lightnings)
{
    ESP_ERROR_CHECK(_as3935_register_write(as, AS3935_MIN_NUM_LIGH, min_lightnings));
    return as3935_get_minimum_lightnings(as);
}

uint8_t as3935_get_lighting_distance_km(AS3935_handle_t as)
{
    uint8_t data;
    ESP_ERROR_CHECK(_as3935_register_read(as, AS3935_DISTANCE, &data));
    return data;
}

void as3935_set_afe_indoors(AS3935_handle_t as)
{
    ESP_ERROR_CHECK(_as3935_register_write(as, AS3935_AFE_GB, AS3935_AFE_INDOOR));
}

void as3935_set_afe_outdoors(AS3935_handle_t as)
{
    ESP_ERROR_CHECK(_as3935_register_write(as, AS3935_AFE_GB, AS3935_AFE_OUTDOOR));
}

uint8_t as3935_get_spike_rejection(AS3935_handle_t as)
{
    uint8_t data;
    ESP_ERROR_CHECK(_as3935_register_read(as, AS3935_SREJ, &data))
    return data;
}

uint8_t as3935_set_spike_rejection(AS3935_handle_t as, uint8_t srej)
{
    ESP_ERROR_CHECK(_as3935_register_write(as, AS3935_SREJ, srej))
    return as3935_get_spike_rejection(as);
}

uint8_t as3935_get_watchdog_threshold(AS3935_handle_t as)
{
    uint8_t data;
    ESP_ERROR_CHECK(_as3935_register_read(as, AS3935_WDTH, &data))
    return data;
}

uint8_t as3935_set_watchdog_threshold(AS3935_handle_t as, uint8_t wdth)
{
    ESP_ERROR_CHECK(_as3935_register_write(as, AS3935_WDTH, wdth))
    return as3935_get_watchdog_threshold(as);
}

void as3935_clear_stats(AS3935_handle_t as)
{
    ESP_ERROR_CHECK(_as3935_register_write(as, AS3935_CL_STAT, 1))
    ESP_ERROR_CHECK(_as3935_register_write(as, AS3935_CL_STAT, 0))
    ESP_ERROR_CHECK(_as3935_register_write(as, AS3935_CL_STAT, 1))
}
