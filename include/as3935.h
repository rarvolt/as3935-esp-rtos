//
// Created by rarvolt on 15.10.2019.
//

#ifndef THUNDER_DETECT_AS3935_H
#define THUNDER_DETECT_AS3935_H

#include <stdint.h>

// register access macros - register address, bitmask
#define AS3935_AFE_GB       0x00, 0x3E
#define AS3935_PWD          0x00, 0x01
#define AS3935_NF_LEV       0x01, 0x70
#define AS3935_WDTH         0x01, 0x0F
#define AS3935_CL_STAT      0x02, 0x40
#define AS3935_MIN_NUM_LIGH 0x02, 0x30
#define AS3935_SREJ         0x02, 0x0F
#define AS3935_LCO_FDIV     0x03, 0xC0
#define AS3935_MASK_DIST    0x03, 0x20
#define AS3935_INT          0x03, 0x0F
#define AS3935_S_LIG_L      0x04, 0xFF
#define AS3935_S_LIG_M      0x05, 0xFF
#define AS3935_S_LIG_MM     0x06, 0x1F
#define AS3935_DISTANCE     0x07, 0x3F
#define AS3935_DISP_LCO     0x08, 0x80
#define AS3935_DISP_SRCO    0x08, 0x40
#define AS3935_DISP_TRCO    0x08, 0x20
#define AS3935_TUN_CAP      0x08, 0x0F
#define AS3935_TRCO_CALIB_DONE  0x3A, 0x80
#define AS3935_TRCO_CALIB_NOK   0x3A, 0x40
#define AS3935_SRCO_CALIB_DONE  0x3B, 0x80
#define AS3935_SRCO_CALIB_NOK   0x3B, 0x40

// other constants
#define AS3935_AFE_INDOOR   0x12
#define AS3935_AFE_OUTDOOR  0x0E

#define AS3935_ADDR_01 0x01     // A1 - LOW,  A0 - HIGH
#define AS3935_ADDR_10 0x02     // A1 - HIGH, A0 - LOW
#define AS3935_ADDR_11 0x03     // A1 - HIGH, A0 - HIGH


struct AS3935_t
{
    uint8_t addr;
    int8_t irq_pin;
};

typedef struct AS3935_t* AS3935_handle_t;

AS3935_handle_t as3935_init(uint8_t address, int8_t irq_pin);
void as3935_destroy(AS3935_handle_t as);
void as3935_power_up(AS3935_handle_t as);
void as3935_power_down(AS3935_handle_t as);
void as3935_calibrate(AS3935_handle_t as);
void as3935_reset(AS3935_handle_t as);

uint8_t as3935_get_noise_floor(AS3935_handle_t as);
void as3935_set_noise_floor(AS3935_handle_t as, uint8_t nf);

uint8_t as3935_get_interrupt_source(AS3935_handle_t as);

void as3935_set_disturbers_mask(AS3935_handle_t as, bool enabled);

uint8_t as3935_get_minimum_lightnings(AS3935_handle_t as);
uint8_t as3935_set_minimum_lightnings(AS3935_handle_t as, uint8_t min_lightnings);

uint8_t as3935_get_lighting_distance_km(AS3935_handle_t as);

void as3935_set_afe_indoors(AS3935_handle_t as);
void as3935_set_afe_outdoors(AS3935_handle_t as);

uint8_t as3935_get_spike_rejection(AS3935_handle_t as);
uint8_t as3935_set_spike_rejection(AS3935_handle_t as, uint8_t srej);

uint8_t as3935_get_watchdog_threshold(AS3935_handle_t as);
uint8_t as3935_set_watchdog_threshold(AS3935_handle_t as, uint8_t wdth);

void as3935_clear_stats(AS3935_handle_t as);

#endif //THUNDER_DETECT_AS3935_H
