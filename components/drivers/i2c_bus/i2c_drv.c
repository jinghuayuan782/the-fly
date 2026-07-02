/**
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 *
 * i2c_drv.c - i2c driver implementation
 *
 * @note
 * For some reason setting CR1 reg in sequence with
 * I2C_AcknowledgeConfig(I2C_SENSORS, ENABLE) and after
 * I2C_GenerateSTART(I2C_SENSORS, ENABLE) sometimes creates an
 * instant start->stop condition (3.9us long) which I found out with an I2C
 * analyzer. This fast start->stop is only possible to generate if both
 * start and stop flag is set in CR1 at the same time. So i tried setting the CR1
 * at once with I2C_SENSORS->CR1 = (I2C_CR1_START | I2C_CR1_ACK | I2C_CR1_PE) and the
 * problem is gone. Go figure...
 */


#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "stm32_legacy.h"
#include "i2c_drv.h"
#include "i2cdev.h"
#include "config.h"
#define DEBUG_MODULE "I2CDRV"
#include "debug_cf.h"

// Definitions of sensors I2C bus
#define I2C_DEFAULT_SENSORS_CLOCK_SPEED             400000

// Definition of eeprom and deck I2C buss,use two i2c with 400Khz clock simultaneously could trigger the watchdog
#define I2C_DEFAULT_DECK_CLOCK_SPEED                100000

static bool isinit_i2cPort[2] = {0, 0};

// Cost definitions of busses
static const I2cDef sensorBusDef = {
    .i2cPort            = I2C_NUM_0,
    .i2cClockSpeed      = I2C_DEFAULT_SENSORS_CLOCK_SPEED,
    .gpioSCLPin         = CONFIG_I2C0_PIN_SCL,
    .gpioSDAPin         = CONFIG_I2C0_PIN_SDA,
    .gpioPullup         = GPIO_PULLUP_ENABLE,
};

I2cDrv sensorsBus = {
    .def                = &sensorBusDef,
};

static const I2cDef deckBusDef = {
    .i2cPort            = I2C_NUM_1,
    .i2cClockSpeed      = I2C_DEFAULT_DECK_CLOCK_SPEED,
    .gpioSCLPin         = CONFIG_I2C1_PIN_SCL,
    .gpioSDAPin         = CONFIG_I2C1_PIN_SDA,
    .gpioPullup         = GPIO_PULLUP_ENABLE,
};

I2cDrv deckBus = {
    .def                = &deckBusDef,
};

static void i2cdrvInitBus(I2cDrv *i2c)
{
    if (isinit_i2cPort[i2c->def->i2cPort]) {
        return;
    }

    i2c_config_t conf = {0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = i2c->def->gpioSDAPin;
    conf.sda_pullup_en = i2c->def->gpioPullup;
    conf.scl_io_num = i2c->def->gpioSCLPin;
    conf.scl_pullup_en = i2c->def->gpioPullup;
    conf.master.clk_speed = i2c->def->i2cClockSpeed;
    esp_err_t err = i2c_param_config(i2c->def->i2cPort, &conf);

    if (!err) {
        err = i2c_driver_install(i2c->def->i2cPort, conf.mode, 0, 0, 0);
    }

    DEBUG_PRINTI(" i2c %d driver install return = %d", i2c->def->i2cPort, err);
    i2c->isBusFreeMutex = xSemaphoreCreateMutex();
    isinit_i2cPort[i2c->def->i2cPort] = true;
}


//-----------------------------------------------------------

void i2cdrvInit(I2cDrv *i2c)
{
    i2cdrvInitBus(i2c);
}

void i2cdrvTryToRestartBus(I2cDrv *i2c)
{
    i2cdrvInitBus(i2c);
}

void i2cdrvScanBus(I2cDrv *i2c)
{
    uint8_t addr;
    int found_count = 0;
    
    DEBUG_PRINTI("Scanning I2C bus %d...\n", i2c->def->i2cPort);
    
    for (addr = 0x03; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK_EN);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(i2c->def->i2cPort, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            DEBUG_PRINTI("Found device at address: 0x%02X\n", addr);
            found_count++;
        } else if (ret == ESP_ERR_TIMEOUT) {
            DEBUG_PRINTD("Timeout at address: 0x%02X\n", addr);
        }
    }
    
    if (found_count == 0) {
        DEBUG_PRINTE("No I2C devices found on bus %d\n", i2c->def->i2cPort);
    } else {
        DEBUG_PRINTI("Total %d I2C devices found\n", found_count);
    }
}

