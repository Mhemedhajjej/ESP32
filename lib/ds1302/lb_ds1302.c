#include "stdio.h"
#include "stdlib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/lb_extrtc_ds1302.h"
#include <string.h>

// bitfiels of command byte 
#define BF_READ_FLAG    0
#define BF_REG          1
#define BF_RAM_CAL      6
#define BF_ONE          7

// read/write flags
#define READ    1
#define WRITE   0

// macros for register value <-> decimal conversion
#define REG_TO_DEC(reg)         ((reg & 0x0F) + (reg >> 4)*10)
#define DEC_TO_REG(dec)         ((dec % 10) + ((dec / 10) << 4))

static char week[7][10] = {"Monday","Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};

// private function
static void _access_register(ds1302_t *dev, uint8_t reg, uint8_t *wbuf, uint8_t *rbuf);
static void _access_register_burst(ds1302_t *dev, uint8_t reg, uint8_t *wbuf, uint8_t *rbuf, uint8_t len);
static void _write_byte(ds1302_t *dev, uint8_t byte);
static void _read_byte(ds1302_t *dev, uint8_t *byte);
static void _power_on(ds1302_t *dev);
static void _transfer_begin(ds1302_t *dev);
static void _transfer_stop(ds1302_t *dev);
static void _send_command_byte(ds1302_t *dev, uint8_t reg, bool read);

//functions
int32_t lb_ds1302_open(ds1302_conf_t *conf, ds1302_t *dev)
{      
        /* configure SCLK Pin */
        if (GPIO_IS_VALID_GPIO(conf->sclk)){
                gpio_reset_pin(conf->sclk);
                gpio_set_direction(conf->sclk, GPIO_MODE_OUTPUT);
                gpio_set_level(conf->sclk, 0);
        } else
                return -DS1302_ERR_INIT;
        /* configure DATA Pin */
        if (GPIO_IS_VALID_GPIO(conf->data)){
                gpio_reset_pin(conf->data);
                gpio_set_direction(conf->data, GPIO_MODE_OUTPUT);
                gpio_set_level(conf->data, 0);
        } else
                return -DS1302_ERR_INIT;

        /* configure RST Pin */
        if (GPIO_IS_VALID_GPIO(conf->rst)){
                gpio_reset_pin(conf->rst);
                gpio_set_direction(conf->rst, GPIO_MODE_OUTPUT);
                gpio_set_level(conf->rst, 0);
        } else
                return -DS1302_ERR_INIT;

         /* copy pins to device structure */
        memcpy(dev, conf, sizeof(ds1302_t));

        /* power on RTC */
        _power_on(dev);

        return DS1302_OK;
}

void lb_ds1302_get_time(ds1302_t *dev, ds1302_time_t *rtc)
{
        uint8_t value[7];
        char text[256];
        char text_tmp[256];
        /* read calendar with burst transfer */
        _access_register_burst(dev, DS1302_REG_CLOCK_BURST, NULL, value, 7);

        //convert to decimal values
        rtc->second = REG_TO_DEC(value[0] & 0x7F);
        rtc->minute = REG_TO_DEC(value[1]);
        rtc->hour = REG_TO_DEC(value[2]);
        rtc->dayMonth = REG_TO_DEC(value[3]);
        rtc->month = REG_TO_DEC(value[4]);
        rtc->dayWeek = REG_TO_DEC(value[5]);
        rtc->year = 2000 + REG_TO_DEC(value[6]);

        sprintf(text, "%d", rtc->dayMonth);
        strcat(text, "-");
        sprintf(text_tmp, "%d", rtc->month);
        strcat(text, text_tmp);
        strcat(text, "-");
        sprintf(text_tmp, "%d", rtc->year);
        strcat(text, text_tmp);
        ESP_LOGI(text, "DATE");
        sprintf(text, "%d", rtc->hour);
        strcat(text, ":");
        sprintf(text_tmp, "%d", rtc->minute);
        strcat(text, text_tmp);
        ESP_LOGI(text, "TIME");
}

void lb_ds1302_set_time(ds1302_t *dev, ds1302_time_t *rtc)
{
        uint8_t time[7];

        /* prepare data to write */
        time[0] = 0 | (DEC_TO_REG((uint8_t)(rtc->second & 0x7F)));
        time[1] = DEC_TO_REG((uint8_t)(rtc->minute & 0x7F));
        time[2] = DEC_TO_REG((uint8_t)(rtc->hour & 0x3F)); //24H
        time[3] = DEC_TO_REG((uint8_t)(rtc->dayMonth & 0x3F)); 
        time[4] = DEC_TO_REG((uint8_t)(rtc->month & 0x1F));
        time[5] = DEC_TO_REG((uint8_t)(rtc->dayWeek & 0x07));
        time[6] = DEC_TO_REG((uint8_t)(rtc->year - 2000));

        _access_register_burst(dev, DS1302_REG_CLOCK_BURST, time, NULL, 7);
}

//private functions

static void _access_register(ds1302_t *dev, uint8_t reg, uint8_t *wbuf, uint8_t *rbuf)
{
        _transfer_begin(dev);

        _send_command_byte(dev, reg, (rbuf != NULL));

        if (rbuf)
                _read_byte(dev, rbuf);

        if (wbuf)
                _write_byte(dev, *wbuf);

        _transfer_stop(dev);
}

static void _access_register_burst(ds1302_t *dev, uint8_t reg, uint8_t *wbuf, uint8_t *rbuf, uint8_t len)
{
        uint8_t i;
        uint8_t value;
        
        _transfer_begin(dev);

        _send_command_byte(dev, reg, (rbuf != NULL));

        if (rbuf) {
                for (i = 0; i < len; i++) {
                        _read_byte(dev, &value);
                        rbuf[i] = value;
                }
        }

        if (wbuf) {
                for (i = 0; i < len; i++) {
                        _write_byte(dev, wbuf[i]);
                }
        }

        _transfer_stop(dev);
}

/* @brief write byte 
*/
static void _write_byte(ds1302_t *dev, uint8_t byte)
{
    uint8_t i = 0;
    uint8_t lvl;

    for (i = 0; i < 8; i++){
        lvl = (byte & (1 << i)) >> i;
        gpio_set_level(dev->sclk, 0);
        gpio_set_level(dev->data, lvl);

        /* clock generation */
        gpio_set_level(dev->sclk, 1);
        vTaskDelay(1);
    }
}

static void _read_byte(ds1302_t *dev, uint8_t *byte)
{
    uint8_t i;
    uint8_t lvl;
    uint8_t value = 0;

    for (i = 0; i < 8; i++){
        gpio_set_level(dev->sclk, 1);
        gpio_set_level(dev->sclk, 0);
        vTaskDelay(1);

        lvl = gpio_get_level(dev->data);

        value |= (lvl << i);
    }

    *byte = value;
}

static void _power_on(ds1302_t *dev)
{
        uint8_t rval = 0;
        uint8_t wval = 0;
        char debug[256];

        /* if clock is stoped => start clock by clear CH bit in seconds reg */
        _access_register(dev, DS1302_REG_SECONDS, NULL, &rval);
        if (rval & 0x80){
                wval = rval & 0x7F; // clear CH bit
                _access_register(dev, DS1302_REG_SECONDS, &wval, NULL);
        }

        /* clear write protect */
        wval = 0;
        _access_register(dev, DS1302_REG_WP, &wval, NULL);
}

static void _transfer_begin(ds1302_t *dev)
{
        gpio_set_level(dev->sclk, 0);
        gpio_set_level(dev->data, 0);
        gpio_set_direction(dev->data, GPIO_MODE_OUTPUT);
        gpio_set_level(dev->rst, 1);
}

static void _transfer_stop(ds1302_t *dev)
{
         gpio_set_level(dev->rst, 0);
}

static void _send_command_byte(ds1302_t *dev, uint8_t reg, bool read)
{
        uint8_t cmd = 0;

        // we support currently only calendar access
        cmd = (read << BF_READ_FLAG)
                | ((reg & 0x3F) << BF_REG)
                | ((0 << BF_RAM_CAL)) // only calendar access is supported
                | (1 << BF_ONE);

        _write_byte(dev, cmd);

        if (read)
               gpio_set_direction(dev->data, GPIO_MODE_INPUT); 
}
