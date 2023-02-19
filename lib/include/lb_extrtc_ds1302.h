// error codes
#define DS1302_OK               0
#define DS1302_ERR_INIT         1

//! DS1302 registers
#define DS1302_REG_SECONDS      0x00    //!< Seconds register
#define DS1302_REG_MINUTES      0x01    //!< Minutes register
#define DS1302_REG_HOURS        0x02    //!< Hours register
#define DS1302_REG_DAY_MONTH    0x03    //!< Day of the month register
#define DS1302_REG_MONTH        0x04    //!< Month register
#define DS1302_REG_DAY_WEEK     0x05    //!< Day of the week register
#define DS1302_REG_YEAR         0x06    //!< Year register
#define DS1302_REG_WP           0x07    //!< Write protect register
#define DS1302_REG_TC           0x08    //!< Tickle Charger register
#define DS1302_REG_CLOCK_BURST  0x1F    //!< Clock Burst Register

// structures 
typedef struct ds1302_s {
    gpio_num_t sclk;
    gpio_num_t data;
    gpio_num_t rst;
} ds1302_t;

typedef struct ds1302_conf_s {
    gpio_num_t sclk;
    gpio_num_t data;
    gpio_num_t rst;
} ds1302_conf_t;

typedef struct ds1302_time_s {
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t dayWeek;
    uint8_t dayMonth;
    uint8_t month;
    uint16_t year;
} ds1302_time_t;

// functionss
int32_t lb_ds1302_open(ds1302_conf_t *cfg, ds1302_t *dev);
void lb_ds1302_get_time(ds1302_t *dev, ds1302_time_t *rtc);
void lb_ds1302_set_time(ds1302_t *dev, ds1302_time_t *rtc);
