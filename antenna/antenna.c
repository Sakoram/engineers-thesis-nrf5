#include "antenna.h"
#include "nrf_gpio.h"

#define NJG_CTRL_1  NRF_GPIO_PIN_MAP(1, 15)
#define NJG_CTRL_2  NRF_GPIO_PIN_MAP(1, 13)
#define NJG_CTRL_3  NRF_GPIO_PIN_MAP(1, 10)
#define NJG_CTRL_4  NRF_GPIO_PIN_MAP(1, 12)
#define NJG_CTRL_5  NRF_GPIO_PIN_MAP(1, 9)
#define NJG_CTRL_6  NRF_GPIO_PIN_MAP(0, 12)
#define NJG_CTRL_7  NRF_GPIO_PIN_MAP(0, 11)
#define NJG_CTRL_8  NRF_GPIO_PIN_MAP(0, 14)
#define NJG_CTRL_9  NRF_GPIO_PIN_MAP(0, 15)
#define NJG_CTRL_10 NRF_GPIO_PIN_MAP(0, 13)
#define NJG_CTRL_11 NRF_GPIO_PIN_MAP(0, 17)
#define NJG_CTRL_12 NRF_GPIO_PIN_MAP(0, 21)

const static uint16_t configs[ANTENNA_CONFIGS_NUM][4] =
{
    {NJG_CTRL_1,  NJG_CTRL_2,  NJG_CTRL_3,  NJG_CTRL_4},
    {NJG_CTRL_2,  NJG_CTRL_3,  NJG_CTRL_4,  NJG_CTRL_5},
    {NJG_CTRL_3,  NJG_CTRL_4,  NJG_CTRL_5,  NJG_CTRL_6},
    {NJG_CTRL_4,  NJG_CTRL_5,  NJG_CTRL_6,  NJG_CTRL_7},
    {NJG_CTRL_5,  NJG_CTRL_6,  NJG_CTRL_7,  NJG_CTRL_8},
    {NJG_CTRL_6,  NJG_CTRL_7,  NJG_CTRL_8,  NJG_CTRL_9},
    {NJG_CTRL_7,  NJG_CTRL_8,  NJG_CTRL_9,  NJG_CTRL_10},
    {NJG_CTRL_8,  NJG_CTRL_9,  NJG_CTRL_10, NJG_CTRL_11},
    {NJG_CTRL_9,  NJG_CTRL_10, NJG_CTRL_11, NJG_CTRL_12},
    {NJG_CTRL_10, NJG_CTRL_11, NJG_CTRL_12, NJG_CTRL_1},
    {NJG_CTRL_11, NJG_CTRL_12, NJG_CTRL_1,  NJG_CTRL_2},
    {NJG_CTRL_12, NJG_CTRL_1,  NJG_CTRL_2,  NJG_CTRL_3},
};

const static uint16_t pins[ANTENNA_CONFIGS_NUM] =
{
    NJG_CTRL_1,  NJG_CTRL_2,  NJG_CTRL_3,  NJG_CTRL_4,  NJG_CTRL_5,  NJG_CTRL_6,
    NJG_CTRL_7,  NJG_CTRL_8,  NJG_CTRL_9,  NJG_CTRL_10, NJG_CTRL_11, NJG_CTRL_12
};

void antenna_init(antenna_t* antenna)
{
    antenna->configuration = 0;
    for(uint8_t i = 0; i < ANTENNA_CONFIGS_NUM; i++)
        nrf_gpio_cfg_output(pins[i]);
    antenna_set_configuration(antenna, 0);
}

void antenna_set_configuration(antenna_t* antenna, uint8_t config)
{
    if(config > ANTENNA_CONFIGS_NUM)
        return;
    for(uint8_t i = 0; i < ANTENNA_CONFIGS_NUM; i++)
        nrf_gpio_pin_clear(pins[i]);

    for(uint8_t i = 0; i < 4; i++)
        nrf_gpio_pin_set(configs[config][i]);

}

/**
 * @brief Function sets next configuration or if current config is last one, it
 * sets 1st configuration.
 *
 * @param antenna antenna structure.
 *
 * @retval was it last configuration?
 */
bool antenna_set_next_configuration(antenna_t* antenna)
{
    antenna->configuration++;
    if(antenna->configuration < ANTENNA_CONFIGS_NUM)
    {
        antenna_set_configuration(antenna, antenna->configuration);
        return false;
    }
    else
    {
        antenna->configuration = 0;
        antenna_set_configuration(antenna, antenna->configuration);
        return true;
    }
}