#include "antenna.h"
#include "nrf_gpio.h"

//pin 0 and pin 9 are far from others so I don't use them.
const static uint16_t configs[] =
{
    0b11110000000000,
    0b01110100000000,
    0b00110110000000,
    0b00010111000000,
    0b00000111100000,
    0b00000011110000,
    0b00000001111000,
    0b00000000111100,
    0b00000000011110,
    0b10000000001110,
    0b11000000000110,
    0b11100000000010,
};
STATIC_ASSERT(ANTENNA_CONFIGS_NUM == sizeof(configs)/sizeof(uint16_t), "configs array should have size of ANTENNA_CONFIGS_NUM");

void antenna_init(antenna_t* antenna)
{
    antenna->configuration = 0;
    NRF_P1->DIR |= 0x3DFE;
    antenna_set_configuration(antenna, 0);
}

void antenna_set_configuration(antenna_t* antenna, uint8_t config)
{
    if(config > ANTENNA_CONFIGS_NUM)
        return;
    antenna->configuration = config;
    NRF_P1->OUT = (NRF_GPIO->OUT & ~0x3DFE) | configs[config];
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