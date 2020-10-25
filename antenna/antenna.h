#ifndef ANTENNA_H__
#define ANTENNA_H__

#include <stdint.h>
#include <stdbool.h>

#define ANTENNA_CONFIGS_NUM   12  // number of configurations in configs array

typedef struct 
{
    uint8_t            configuration;
}antenna_t;

void antenna_init(antenna_t* antenna);
void antenna_set_configuration(antenna_t* antenna, uint8_t config);
bool antenna_set_next_configuration(antenna_t* antenna);

#endif // ANTENNA_H__