#ifndef SIMULATION_H__
#define SIMULATION_H__

#include "doa.h"

void simulation_send_calibration_packets(doa_t* doa);
uint16_t simulation_send_measurement_packets(doa_t* doa);

#endif // SIMULATION_H__