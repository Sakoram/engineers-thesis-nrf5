#ifndef SIMULATION_H__
#define SIMULATION_H__

#include "doa.h"

#define DOA_SIMULATION_USED false  // true or false
//_Static_assert(!(DOA_SIMULATION_USED == true && defined (DOA_DEFAULT_VALUE)) , "simulation CANNOT be performed if DOA_DEFAULT_VALUE is used");

void simulation_send_calibration_packets(doa_t* doa);
uint16_t simulation_send_measurement_packets(doa_t* doa);

#endif // SIMULATION_H__