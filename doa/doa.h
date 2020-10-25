#ifndef DoA_H__
#define DoA_H__

#include <stdint.h>
#include <stdbool.h>
#include "antenna.h"

#define DOA_SAMPLES_NUM           1
#define DOA_ANGULAR_PRECISION     1
#define DOA_MESSERED_ANGLE_NUM    360 / DOA_ANGULAR_PRECISION
#define DOA_MISSED_PACKETS_MAX    12

typedef enum
{
    DOA_PACKET_TIMEOUT,
    DOA_CALIBRATION_END,
    DOA_DIRECTION_ESTIMATED,
    DOA_ANGLE_SETUP_REQUIRED,
} doa_evt_id_t;

typedef struct
{
    doa_evt_id_t evt_id;       //!< Event ID.
    union
    {
        bool        calibration_successful;
        bool        timeout_while_configuration;
        uint16_t    direction;
    } param;
} doa_evt_t;

typedef void (*doa_evt_handler_t) (doa_evt_t const * p_evt);

typedef struct 
{
    uint8_t            rss_samples[DOA_SAMPLES_NUM];
    uint8_t            calibration_rss_samples[DOA_MESSERED_ANGLE_NUM][ANTENNA_CONFIGS_NUM];
    uint8_t            mean_rss[ANTENNA_CONFIGS_NUM];
    bool               calibrated;
    bool               manual_angle_setup;
    bool               angle_set;
    uint8_t            missed_packets;
    uint8_t            sample_count;
    uint16_t           messered_angle;
    antenna_t          antenna;
    doa_evt_handler_t  evt_handler;                     //!< DoA event handler.
}doa_t;

void doa_init(doa_t* doa, doa_evt_handler_t doa_evt_handler);
void doa_receive_RSS(doa_t* doa, uint8_t rss);
void doa_start_calibration(doa_t* doa, bool single_packet_per_angle);
uint16_t doa_inc_angle(doa_t* doa);
uint16_t doa_dec_angle(doa_t* doa);
void doa_receive_RSS_timeout(doa_t* doa);

#endif // DoA_H__