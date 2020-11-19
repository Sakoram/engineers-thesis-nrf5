#ifndef DoA_H__
#define DoA_H__

#include <stdint.h>
#include <stdbool.h>
#include "antenna.h"

#define DOA_MEASUREMENTS_SAMPLES_NUM 10
#define DOA_CALIBRATION_SAMPLES_NUM  10
#define DOA_ANGULAR_PRECISION        10
#define DOA_MEASURED_ANGLE_NUM       360 / DOA_ANGULAR_PRECISION
#define DOA_MISSED_PACKETS_MAX       12

#define DOA_GAIN36_FISRT \
    { .gain_samples = \
   {{186, 184, 184, 183, 184, 186, 184, 185, 187, 187, 188, 188},\
    {189, 185, 185, 187, 185, 185, 184, 187, 188, 187, 190, 190},\
    {190, 188, 185, 187, 185, 190, 187, 187, 188, 188, 190, 191},\
    {193, 190, 186, 187, 188, 185, 188, 186, 188, 189, 191, 193},\
    {194, 193, 190, 187, 190, 190, 188, 187, 189, 191, 193, 194},\
    {195, 194, 190, 188, 188, 188, 188, 187, 187, 190, 193, 194},\
    {194, 194, 190, 187, 184, 187, 188, 188, 187, 188, 191, 193},\
    {191, 193, 191, 189, 184, 187, 187, 188, 185, 188, 189, 191},\
    {194, 194, 193, 189, 187, 187, 187, 188, 189, 191, 190, 193},\
    {195, 194, 194, 191, 188, 187, 185, 188, 189, 190, 192, 193},\
    {194, 195, 194, 193, 191, 189, 188, 189, 191, 192, 192, 194},\
    {194, 195, 196, 195, 193, 189, 188, 188, 189, 191, 192, 195},\
    {193, 194, 196, 194, 193, 191, 189, 186, 185, 189, 189, 192},\
    {193, 193, 195, 194, 193, 189, 186, 185, 186, 188, 187, 188},\
    {193, 194, 195, 196, 194, 191, 188, 186, 187, 184, 188, 189},\
    {192, 194, 195, 197, 196, 194, 190, 189, 191, 191, 192, 191},\
    {193, 194, 195, 196, 196, 195, 192, 191, 190, 191, 192, 192},\
    {193, 193, 194, 195, 195, 194, 192, 188, 188, 187, 187, 191},\
    {192, 193, 192, 194, 194, 193, 192, 189, 188, 187, 187, 190},\
    {192, 193, 194, 194, 194, 195, 194, 191, 189, 187, 188, 190},\
    {191, 191, 193, 194, 195, 194, 194, 193, 190, 189, 189, 189},\
    {189, 190, 192, 193, 194, 196, 195, 192, 189, 189, 187, 187},\
    {189, 190, 191, 193, 194, 195, 194, 193, 190, 189, 188, 189},\
    {187, 189, 190, 192, 192, 193, 193, 193, 190, 187, 184, 187},\
    {184, 185, 186, 188, 189, 190, 192, 192, 191, 188, 183, 181},\
    {184, 183, 185, 185, 187, 188, 190, 189, 189, 188, 184, 184},\
    {185, 186, 186, 187, 187, 191, 190, 190, 188, 187, 185, 186},\
    {186, 185, 185, 184, 187, 190, 190, 191, 191, 191, 188, 184},\
    {186, 189, 188, 184, 186, 189, 190, 191, 193, 192, 192, 190},\
    {185, 186, 186, 189, 191, 192, 192, 194, 195, 195, 194, 190},\
    {184, 187, 186, 187, 188, 190, 191, 193, 194, 194, 193, 189},\
    {188, 186, 186, 187, 187, 190, 191, 193, 194, 194, 192, 189},\
    {189, 185, 184, 187, 185, 188, 190, 192, 193, 194, 194, 192},\
    {188, 186, 187, 187, 187, 185, 182, 188, 189, 193, 193, 191},\
    {185, 186, 186, 190, 189, 189, 187, 189, 190, 191, 192, 189},\
    {184, 184, 187, 181, 181, 183, 182, 190, 189, 187, 189, 189}}};

#define DOA_DEFAULT_VALUE    DOA_GAIN36_FISRT


typedef enum
{
    DOA_PACKET_TIMEOUT,
    DOA_CALIBRATION_END,
    DOA_DIRECTION_ESTIMATED,
    DOA_ANGLE_SETUP_REQUIRED,
    DOA_WAITING_FOR_START,
    DOA_ANGLE_MEASURED,
} doa_evt_id_t;

typedef enum
{
    DOA_WITHOUT_CALIBRATION,    //gain samples have to be provided in compile time
    DOA_MANUAL_CALIBRATION,     //the antenna DOES wait for confirmation of setup of the next angle
    DOA_AUTOMATIC_CALIBRATION,  //the antenna does NOT wait for confirmation of setup of the next angle
} doa_calibration_mode_t;

typedef struct
{
    doa_evt_id_t evt_id;       //!< Event ID.
    union
    {
        bool        calibration_successful;
        bool        timeout_while_configuration;
        uint16_t    direction;
        uint16_t    angle;
        struct {
            uint16_t angle;
            uint8_t* samples;
        } calibration_samples;
    } param;
} doa_evt_t;

typedef void (*doa_evt_handler_t) (doa_evt_t const * p_evt);

typedef struct 
{
    uint8_t            rss_samples_measurements[DOA_MEASUREMENTS_SAMPLES_NUM];
    uint8_t            rss_samples_calibration[DOA_CALIBRATION_SAMPLES_NUM];
    uint8_t            gain_samples[DOA_MEASURED_ANGLE_NUM][ANTENNA_CONFIGS_NUM];
    uint8_t            median_rss[ANTENNA_CONFIGS_NUM];
    bool               calibrated;
    bool               manual_angle_setup;
    bool               angle_set;
    uint8_t            missed_packets;
    uint8_t            sample_count;
    uint16_t           measured_angle;
    antenna_t          antenna;
    doa_evt_handler_t  evt_handler;                     //!< DoA event handler.
}doa_t;

void doa_init(doa_t* doa, doa_evt_handler_t doa_evt_handler, doa_calibration_mode_t doa_calibration_mode);
void doa_receive_RSS(doa_t* doa, uint8_t rss);
void doa_start_calibration(doa_t* doa, bool single_packet_per_angle);
uint16_t doa_inc_angle(doa_t* doa);
uint16_t doa_dec_angle(doa_t* doa);
void doa_receive_RSS_timeout(doa_t* doa);
void doa_start(doa_t* doa);



#endif // DoA_H__