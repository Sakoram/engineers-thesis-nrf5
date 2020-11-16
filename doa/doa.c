#include "doa.h"
#include "math.h"
#include "nrf_log.h"

static void doa_calculate_doa(doa_t* doa);
static void doa_set_calibration_sample(doa_t* doa, uint8_t rss);
static void doa_analyze_direction(doa_t* doa);
static void _doa_receive_RSS(doa_t* doa, uint8_t rss);

void doa_init(doa_t* doa, doa_evt_handler_t doa_evt_handler)
{
    doa->calibrated     = false;
    doa->sample_count   = 0;
    doa->evt_handler    = doa_evt_handler;
    doa->messered_angle = 0;

    antenna_init(&doa->antenna);
}

inline void doa_receive_RSS(doa_t* doa, uint8_t rss)
{
    doa->missed_packets = 0;
    _doa_receive_RSS(doa, rss);
}

static void _doa_receive_RSS(doa_t* doa, uint8_t rss)
{
    //NRF_LOG_DEBUG("_doa_receive_RSS rss == %d", rss);
    if (doa->calibrated)
    {
        doa->rss_samples[doa->sample_count++] = rss;
        if (doa->sample_count == DOA_SAMPLES_NUM)
        {
            doa->sample_count = 0;
            doa_analyze_direction(doa);
        }
    }
    else
    {
        if (doa->angle_set)
            doa_set_calibration_sample(doa, rss);
    }
}

void doa_receive_RSS_timeout(doa_t* doa)
{
    doa_evt_t evt;
    if(doa->calibrated)
    {
        evt.evt_id = DOA_PACKET_TIMEOUT;
        evt.param.timeout_while_configuration = false;
        doa->evt_handler(&evt);
        _doa_receive_RSS(doa, 0);
    }
    else if(doa->angle_set)
    {
        evt.evt_id = DOA_PACKET_TIMEOUT;
        evt.param.timeout_while_configuration = true;
        doa->evt_handler(&evt);

        doa->missed_packets++;
        if (doa->missed_packets < DOA_MISSED_PACKETS_MAX)
            doa_set_calibration_sample(doa, 0);
        else
        {
            doa->missed_packets = 0;
            doa_evt_t evt;
            evt.evt_id = DOA_CALIBRATION_END;
            evt.param.calibration_successful = false;
            doa->evt_handler(&evt);
            doa_start_calibration(doa, doa->manual_angle_setup);
        }
    }
}

static void doa_calculate_doa(doa_t* doa)
{
    doa_evt_t evt;
    evt.evt_id = DOA_DIRECTION_ESTIMATED;
    float T[DOA_MESSERED_ANGLE_NUM];
    float sumPY, sumP, sumY;
    for (uint16_t angle = 0; angle < DOA_MESSERED_ANGLE_NUM; angle++)
    {
        sumPY = sumP = sumY = 0;
        for (uint8_t configuration = 0; configuration < ANTENNA_CONFIGS_NUM; configuration++)
        {
            sumPY += doa->calibration_rss_samples[angle][configuration] *
                     doa->mean_rss[configuration];
            sumP  += doa->calibration_rss_samples[angle][configuration];
            sumY  += doa->mean_rss[configuration];
        }
        T[angle] = sumPY / (sqrt(sumP*sumP) * sqrt(sumY*sumY)); 
    }
    uint16_t direction = 0;
    for (uint16_t i = 0; i < DOA_MESSERED_ANGLE_NUM; i++)
    {
            if (T[direction] < T[i]) 
            {
                direction = i;
            }
    }
    evt.param.direction = direction;
    doa->evt_handler(&evt);
}

static void doa_set_calibration_sample(doa_t* doa, uint8_t rss)
{
    doa_evt_t evt;
    NRF_LOG_DEBUG("doa_set_calibration_sample: %d angle == %d, config == %d ", rss,  doa->messered_angle, doa->antenna.configuration);
    doa->calibration_rss_samples[doa->messered_angle][doa->antenna.configuration] = rss;
    if (antenna_set_next_configuration(&doa->antenna))
    {
        if (doa->messered_angle == DOA_MESSERED_ANGLE_NUM - 1)
        {
            doa->calibrated = true;
            evt.evt_id = DOA_CALIBRATION_END;
            evt.param.calibration_successful = true;
            doa->evt_handler(&evt);
        }
        else if (doa->manual_angle_setup)
        {
            doa->angle_set = false;
            evt.evt_id = DOA_ANGLE_SETUP_REQUIRED;
            doa->evt_handler(&evt);
        }
        else
        {
            doa_inc_angle(doa);
        }
    }
}

static void doa_analyze_direction(doa_t* doa)
{
    uint16_t sum_rss;
    for(uint8_t i = 0; i < DOA_SAMPLES_NUM; i++)
        sum_rss += doa->rss_samples[i];

    doa->mean_rss[doa->antenna.configuration] = sum_rss / DOA_SAMPLES_NUM;
    if (antenna_set_next_configuration(&doa->antenna))
        doa_calculate_doa(doa);
}

void doa_start_calibration(doa_t* doa, bool manual_angle_setup)
{
    antenna_set_configuration(&doa->antenna, 0);
    doa->messered_angle     = 0;
    doa->sample_count       = 0;
    doa->manual_angle_setup = manual_angle_setup;
    doa->calibrated         = false;
    if(manual_angle_setup)
    {
        doa->angle_set = false;
        doa_evt_t evt;
        evt.evt_id = DOA_WAITING_FOR_START;
        doa->evt_handler(&evt);
    }
    else
        doa->angle_set = true;
}

uint16_t doa_inc_angle(doa_t* doa)
{
    if (doa->messered_angle < DOA_MESSERED_ANGLE_NUM - 1)
    {
        doa->messered_angle++;
        doa->angle_set = true;
    }

    return doa->messered_angle;
}
uint16_t doa_dec_angle(doa_t* doa)
{
    if (doa->messered_angle > 0)
    {
        doa->messered_angle--;
        doa->angle_set = true;
    }
    return doa->messered_angle;
}
void doa_start(doa_t* doa)
{
    doa->angle_set = true;
}