#include "doa.h"
#include "math.h"
#include "nrf_log.h"

static void doa_calculate_doa(doa_t* doa);
static void doa_set_gain_sample(doa_t* doa, float rss);
static void doa_analyze_direction(doa_t* doa);
static void _doa_receive_RSS(doa_t* doa, int8_t rss);
static int8_t median(uint8_t size, int8_t array[]);
static float mean(uint8_t size, int8_t array[]);

void doa_init(doa_t* doa, doa_evt_handler_t doa_evt_handler, doa_calibration_mode_t doa_calibration_mode)
{
    doa->calibrated     = false;
    doa->estimate       = false;
    doa->sample_count   = 0;
    doa->evt_handler    = doa_evt_handler;
    doa->measured_angle = 0;

    antenna_init(&doa->antenna);
    switch(doa_calibration_mode)
    {
        case DOA_WITHOUT_CALIBRATION:
        {
            doa->angle_set  = true;
            doa->calibrated = true;
            break;
        }
        case DOA_MANUAL_CALIBRATION:
        {
            doa_start_calibration(doa, true);
            break;
        }
        case DOA_AUTOMATIC_CALIBRATION:
        {
            doa_start_calibration(doa, false);
            break;
        }
        
        default:
            break;
        }
    
}

inline void doa_receive_RSS(doa_t* doa, int8_t rss)
{
    doa->missed_packets = 0;
    _doa_receive_RSS(doa, rss);
}

static void _doa_receive_RSS(doa_t* doa, int8_t rss)
{
    //NRF_LOG_DEBUG("_doa_receive_RSS rss == %d", rss);
    doa_evt_t evt;
    if (doa->calibrated)
    {
        if(doa->estimate)
        {
            doa->rss_samples_measurements[doa->sample_count++] = rss;
            if (doa->sample_count == DOA_MEASUREMENTS_SAMPLES_NUM)
            {
                doa->sample_count = 0;
                float mean_of_samples = mean(DOA_MEASUREMENTS_SAMPLES_NUM, doa->rss_samples_measurements);
                double sample_in_mW = pow(10, mean_of_samples / 10);
                doa->mean_rss[doa->antenna.configuration] = sample_in_mW;
                evt.evt_id = DOA_MEASUREMENTS_CONFIG_MEASURED;
                evt.param.measurements_sample_per_config.config  = doa->antenna.configuration;
                evt.param.measurements_sample_per_config.sample = mean_of_samples;
                doa->evt_handler(&evt);
                if (antenna_set_next_configuration(&doa->antenna))
                    doa_calculate_doa(doa);
            }
        }
    }
    else
    {
        if (doa->angle_set)
            doa->rss_samples_calibration[doa->sample_count++] = rss;
            if (doa->sample_count == DOA_CALIBRATION_SAMPLES_NUM)
            {
                doa->sample_count = 0;
                evt.evt_id = DOA_CALIBRATION_CONFIG_MEASURED;
                evt.param.calibrationg_samples_per_config.angle        = doa->measured_angle * DOA_ANGULAR_PRECISION;
                evt.param.calibrationg_samples_per_config.config        = doa->antenna.configuration;
                evt.param.calibrationg_samples_per_config.samples_size = DOA_CALIBRATION_SAMPLES_NUM;
                evt.param.calibrationg_samples_per_config.samples      = doa->rss_samples_calibration;
                doa->evt_handler(&evt);
                float mean_of_samples = mean(DOA_CALIBRATION_SAMPLES_NUM, doa->rss_samples_calibration);
                doa_set_gain_sample(doa, mean_of_samples);
            }
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
        _doa_receive_RSS(doa, -127);
    }
    else if(doa->angle_set)
    {
        evt.evt_id = DOA_PACKET_TIMEOUT;
        evt.param.timeout_while_configuration = true;
        doa->evt_handler(&evt);

        doa->missed_packets++;
        if (doa->missed_packets < DOA_MISSED_PACKETS_MAX)
            _doa_receive_RSS(doa, -127);
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
    doa->estimate = false;
    doa_evt_t evt;
    double T[DOA_MEASURED_ANGLE_NUM];
    double sumPY, sumP, sumY;
    for (uint8_t pattern = 0; pattern < 3; pattern++)
    {
        for (uint16_t angle = 0; angle < DOA_MEASURED_ANGLE_NUM; angle++)
        {
            sumPY = sumP = sumY = 0;
            for (uint8_t configuration = 0; configuration < ANTENNA_CONFIGS_NUM; configuration++)
            {
                sumPY += doa->gain_samples[pattern][angle][configuration] *
                         doa->mean_rss[configuration];
                sumP  += doa->gain_samples[pattern][angle][configuration];
                sumY  += doa->mean_rss[configuration];
            }
            T[angle] = sumPY / (sqrt(sumP * sumP) * sqrt(sumY * sumY)); 
        }
        uint16_t direction = 0;
        for (uint16_t i = 0; i < DOA_MEASURED_ANGLE_NUM; i++)
        {
            if (T[direction] < T[i]) 
            {
                direction = i;
            }
        }
        evt.evt_id = DOA_DIRECTION_ESTIMATED;
        evt.param.direction_info.direction = direction * DOA_ANGULAR_PRECISION;
        evt.param.direction_info.T = T;
        evt.param.direction_info.size = DOA_MEASURED_ANGLE_NUM;
        doa->evt_handler(&evt);
   }
}

static void doa_set_gain_sample(doa_t* doa, float rss)
{
    doa_evt_t evt;
    //NRF_LOG_DEBUG("doa_set_gain_sample: %d angle == %d, config == %d ", rss,  doa->measured_angle, doa->antenna.configuration);
    doa->gain_samples[0][doa->measured_angle][doa->antenna.configuration] = rss;

    if (antenna_set_next_configuration(&doa->antenna))
    {

        evt.evt_id = DOA_ANGLE_MEASURED;
        evt.param.calibration_samples_per_angle.angle      = doa->measured_angle * DOA_ANGULAR_PRECISION;
        evt.param.calibration_samples_per_angle.configs_num = ANTENNA_CONFIGS_NUM;
        evt.param.calibration_samples_per_angle.samples    = doa->gain_samples[0][doa->measured_angle];
        doa->evt_handler(&evt);
        if (doa->measured_angle == DOA_MEASURED_ANGLE_NUM - 1)
        {
            doa->calibrated = true;
            evt.evt_id = DOA_CALIBRATION_END;
            evt.param.calibration_successful = true;
            doa->evt_handler(&evt);
        }
        else if (doa->manual_angle_setup)
        {
            doa->angle_set  = false;
            evt.evt_id      = DOA_ANGLE_SETUP_REQUIRED;
            evt.param.angle = doa->measured_angle * DOA_ANGULAR_PRECISION;
            doa->evt_handler(&evt);
        }
        else
        {
            doa_inc_angle(doa);
        }
    }
}

void doa_start_calibration(doa_t* doa, bool manual_angle_setup)
{
    antenna_set_configuration(&doa->antenna, 0);
    doa->measured_angle     = 0;
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
    if (doa->measured_angle < DOA_MEASURED_ANGLE_NUM - 1)
    {
        doa->measured_angle++;
        doa->angle_set = true;
    }

    return doa->measured_angle * DOA_ANGULAR_PRECISION;
}
uint16_t doa_dec_angle(doa_t* doa)
{
    if (doa->measured_angle > 0)
    {
        doa->measured_angle--;
        doa->angle_set = true;
    }
    return doa->measured_angle * DOA_ANGULAR_PRECISION;
}

void doa_start(doa_t* doa)
{
    doa->angle_set = true;
}
void doa_estimate(doa_t* doa)
{
    doa->estimate = true;
}
static int8_t median(uint8_t size, int8_t array[]) 
{
    int8_t temp;
    uint8_t i, j;
    // simple bubble sort
    for(i = 0; i < size - 1; i++)
    {
        for(j= i + 1; j < size; j++)
        {
            if(array[j] < array[i])
            {
                temp = array[i];
                array[i] = array[j];
                array[j] = temp;
            }
        }
    }
    return array[size/2];
}

static float mean(uint8_t size, int8_t array[])
{
    int32_t sum_rss = 0;
    for(uint8_t i = 0; i < size; i++)
        sum_rss += array[i];

    return ((float)sum_rss) / size;
}