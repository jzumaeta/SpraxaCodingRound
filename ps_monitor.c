#include "ps_monitor.h"
#include "app_scheduler.h"
#include "nrf_log.h"
#include "nrf_drv_saadc.h"
#include "nrf_gpio.h"


#define SAADC_SAMPLES_IN_BUFFER       1
#define SAADC_CALIBRATION_INTERVAL    5 

static nrf_saadc_value_t              m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t                       m_adc_evt_counter = 0;
static bool                           m_saadc_calibrate = false;


extern void update_ble_adv_ps_value(int ps_value);

//prototype functions
void saadc_callback(nrf_drv_saadc_evt_t const * p_event);
void psmonitor_resultEventHandler(void *p_event_data, uint16_t event_size);


void saadc_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config;       

    //Initialize SAADC
    saadc_config.low_power_mode = true;                                                   //Enable low power mode
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;                             
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    //Configure SAADC channel
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config.pin_p = NRF_SAADC_INPUT_AIN1;                                          //Select the input pin for the channel. AIN1 pin maps to physical pin P0.03.
    channel_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

void saadc_deinit(void)
{
    nrf_drv_saadc_uninit();
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    ret_code_t err_code;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)                                //Capture offset calibration complete event
    {
        if((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0)               //Evaluate if offset calibration should be performed. Configure the SAADC_CALIBRATION_INTERVAL constant to change the calibration frequency
        {
            m_saadc_calibrate = true;                                           // Set flag to trigger calibration in main context when SAADC is stopped
        }

        //for (int i = 0; i < p_event->data.done.size; i++)
        //{
        //    NRF_LOG_INFO("%d\r\n", p_event->data.done.p_buffer[i]);             //Print the SAADC result on UART
        //}     
        
        if(m_saadc_calibrate == false)
        {
            err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again. 
            APP_ERROR_CHECK(err_code);
        }

        //get ADC value and schedule it...
        int adcValue = p_event->data.done.p_buffer[0];
        app_sched_event_put(&adcValue, sizeof(adcValue), psmonitor_resultEventHandler);
        
        m_adc_evt_counter++;

        //deinit SAADC        
        saadc_deinit();
  
    }
    else if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE)
    {
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);             //Set buffer so the SAADC can write to it again. 
        APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);             //Need to setup both buffers, as they were both removed with the call to nrf_drv_saadc_abort before calibration.
        APP_ERROR_CHECK(err_code);
        
        NRF_LOG_INFO("SAADC calibration complete ! \r\n");                                              //Print on UART
    }
}


void psmonitor_resultEventHandler(void *p_event_data, uint16_t event_size)
{
    int adcResult = *((int*)p_event_data);
    //note: Vin = ADCresult * Reference / (Resolution * Gain)
    double Vin_mV = ((adcResult*0.6)/(4095*1/6))*1000;//x1000 to get millivolts

    //NRF_LOG_INFO("PS Voltage: %dmV", Vbat_mV); 
    NRF_LOG_INFO("Vin: %d mV", Vin_mV); 
    
    //send data...
    update_ble_adv_ps_value((uint16_t) Vin_mV);
}

void saadc_sampling_trigger(void)
{
    ret_code_t err_code;
    //Note: Event handler is called immediately after conversion is finished.
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
}


void psMonitor_startReading(void)
{
    //init SAADC
    saadc_init();
    //start sampling (one sample)
    saadc_sampling_trigger();
}