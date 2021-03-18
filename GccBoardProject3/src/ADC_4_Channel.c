/*
 * ADC_4_Channel.c
 *
 * Created: 2020-04-30 오후 10:25:57
 *  Author: khs
 */ 

// adc.h 참고

#include <ADC_4_Channel.h>

void config_adc_func(void){
	struct adc_config config_adc;
	struct port_config pin_conf;

	adc_get_config_defaults(&config_adc);
	config_adc.positive_input = SEN1;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
	config_adc.reference = ADC_REFERENCE_INTVCC0;	// 2.027
	config_adc.resolution = ADC_RESOLUTION_12BIT;
	config_adc.gain_factor = ADC_GAIN_FACTOR_DIV2;
	config_adc.pin_scan.offset_start_scan = 0;
	config_adc.pin_scan.inputs_to_scan = 4;
	// use 4 channel
	
	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);
}
void sensor_on(void){
	port_pin_set_output_level(SENSOR_1_PWR_EN, true);
	port_pin_set_output_level(SENSOR_2_PWR_EN, true);
	port_pin_set_output_level(SENSOR_3_PWR_EN, true);
	port_pin_set_output_level(SENSOR_4_PWR_EN, true);	
}
void sensor_off(void){
	port_pin_set_output_level(SENSOR_1_PWR_EN, false);
	port_pin_set_output_level(SENSOR_2_PWR_EN, false);
	port_pin_set_output_level(SENSOR_3_PWR_EN, false);
	port_pin_set_output_level(SENSOR_4_PWR_EN, false);	
}
void get_sen_data(uint16_t* ary_4){
	for(uint8_t i = 0; i < 4; i++){
		adc_start_conversion(&adc_instance);
		do {} while (adc_read(&adc_instance, ary_4 + i) == STATUS_BUSY);
		ary_4[i] = ary_4[i] * VREF_mV / RESOLUTION_ADC * REVERSE_GAIN;
	}
}
void disable_adc(struct adc_module* adc_instance){
	adc_disable(adc_instance);
}