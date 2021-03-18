/*
 * custom_pin_map.c
 *
 * Created: 2020-09-17 오전 10:51:27
 *  Author: rtlab
 */ 

#include <custom_pin_map.h>
#include <asf.h>

void pin_configure(void){
	struct port_config pin_conf;

	port_get_config_defaults(&pin_conf);
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;

	port_pin_set_config(SOL1_EN, &pin_conf);
	port_pin_set_config(SOL2_EN, &pin_conf);
	port_pin_set_config(SOL3_EN, &pin_conf);
	
	
	port_pin_set_config(SENSOR_1_PWR_EN, &pin_conf);
	port_pin_set_output_level(SENSOR_1_PWR_EN, false);
	port_pin_set_config(SENSOR_2_PWR_EN, &pin_conf);
	port_pin_set_output_level(SENSOR_2_PWR_EN, false);
	port_pin_set_config(SENSOR_3_PWR_EN, &pin_conf);
	port_pin_set_output_level(SENSOR_3_PWR_EN, false);
	port_pin_set_config(SENSOR_4_PWR_EN, &pin_conf);
	port_pin_set_output_level(SENSOR_4_PWR_EN, false);
}
