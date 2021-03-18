/*
 * custom_pin_map.h
 *
 * Created: 2020-04-09 오후 2:04:15
 *  Author: khs
 */ 


#ifndef CUSTOM_PIN_MAP_H_
#define CUSTOM_PIN_MAP_H_


#define	BAT_ADC_IN			PIN_PA05		// AIN[5]
#define BAT_ADC_EN			PIN_PA06

#define DETECT_24V			PIN_PA06

#define	SENSOR_1_IN			PIN_PA03		// AIN[1]
#define	SENSOR_2_IN			PIN_PB08		// AIN[2]
#define	SENSOR_3_IN			PIN_PB09		// AIN[3]
#define	SENSOR_4_IN			PIN_PA04		// AIN[4]
#define SENSOR_1_PWR_EN		PIN_PB01
#define SENSOR_2_PWR_EN		PIN_PB02
#define SENSOR_3_PWR_EN		PIN_PB03
#define SENSOR_4_PWR_EN		PIN_PB04

#define SOL1_EN				PIN_PA08
#define SOL2_EN				PIN_PA09
#define SOL3_EN				PIN_PA10

#define UART_RX				PIN_PB11
#define UART_TX				PIN_PB10

#define TX_SERCOM_PAD		PINMUX_PB10D_SERCOM4_PAD2
#define RX_SERCOM_PAD		PINMUX_PB11D_SERCOM4_PAD3


// Analog Input Index - It's index list for ADC functions. it,s not pin number list.
#define SEN1				ADC_POSITIVE_INPUT_PIN1
#define SEN2				ADC_POSITIVE_INPUT_PIN2
#define SEN3				ADC_POSITIVE_INPUT_PIN3
#define SEN4				ADC_POSITIVE_INPUT_PIN4
#define BAT_SEN				ADC_POSITIVE_INPUT_PIN5

void pin_configure(void);

#endif /* CUSTOM_PIN_MAP_H_ */