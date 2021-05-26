/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _DT_BINDINGS_MAXIM_MAX11300_H
#define _DT_BINDINGS_MAXIM_MAX11300_H


#define	PORT_MODE_0	0
#define	PORT_MODE_1	1
#define	PORT_MODE_2	2
#define	PORT_MODE_3	3
#define	PORT_MODE_4	4
#define	PORT_MODE_5	5
#define	PORT_MODE_6	6
#define	PORT_MODE_7	7
#define	PORT_MODE_8	8
#define	PORT_MODE_9	9
#define	PORT_MODE_10	10
#define	PORT_MODE_11	11
#define	PORT_MODE_12	12


#define ADC_SAMPLES_1 	0
#define	ADC_SAMPLES_2	1
#define	ADC_SAMPLES_4	2
#define	ADC_SAMPLES_8	3
#define	ADC_SAMPLES_16	4
#define	ADC_SAMPLES_32	5
#define	ADC_SAMPLES_64	6
#define ADC_SAMPLES_128	7

/* ADC voltage ranges */
#define	ADC_VOLTAGE_RANGE_NOT_SELECTED	0	
#define	ADC_VOLTAGE_RANGE_PLUS10	1  // 0 to +5V range
#define	ADC_VOLTAGE_RANGE_PLUSMINUS5	2  // -5V to +5V range
#define	ADC_VOLTAGE_RANGE_MINUS10	3  // -10V to 0 range
#define	ADC_VOLTAGE_RANGE_PLUS25	4  // 0 to +2.5 range

/* DAC voltage ranges mode 5*/
#define	DAC_VOLTAGE_RANGE_NOT_SELECTED	0 	
#define	DAC_VOLTAGE_RANGE_PLUS10	1 	
#define	DAC_VOLTAGE_RANGE_PLUSMINUS5	2	
#define	DAC_VOLTAGE_RANGE_MINUS10	3 

#endif /* _DT_BINDINGS_MAXIM_MAX11300_H */
