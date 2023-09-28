#include "sensor_reading.h"

#include "main.h" // for hadc1

static uint16_t ADC_reading_to_mV(uint16_t a) {
    static const uint16_t Vref_mV = 3300, max_ADC_reading = 1 << 12;
    return a * Vref_mV / max_ADC_reading;
}

static int16_t hall_sensor_reading_mV_to_magnetic_field_Gs(uint16_t a) {
	//2500mV is the quiescent output voltage, which is the voltage the sensor outputs when no magnetic field is detected (=> 0GS)
	//1.6 is the output voltage sensitivity, measured in mV/Gs. it correlates the strength of the magnetic field with the voltage output of the sensor
	return ((int16_t)a - 2500) * 10 / 16;
}

int16_t read_hall_sensor_Gs() {
	uint16_t hall_sensor_reading = ADC_DMA_buffer[0];
	uint16_t hall_sensor_voltage_mV = ADC_reading_to_mV(hall_sensor_reading);
	int16_t magnetic_field = hall_sensor_reading_mV_to_magnetic_field_Gs(hall_sensor_voltage_mV);

	return magnetic_field;
}

uint16_t read_sys_voltage_mV() {
	uint16_t ADC_reading = ADC_DMA_buffer[1];
	uint16_t sys_voltage_mV = ADC_reading_to_mV(ADC_reading);

	return sys_voltage_mV;
}
