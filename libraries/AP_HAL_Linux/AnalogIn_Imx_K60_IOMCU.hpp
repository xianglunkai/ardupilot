#pragma once

#include "AP_HAL_Linux.h"
#include "Imx_K60_IOMCU.hpp"

extern const AP_HAL::HAL &hal;

#if HAL_BORAD_MCU_V2 == 1
#define IMXK60_ADC_MAX_CHANNELS 24
#else
#define IMXK60_ADC_MAX_CHANNELS 6
#endif

class AnalogSource_Imx_K60_IOMCU : public AP_HAL::AnalogSource {
public:
	friend class AnalogIn_Imx_K60_IOMCU;
	AnalogSource_Imx_K60_IOMCU(int16_t pin)
	{
		_pin = pin;
		_value = 0;
	}
	float read_average() override
	{
		return read_latest();
	}
	float read_latest() override
	{
		return _value;
	}
	bool set_pin(uint8_t pin) override
	{
		if (_pin == pin) {
       		 return true;
   		 }
    	_pin = pin;
   		return true;
	}
	void set_stop_pin(uint8_t p) {}
	void set_settle_time(uint16_t settle_time_ms) {}
	float voltage_average() override
	{
		return _value;
	}
	float voltage_latest() override
	{
		return _value;
	}
	float voltage_average_ratiometric() override
	{
		return _value;
	}
private:
	int16_t _pin;
	float _value;
};

class AnalogIn_Imx_K60_IOMCU : public AP_HAL::AnalogIn {
public:
	AnalogIn_Imx_K60_IOMCU(Imx_K60::Imx_IOMCU *m)
	{
		mcu = m;
	}

	void init() override
	{
		hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AnalogIn_Imx_K60_IOMCU::_update, void));
	}
	
	AP_HAL::AnalogSource *channel(int16_t pin) override
	{
		 WITH_SEMAPHORE(_semaphore);

		if(pin >= 0 && pin < IMXK60_ADC_MAX_CHANNELS) {
			if(_channels[pin] == nullptr) {
				_channels[pin] = new AnalogSource_Imx_K60_IOMCU(pin);
				return _channels[pin];
			}
		}

		hal.console->printf("Out of analog channels\n");
		return nullptr;
	}

	/* Board voltage is not available */
	float board_voltage() override { return 5.0f; }

private:
	void _update()
	{
		if (AP_HAL::micros() - _last_update_timestamp < 50000) {
			return;
		}

		Imx_K60::adc_report_s reports[IMXK60_ADC_MAX_CHANNELS];

		if(mcu->read_adc(reports, IMXK60_ADC_MAX_CHANNELS)) {
			for(uint8_t i =0;i < IMXK60_ADC_MAX_CHANNELS; i++) {
				AnalogSource_Imx_K60_IOMCU *source = _channels[i];
				if(source != nullptr){
					uint8_t pin = source->_pin;
					source->_value = reports[pin].data;
				}
			}
		}
		

		 _last_update_timestamp = AP_HAL::micros();
	}

	AnalogSource_Imx_K60_IOMCU *_channels[IMXK60_ADC_MAX_CHANNELS];
	uint32_t _last_update_timestamp;
	Imx_K60::Imx_IOMCU *mcu;
	HAL_Semaphore _semaphore;
};
