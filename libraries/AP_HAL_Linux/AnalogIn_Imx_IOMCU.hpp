#pragma once

#include "AP_HAL_Linux.h"
#include "Imx_IOMCU.hpp"

extern const AP_HAL::HAL &hal;

class AnalogSource_Imx_IOMCU : public AP_HAL::AnalogSource {
public:
	friend class AnalogIn_Imx_IOMCU;
	AnalogSource_Imx_IOMCU(int16_t pin)
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

class AnalogIn_Imx_IOMCU : public AP_HAL::AnalogIn {
public:
	AnalogIn_Imx_IOMCU(Imx_IOMCU *m)
	{
		mcu = m;
	}

	void init() override
	{
		hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AnalogIn_Imx_IOMCU::_update, void));
	}
	
	AP_HAL::AnalogSource *channel(int16_t n) override
	{
		for (uint8_t j = 0; j < 2; j++) {
			if (_channels[j] == nullptr) {
				_channels[j] = new AnalogSource_Imx_IOMCU(n);
				return _channels[j];
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

		int adc_pc0_data;
		int adc_pc1_data;

		if (mcu->read_adc(&adc_pc0_data, &adc_pc1_data))
		{
			for (size_t i = 0; i < 2; i++) {
				AnalogSource_Imx_IOMCU *source = _channels[i];

				if (source != nullptr)
				{
					if (source->_pin == 0)
					{
						source->_value = adc_pc0_data * 3.3 / 4096.0;
					}
					else if (source->_pin == 1)
					{
						source->_value = adc_pc1_data * 3.3 / 4096.0;
					}
				}
			}

			_last_update_timestamp = AP_HAL::micros();
		}
	}

	AnalogSource_Imx_IOMCU *_channels[2];
	uint32_t _last_update_timestamp;
	Imx_IOMCU *mcu;
};
