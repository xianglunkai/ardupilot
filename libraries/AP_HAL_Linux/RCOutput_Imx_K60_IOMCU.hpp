#pragma once

#include "AP_HAL_Linux.h"
#include "Imx_K60_IOMCU.hpp"

namespace Linux
{
	class RCOutput_Imx_K60_IOMCU : public AP_HAL::RCOutput
	{
	public:
		RCOutput_Imx_K60_IOMCU(Imx_K60::Imx_IOMCU *m)
		{
			mcu = m;
		}

		~RCOutput_Imx_K60_IOMCU()
		{

		}
		void init() override
		{
			set_freq(0xFF, 50);
		}
		void set_freq(uint32_t chmask, uint16_t freq_hz) override
		{
			if (freq_hz > 400)
				freq_hz = 400;
			for (int i = 0; i < 12; i++)
			{
				if (chmask & (1u << i))
				{
					all_freq[i] = freq_hz;
				}
			}
			mcu->set_freq(chmask, all_freq);
		}
		uint16_t get_freq(uint8_t ch) override
		{
			return all_freq[ch];
		}
		void enable_ch(uint8_t ch) override
		{

		}
		void disable_ch(uint8_t ch) override
		{
			// write(ch, 1);
		}
		void write(uint8_t ch, uint16_t period_us) override
		{
			if (ch >= 12)
				return;

			_pulses_buffer[ch] = period_us;
			_pending_write_mask |= (1U << ch);

			if (!_corking) {
				_corking = true;
				push();
			}
		}
		void cork() override
		{
			_corking = true;
		}
		void push() override
		{
			if (!_corking) {
				return;
			}
			_corking = false;

			if (_pending_write_mask == 0)
				return;

			mcu->set_duty(_pending_write_mask, _pulses_buffer);

			_pending_write_mask = 0;
		}
		uint16_t read(uint8_t ch) override
		{
			return _pulses_buffer[ch];
		}
		void read(uint16_t *period_us, uint8_t len) override
		{
			for (int i = 0; i < len; i++)
			{
				period_us[i] = read(i);
			}
		}

	private:
		Imx_K60::Imx_IOMCU *mcu;
		uint32_t _pulses_buffer[12];
		uint32_t all_freq[12];
		bool _corking = false;
		uint32_t _pending_write_mask;
	};
}
