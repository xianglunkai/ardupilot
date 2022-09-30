#pragma once

#include <termios.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <poll.h>
#include <AP_Vehicle/AP_Vehicle.h>

namespace Imx_K60{

#define BSKLINK_MSG_HEAD_1    0xDE
#define BSKLINK_MSG_HEAD_2    0xED
#define BSKLINK_DEVICE_ID     0x02
#define BSKLINK_SYS_ID        0x00
#define BSKLINK_MAX_PAYLOAD_LENGTH 64

#define PACKET_HEADER 0x11

#define PACKET_SEND_ADC_VALUE 0x81
#define PACEET_RECV_SET_PWM_FREQ 101
#define PACKET_RECV_SET_PWM_DUTY 102
#define PACKET_RECV_SET_IO 103
#define MAX_AI_CHN_NUM 12

struct adc_value_s
{
	uint16_t root;           // 0: others 1: ARM 2: rc
	uint8_t PORT_armed_state;   
    uint8_t PORT_ems_state;    
    int16_t PORT_speed;         
    
    uint8_t STBD_armed_state;   
    uint8_t STBD_ems_state;    
    int16_t STBD_speed;   

    uint16_t AIVal[MAX_AI_CHN_NUM];   
    uint16_t DIVal;   

#if HAL_BORAD_MCU_V2 == 1
	uint16_t BMS_Total_V;			//单位10mV
	int16_t  BMS_Total_C;			//单位10mA 充电为正,放电为负
	uint16_t BMS_Remnant_Capacity;	//剩余容量
	uint16_t Reserve[5];
#endif
}__attribute__((packed));


enum class ADC_Channel:unsigned int{
	control_root = 0,
	rpm_left  = 1,
	rpm_right = 2,
	temperature  = 3,
	voltage_battery = 4,
	current_battery = 5,
};

struct adc_report_s
{
    uint8_t id;
    float data;
};

class Imx_IOMCU
{
public:
	int fd;

	Imx_IOMCU()
	{
		fd = -1;
		buf_used = 0;
	}
	virtual ~Imx_IOMCU()
	{
		if (fd != -1)
		{
			close(fd);
			fd = -1;
			buf_used = 0;
			poll_fd.fd = -1;
		}
	}
	int init(const char *uart_name, unsigned int baud = 115200)
	{
		fd = open(uart_name, O_RDWR | O_NOCTTY);
		if (fd < 0) {
			printf("Message : %s\n", strerror(errno));
			return fd;
		}
		if (set_uart_baudrate(fd, baud) <= 0)
		{
			printf("Message : %s\n", strerror(errno));
			close(fd);
			fd = -1;
			return -1;
		}
		poll_fd.events = POLLIN;
		poll_fd.fd = fd;
		return fd;
	}
	bool set_freq(uint32_t chmask, uint32_t freq[12])
	{

		
		uint32_t packet[13] = { 0 };
		char packet2[128];
		packet[0] = chmask;
		for (int i = 0; i < 12; i++)
		{
			packet[i + 1] = freq[i];
		}

 		//set_io(0XED,0XDE);	

		return uart_send_packet(fd, PACEET_RECV_SET_PWM_FREQ, packet, packet2, 13 * 4);
	}
	bool set_duty(uint32_t chmask, uint32_t duty[12])
	{
		uint32_t packet[13] = { 0 };
		char packet2[128];
		packet[0] = chmask;
		//printf("chmask duty = %0x\n",chmask);
		for (int i = 0; i < 12; i++)
		{
			packet[i + 1] = duty[i];
		}

		return uart_send_packet(fd, PACKET_RECV_SET_PWM_DUTY, packet, packet2, 13 * 4);
	}
		bool set_io(uint32_t chmask,uint32_t packet_io)
	{
		uint32_t packet[2] = { 0 };
		char packet2[4];
		packet[0] = chmask;
		packet[1] =packet_io;

		return uart_send_packet(fd, PACKET_RECV_SET_IO, packet, packet2, 2 * 4);
	}

	bool read_adc(adc_report_s *report,unsigned len)
	{
	
		int poll_ret = poll(&poll_fd, 1, 0);
		if (poll_ret <= 0)
			return false;

		int read_len = read(fd, &uart_buf[buf_used], 4096 - buf_used);
		if (read_len <= 0)
		{
			return false;
		}
		buf_used += read_len;

		uint8_t packet_id;
		uint8_t packet[4096];
		uint8_t packet_size;
		packet[0] = 0;
		if (recv_packet(uart_buf, &buf_used, &packet_id, packet, &packet_size) == false)
		{
			return false;
		}
		if (PACKET_SEND_ADC_VALUE != packet_id)
			return false;

		adc_value_s *padc_value = (adc_value_s *)packet;

	#if HAL_BORAD_MCU_V2 == 1
		if(len < 1 || len > 24){return false;}

		// manul fill 
		report[0].data = padc_value->root; 
		report[0].id = 0;
		report[1].data = padc_value->PORT_speed; 
		report[1].id = 1;
		report[2].data = padc_value->STBD_speed;
		report[2].id = 2;
		report[3].data =padc_value->AIVal[10]*0.01;// temperature 0.01;
		report[3].id = 3;
		report[4].data = padc_value->AIVal[7]*0.01; //DC24v
		report[4].id = 4;

		report[5].data = padc_value->AIVal[8]*0.01; //DC3.3
		report[5].id = 5;

		report[6].data = padc_value->AIVal[9]*0.01; //DC4.2
		report[6].id = 6;

		report[7].data = padc_value->AIVal[0]*0.01; //reference voltage
		report[7].id = 7;

		report[8].data = padc_value->AIVal[1]*0.01; //AIN2
		report[8].id = 8;
		report[9].data = padc_value->AIVal[2]*0.01; //AIN1
		report[9].id = 9;
		report[10].data = padc_value->AIVal[3]*0.01; //AIN4
		report[10].id = 10;
		report[11].data = padc_value->AIVal[4]*0.01; //AIN3
		report[11].id = 11;
		report[12].data = padc_value->AIVal[5]*0.1; //Rt2
		report[12].id = 12;	
		report[13].data = padc_value->AIVal[6]*0.1; //Rt1
		report[13].id = 13;	
		report[14].data = padc_value->AIVal[11]*0.01; //AD Reserve
		report[14].id = 14;	

		report[15].data = padc_value->DIVal; //DI
		report[15].id = 15;	

		report[16].data = padc_value->BMS_Total_V*0.01; //
		report[16].id = 16;	
		report[17].data = padc_value->BMS_Total_C*0.01; //
		report[17].id = 17;	
		report[18].data = padc_value->BMS_Remnant_Capacity*0.01; //
		report[18].id = 18;	
		report[19].data = padc_value->Reserve[0]*0.01; //Reserve
		report[19].id = 19;	
		report[20].data = padc_value->Reserve[1]*0.01; //
		report[20].id = 20;	
		report[21].data = padc_value->Reserve[2]*0.01; //
		report[21].id = 21;	
		report[22].data = padc_value->Reserve[3]*0.01; //
		report[22].id = 22;	
		report[23].data = padc_value->Reserve[4]*0.01; //
		report[23].id = 23;	
	#else
		float temperature = 4096000/padc_value->AIVal[10]-1000;
		temperature = -0.00004*temperature*temperature+0.2118*temperature-147.53;
		float voltage_battery  = padc_value->AIVal[6]*36.3/4096.0+0.3;

		if(len < 1 || len > 6){return false;}

		// manul fill 
		report[0].data = padc_value->root; 
		report[0].id = 0;
		report[1].data = padc_value->PORT_speed; 
		report[1].id = 1;
		report[2].data = padc_value->STBD_speed;
		report[2].id = 2;
		report[3].data = temperature;
		report[3].id = 3;
		report[4].data = voltage_battery; 
		report[4].id = 4;
		report[5].data = 0; 
		report[5].id = 5;
	#endif

		// siwtch to manual mode if K60 get the control root
		if (padc_value->root == 2) {
			AP::vehicle()->set_mode(0, ModeReason::RC_COMMAND);
		}
		
		return true;
	}

	bool read_adc(int32_t *adc_pc0_data, int32_t *adc_pc1_data)
	{
		float temperature=0;
		float voltage_battery;
		int poll_ret = poll(&poll_fd, 1, 0);
		if (poll_ret <= 0)
			return false;

		int read_len = read(fd, &uart_buf[buf_used], 4096 - buf_used);
		if (read_len <= 0)
		{
			return false;
		}
		buf_used += read_len;

		uint8_t packet_id;
		uint8_t packet[4096];
		uint8_t packet_size;
		packet[0] = 0;
		if (recv_packet(uart_buf, &buf_used, &packet_id, packet, &packet_size) == false)
		{
			return false;
		}
		if (PACKET_SEND_ADC_VALUE != packet_id)
			return false;

		adc_value_s *padc_value = (adc_value_s *)packet;

		temperature		 = 4096000/padc_value->AIVal[10]-1000;
		temperature 	 = -0.00004*temperature*temperature+0.2118*temperature-147.53;

		voltage_battery  = padc_value->AIVal[6]*36.3/4096.0+0.3;

		*adc_pc0_data   =  voltage_battery;
		*adc_pc1_data   =  temperature;

		return true;
	}

private:
	uint8_t uart_buf[4096];
	int buf_used;
	struct pollfd poll_fd;

	int set_uart_baudrate(const int _fd, unsigned int baud)
	{
		int speed;
		switch (baud) {
		case 9600:   speed = B9600;   break;
		case 19200:  speed = B19200;  break;
		case 38400:  speed = B38400;  break;
		case 57600:  speed = B57600;  break;
		case 115200: speed = B115200; break;
		case 230400: speed = B230400; break;
		default:
			return -EINVAL;
		}

		struct termios uart_config;

		int termios_state;

		tcgetattr(_fd, &uart_config);

		uart_config.c_cflag |= (CLOCAL | CREAD);
		uart_config.c_cflag &= ~PARENB;
		uart_config.c_cflag &= ~CSTOPB;
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;
		uart_config.c_cflag &= ~CRTSCTS;

		uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

		uart_config.c_iflag = 0;

		uart_config.c_oflag = 0;

		uart_config.c_cc[VTIME] = 0;
		uart_config.c_cc[VMIN] = 1;

		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			return 0;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			return 0;
		}

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			return 0;
		}

		return 1;
	}

	bool uart_send_packet(int _fd, uint8_t packet_id, void *buffer, void *buffer2, int size)
	{
		if (size <= 0)
			return false;

		int newbuf_len = size  + 2 + 2 + 2 ;
		uint8_t *oldbuf = (uint8_t *)buffer;
		uint8_t *newbuf = (uint8_t *)buffer2;
		newbuf[0] = BSKLINK_MSG_HEAD_1;
		newbuf[1] = BSKLINK_MSG_HEAD_2;
		newbuf[2] = BSKLINK_DEVICE_ID;
		newbuf[3] = BSKLINK_SYS_ID;
		newbuf[4] = packet_id;
		newbuf[5] = (uint8_t)size;
		int i;
		uint8_t mask = oldbuf[0];
		for (i = 1; i < size; i++)
		{
			mask = mask ^ oldbuf[i];
		}

		for (i = 0; i < size; i++)
		{
			newbuf[i  + 6] = oldbuf[i];
			
		}

		int write_bytes = write(_fd, newbuf, newbuf_len);
		if (write_bytes != newbuf_len)
			return false;

		return true;
	}

	bool recv_packet(uint8_t *_uart_buf, int *_buf_used, uint8_t *packet_id, uint8_t *packet, uint8_t *packet_size)
	{
		int i;
		bool first_hit = false;
		bool found_header = false;
		for (i = 0; i < *_buf_used; i++)
		{
			if (_uart_buf[i] == BSKLINK_MSG_HEAD_1 /*||_uart_buf[i] == BSKLINK_MSG_HEAD_2*/)
			{
				if (first_hit == false&&_uart_buf[i] == BSKLINK_MSG_HEAD_1)
				{
					first_hit = true;
					continue; 
				}
			}else if(first_hit == true&&_uart_buf[i] == BSKLINK_MSG_HEAD_2)
			{
				found_header = true;
				i--;
				break;
			}
			else
			{
				if (first_hit == true)
					first_hit = false;
			}
		}

		if (found_header)
		{
			memmove(_uart_buf, _uart_buf + i, *_buf_used - i);
			*_buf_used = *_buf_used - i;
		}
		else
		{
			printf("ACM data miss header!\n");
			return false;
		}

		*packet_id = _uart_buf[4];
		*packet_size = _uart_buf[5];
		if (*_buf_used < *packet_size  + 2 + 2 + 2 )
		{
			return false;
		}
		for (i = 0; i < *packet_size; i++)
		{
			packet[i] = _uart_buf[6 + i ];
		}

		memmove(_uart_buf, _uart_buf + *packet_size + 6, *_buf_used - (*packet_size + 6));
		*_buf_used = *_buf_used - (*packet_size  + 6);

		return true;
	}

};
}