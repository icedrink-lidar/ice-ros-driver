/*
																																										 
			_/_/_/    _/_/_/  _/_/_/_/  _/_/_/    _/_/_/    _/_/_/  _/      _/  _/    _/   
			 _/    _/        _/        _/    _/  _/    _/    _/    _/_/    _/  _/  _/      
			_/    _/        _/_/_/    _/    _/  _/_/_/      _/    _/  _/  _/  _/_/         
	               _/    _/        _/        _/    _/  _/    _/    _/    _/    _/_/  _/  _/        
	            _/_/_/    _/_/_/  _/_/_/_/  _/_/_/    _/    _/  _/_/_/  _/      _/  _/    _/       
																																																																																					
		    Copyright (C) 2017                          Beijing IceDrink Technology Co.,Ltd.
*/
#include <ice_laser_driver/ice_laser.h>

namespace ice_laser_driver
{
	IceLaser::IceLaser(const std::string &port, uint32_t baud_rate, boost::asio::io_service &io) : port_(port),
	baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
	{
		serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
	}

	void IceLaser::poll(sensor_msgs::LaserScan::Ptr scan)
	{
		uint8_t temp_char, init_level = 0;
		uint16_t idx, num_of_deg, i;
		bool got_scan = false;
		boost::array<uint8_t, 4> raw_bytes;
		boost::array<uint16_t, 360> data_bytes;
		while (!shutting_down_ && !got_scan)
		{
			if (init_level == 0)
			{
				boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
				if (temp_char == 0xFF)
				{
					init_level = 1;
				}
				else
				{
					init_level = 0;
				}
			}
			else if (init_level == 1)
			{
				boost::asio::read(serial_, boost::asio::buffer(&raw_bytes, 4));
				idx = raw_bytes[0] << 8 | raw_bytes[1];
				if (idx == 0)
				{
					scan->angle_min = 0;
					scan->angle_max = 2.0 * M_PI;
					scan->angle_increment = (2.0 * M_PI / (num_of_deg * 1.0));
					scan->time_increment = 1 / 1e8;
					scan->range_min = 0.15;
					scan->range_max = 10.0;
					scan->ranges.reserve(num_of_deg);
					if (num_of_deg > 300)
					{
						for (i = 0; i < num_of_deg; i++)
						{
							scan->ranges.push_back(data_bytes[i] / 100.0);
						}
					}
					got_scan = true;
				}
				if (idx >= 0 && idx < 360)
				{
					data_bytes[idx] = raw_bytes[2] << 8 | raw_bytes[3];
					num_of_deg = idx + 1;
					init_level = 0;
				}
				else
				{
					init_level = 0;
				}
			}
			else
			{
				init_level = 0;
			}
		}
	}
};
