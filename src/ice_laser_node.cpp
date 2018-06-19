/*
																																										 
			_/_/_/    _/_/_/  _/_/_/_/  _/_/_/    _/_/_/    _/_/_/  _/      _/  _/    _/   
			 _/    _/        _/        _/    _/  _/    _/    _/    _/_/    _/  _/  _/      
			_/    _/        _/_/_/    _/    _/  _/_/_/      _/    _/  _/  _/  _/_/         
	               _/    _/        _/        _/    _/  _/    _/    _/    _/    _/_/  _/  _/        
	            _/_/_/    _/_/_/  _/_/_/_/  _/_/_/    _/    _/  _/_/_/  _/      _/  _/    _/       
																																																																																					
		    Copyright (C) 2017                          Beijing IceDrink Technology Co.,Ltd.
*/
#include <ice_laser_driver/ice_laser.h>
#include <ros/console.h>

namespace ice_laser_driver
{
	IceLaser::IceLaser(const std::string &port, uint32_t baud_rate, boost::asio::io_service &io) : port_(port),
	baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
	{
		serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
	}

	void IceLaser::poll(sensor_msgs::LaserScan::Ptr scan)
	{
		uint8_t temp_char, index;
		uint8_t angle_resolution, point_resolution;
		uint8_t start_count = 0;
		bool got_scan = false;
		boost::array<uint8_t, 160> raw_bytes;
		boost::array<uint16_t, 1440> data_bytes;
		boost::array<uint8_t, 5> opt_bytes;
		while (!shutting_down_ && !got_scan)
		{
			// Wait until the start sequence 0x5A, 0xA5, 0x00, 0xC0 comes around
			boost::asio::read(serial_, boost::asio::buffer(&temp_char,1));
			if(start_count == 0) {
			if(temp_char == 0x49) {
				start_count = 1;
			}
			} else if(start_count == 1) {
			if(temp_char == 0x63) {
				start_count = 2;
			}
			} else if(start_count == 2) 
			{
				if(temp_char == 0x65) 
				{
					// Now that entire start sequence has been found, read in the rest of the message
					boost::asio::read(serial_,boost::asio::buffer(&angle_resolution,1));
					boost::asio::read(serial_,boost::asio::buffer(&point_resolution,1));
					boost::asio::read(serial_,boost::asio::buffer(&index,1));
					boost::asio::read(serial_,boost::asio::buffer(&temp_char,1));
					boost::asio::read(serial_,boost::asio::buffer(&temp_char,1));
					// Read in 360*4 = 1440 chars for each point
					//boost::asio::read(serial_,boost::asio::buffer(&raw_bytes,1440));
					
					if(point_resolution == 80)
					{
						if (index >= 0 && index <= 17)
						{
							boost::asio::read(serial_, boost::asio::buffer(&raw_bytes, 160));
							for(uint8_t i=0;i<80;i++)
							{
								data_bytes[index*80+i] = raw_bytes[i*2+1] << 8 | raw_bytes[i*2];
							}
						}
						if(index == 17)
						{
							scan->angle_min = 0;
							scan->angle_max = 2.0 * M_PI;
							scan->angle_increment = ((2.0 * M_PI) / 1440.0);
							scan->time_increment = 1 / 1e8;
							scan->range_min = 0.15;
							scan->range_max = 50.0;
							scan->ranges.reserve(1440);
	
							for (uint16_t i = 0; i < 1440; i++)
							{
								scan->ranges.push_back(data_bytes[i] / 100.0);
								//scan->ranges.push_back(i / 100.0);
							}
							got_scan = true;
							ROS_INFO("push data ok\r\n");
						}
					}
					else if(point_resolution == 40)
					{
						if (index >= 0 && index <= 17)
						{
							boost::asio::read(serial_, boost::asio::buffer(&raw_bytes, 80));
							for(uint8_t i=0;i<40;i++)
							{
								data_bytes[index*40+i] = raw_bytes[i*2+1] << 8 | raw_bytes[i*2];
							}
						}
						if(index == 17)
						{
							scan->angle_min = 0;
							scan->angle_max = 2.0 * M_PI;
							scan->angle_increment = ((2.0 * M_PI) / 720.0);
							scan->time_increment = 1 / 1e8;
							scan->range_min = 0.15;
							scan->range_max = 50.0;
							scan->ranges.reserve(720);
	
							for (uint16_t i = 0; i < 720; i++)
							{
								scan->ranges.push_back(data_bytes[i] / 100.0);
								//scan->ranges.push_back(i / 100.0);
							}
							got_scan = true;
							ROS_INFO("push data ok\r\n");
						}
					}

					start_count = 0;
				}
			}
		}
	}
};
