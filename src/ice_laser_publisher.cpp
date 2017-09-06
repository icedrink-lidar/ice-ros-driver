/*
																																										 
				_/_/_/    _/_/_/  _/_/_/_/  _/_/_/    _/_/_/    _/_/_/  _/      _/  _/    _/   
				 _/    _/        _/        _/    _/  _/    _/    _/    _/_/    _/  _/  _/      
				_/    _/        _/_/_/    _/    _/  _/_/_/      _/    _/  _/  _/  _/_/         
			       _/    _/        _/        _/    _/  _/    _/    _/    _/    _/_/  _/  _/        
			    _/_/_/    _/_/_/  _/_/_/_/  _/_/_/    _/    _/  _/_/_/  _/      _/  _/    _/       
																																																																																					
		            Copyright (C) 2017                          Beijing IceDrink Technology Co.,Ltd.
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <ice_laser_driver/ice_laser.h>
#include <std_msgs/UInt16.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ice_laser_publisher");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");

  std::string port;
  int baud_rate;
  std::string frame_id;

  priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
  priv_nh.param("baud_rate", baud_rate, 115200);
  priv_nh.param("frame_id", frame_id, std::string("ice_laser"));

  boost::asio::io_service io;

  try
  {
    ice_laser_driver::IceLaser laser(port, baud_rate, io);
    ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);

    while (ros::ok())
    {
      sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
      scan->header.frame_id = frame_id;
      scan->header.stamp = ros::Time::now();
      laser.poll(scan);
      laser_pub.publish(scan);
    }
    laser.close();
    return 0;
  }
  catch (boost::system::system_error ex)
  {
    ROS_ERROR("Error instantiating laser object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
    return -1;
  }
}
