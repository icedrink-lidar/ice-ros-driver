/*
																																										 
				_/_/_/    _/_/_/  _/_/_/_/  _/_/_/    _/_/_/    _/_/_/  _/      _/  _/    _/   
				 _/    _/        _/        _/    _/  _/    _/    _/    _/_/    _/  _/  _/      
				_/    _/        _/_/_/    _/    _/  _/_/_/      _/    _/  _/  _/  _/_/         
			       _/    _/        _/        _/    _/  _/    _/    _/    _/    _/_/  _/  _/        
		            _/_/_/    _/_/_/  _/_/_/_/  _/_/_/    _/    _/  _/_/_/  _/      _/  _/    _/       
																																																																																					
		            Copyright (C) 2017                          Beijing IceDrink Technology Co.,Ltd.
*/

#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>

namespace ice_laser_driver {
    class IceLaser {
        public:
            /**
              * @brief Constructs a new IceLaser attached to the given serial port
              * @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
              * @param baud_rate The baud rate to open the serial port at.
              * @param io Boost ASIO IO Service to use when creating the serial port object
              */
              IceLaser(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io);

            /**
              * @brief Default destructor
              */
            ~IceLaser() {};

            /**
              * @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
              * @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
              */
            void poll(sensor_msgs::LaserScan::Ptr scan);

            /**
              * @brief Close the driver down and prevent the polling loop from advancing
              */
            void close() { shutting_down_ = true; };

        private:
            std::string port_; ///< @brief The serial port the driver is attached to
            uint32_t baud_rate_; ///< @brief The baud rate for the serial connection

            bool shutting_down_; ///< @brief Flag for whether the driver is supposed to be shutting down or not
            boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the Ice Laser Scanner
    };
};
