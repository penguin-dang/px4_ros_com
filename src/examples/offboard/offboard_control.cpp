/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <thread>

#include <chrono>
#include <iostream>
#include <unistd.h>
#include <termios.h> // For custom getch()

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

#define USE_POSITION;

double drone_x = 0.0;
double drone_y = 0.0;
double drone_z = -3.0; // Start at 3 meters altitude (negative for down in ROS)
double drone_yaw = 0.0;
double velocity = 1.0;
bool tookOff = false;
bool armed = false;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control"), running_(true)
	{
		// offboard_control_node_ = std::make_shared<OffboardControl>();
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Start a thread to handle keyboard input
        input_thread_ = std::thread(&OffboardControl::inputLoop, this);

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			// if (offboard_setpoint_counter_ == 10) {
			// 	// Change to Offboard mode after 10 setpoints
			// 	this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

			// 	// Arm the vehicle
			// 	this->arm();
			// }

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode(); // Always call
			// publish_trajectory_setpoint();

			// stop the counter after reaching 11
			// if (offboard_setpoint_counter_ < 11) {
			// 	offboard_setpoint_counter_++;
			// }
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

    ~OffboardControl() {
        running_ = false; // Stop the input loop
        if (input_thread_.joinable()) {
            input_thread_.join(); // Wait for the input thread to finish
        }
    }

    void inputLoop() {
		std::cout << "---------------------------" << std::endl;
		std::cout << "Left wheel - gas & heading" << std::endl;
		std::cout << "        w     " << std::endl;
		std::cout << "   a    s    d" << std::endl;
		std::cout << std::endl;
		std::cout << "Right wheel - move" << std::endl;
		std::cout << "---------------------------" << std::endl;
		std::cout << "        i     " << std::endl;
		std::cout << "   j    k    l" << std::endl;
		std::cout << std::endl;
		std::cout << "f/h : arm/disarm" << std::endl;
		std::cout << "t   : RTL" << std::endl;
		std::cout << std::endl;
		std::cout << "x/z : increase/decrease max speeds by 10%" << std::endl;
		std::cout << std::endl;
		std::cout << "q to quit" << std::endl;
        while (running_) {
			usleep(100000);
            char command = getch();
            switch (command) {
                case 'f':
					publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                    arm();
					armed = true;
                    break;
                case 'h':
                    disarm();
					tookOff = false;
					armed = false;
                    break;
                case 'w':
					if (!armed) break;
					tookOff = true;
					#ifdef USE_POSITION
						drone_z-=velocity;
					#else
						drone_z=-velocity;
					#endif
					publish_trajectory_setpoint();
                    break;
                case 's':
					if (!tookOff) break;
					#ifdef USE_POSITION
						drone_z+=velocity;
					#else
						drone_z=+velocity;
					#endif
					publish_trajectory_setpoint();
                    break;
                case 'a':
					if (!tookOff) break;
					#ifdef USE_POSITION
						drone_yaw-=velocity/10.0;
					#else
						drone_yaw=-velocity/10.0;
					#endif
					publish_trajectory_setpoint();
                    break;
                case 'd':
					if (!tookOff) break;
					#ifdef USE_POSITION
						drone_yaw+=velocity/10.0;
					#else
						drone_yaw=+velocity/10.0;
					#endif
					publish_trajectory_setpoint();
                    break;
                case 'i':
					tookOff = true;
					#ifdef USE_POSITION
						drone_x-=velocity;
					#else
						drone_x=-velocity;
					#endif
					publish_trajectory_setpoint();
                    break;
                case 'k':
					if (!tookOff) break;
					#ifdef USE_POSITION
						drone_x+=velocity;
					#else
						drone_x=+velocity;
					#endif
					publish_trajectory_setpoint();
                    break;
                case 'j':
					if (!tookOff) break;
					#ifdef USE_POSITION
						drone_y-=velocity;
					#else
						drone_y=-velocity;
					#endif
					publish_trajectory_setpoint();
                    break;
                case 'l':
					if (!tookOff) break;
					#ifdef USE_POSITION
						drone_y+=velocity;
					#else
						drone_y=+velocity;
					#endif
					publish_trajectory_setpoint();
                    break;
                case 'z':
					velocity*=0.9;
					std::cout << "velocity = " << velocity << std::endl;
                    break;
                case 'x':
					velocity/=0.9;
					std::cout << "velocity = " << velocity << std::endl;
                    break;
                case 't':
					if (!tookOff) break;
					publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 5);
					tookOff = false;
					armed = false;
                    break;
                case 'q':
                    running_ = false; // Exit the loop
                    break;
                default:
                    std::cout << "Invalid command." << std::endl;
            }
        }
    }

    // Function to capture a single character input without Enter
    char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
            perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
            perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
            perror("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
            perror("tcsetattr ~ICANON");
        return buf;
    }

	void arm();
	void disarm();

private:
	std::shared_ptr<OffboardControl> offboard_control_node_;
    std::thread input_thread_;
    std::atomic<bool> running_; // Control flag for the input loop

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	// RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	// RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	// printf("is calles\n");
	TrajectorySetpoint msg{};
	msg.position = {drone_x, drone_y, drone_z};
	msg.yaw = drone_yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2, float param3)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
