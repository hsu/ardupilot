/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for ardupilot version of Gazebo
*/

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "SIM_Gazebo.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
  constructor
 */
Gazebo::Gazebo(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    last_timestamp(0),
    sock(true)
{
    // try to bind to a specific port so that if we restart ArduPilot
    // Gazebo keeps sending us packets. Not strictly necessary but
    // useful for debugging
    sock.bind("127.0.0.1", 9003);

    sock.reuseaddress();
    sock.set_blocking(false);
    fprintf(stdout, "bind\n");
}

/*
  decode and send servos
*/
void Gazebo::send_servos(const struct sitl_input &input)
{
    fprintf(stdout, "send\n");
    float swash1 = (input.servos[0]-1000) / 1000.0f;
    float swash2 = (input.servos[1]-1000) / 1000.0f;
    float swash3 = (input.servos[2]-1000) / 1000.0f;
    float tail_rotor = (input.servos[3]-1000) / 1000.0f;
    float rsc = (input.servos[7]-1000) / 1000.0f;

    float col_pitch = (swash1+swash2+swash3)/3.0 - 0.5f;
    float roll_rate = (swash1 - swash2)/2;
    float pitch_rate = -((swash1 + swash2)/2.0 - swash3)/2;
    float yaw_rate = -(tail_rotor - 0.5);
    
    servo_packet pkt;
    pkt.motor_speed[0] = 100;
    pkt.motor_speed[1] = 100;
    pkt.motor_speed[2] = 100;
    pkt.motor_speed[3] = 100;
    sock.sendto(&pkt, sizeof(pkt), "127.0.0.1", 9002);
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void Gazebo::recv_fdm(const struct sitl_input &input)
{
    fprintf(stdout, "recv\n");
    fdm_packet pkt;

    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    while (sock.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
        send_servos(input);
    }

    double t = pkt.timestamp;
    double anguler_velocity = pkt.imu_angular_velocity_rpy[2];
    double linear_acceleration = pkt.imu_linear_acceleration_xyz[2];
    double orientation_quat = pkt.imu_orientation_quat[3];
    double velocity_rpy = pkt.velocity_rpy[2];
    double position_xyz = pkt.position_xyz[2];

    // dcm.from_euler(pkt.roll, pkt.pitch, pkt.yaw);

    // auto-adjust to simulation frame rate
    double deltat = pkt.timestamp - last_timestamp;
    time_now_us += deltat * 1.0e6;

    if (deltat < 0.01 && deltat > 0) {
        adjust_frame_time(1.0/deltat);
    }
    last_timestamp = pkt.timestamp;
}

/*
  update the Gazebo simulation by one time step
 */
void Gazebo::update(const struct sitl_input &input)
{
    fprintf(stdout, "test\n");
    send_servos(input);
    recv_fdm(input);
    update_position();
}
#endif // CONFIG_HAL_BOARD
