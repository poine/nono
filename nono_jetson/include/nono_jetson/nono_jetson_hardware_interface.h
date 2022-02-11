#ifndef NONO_JETSON__NONO_JETSON_HARDWARE_INTERFACE_H
#define NONO_JETSON__NONO_JETSON_HARDWARE_INTERFACE_H

//
// This is ROS compatible hardware interface that uses beaglebone robotic hardware
// ( IMU, encoders) and a Odrive motor controller
//

#include <ros/ros.h>
// ros_control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include <nono_jetson/nono_jetson_md25.h>

#define NB_JOINTS 2

class NonoJetsonHardwareInterface : public hardware_interface::RobotHW
{
 public:
  NonoJetsonHardwareInterface();
  virtual ~NonoJetsonHardwareInterface();
  bool start(const ros::Time& time);
  void read(const ros::Time& time);
  void write();
  bool shutdown();
 private:
  // Joints
  double joint_position_[NB_JOINTS];
  double joint_velocity_[NB_JOINTS];
  double joint_effort_[NB_JOINTS];
  double joint_velocity_command_[NB_JOINTS];
  //double joint_effort_command_[NB_JOINTS];
 
  // for publishing joints state
  hardware_interface::JointStateInterface    js_interface_;
  // for accepting joints setpoints
  //hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;

  NonoJetsonMd25 _md25;
};

// caroline
#define ODRV_LW_CHAN 0
#define ODRV_RW_CHAN 1
#define ODRV_LW_POL  1.
#define ODRV_RW_POL  1.

#endif // NONO_JETSON__NONO_JETSON_HARDWARE_INTERFACE_H
