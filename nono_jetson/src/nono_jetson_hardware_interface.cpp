
#include <controller_manager/controller_manager.h>

#include <nono_jetson/nono_jetson_hardware_interface.h>

#define __NAME "nono_jetson_hardware_interface"
const std::string joint_name_[NB_JOINTS] = {"left_wheel_joint","right_wheel_joint"};

NonoJetsonHardwareInterface::NonoJetsonHardwareInterface() {
  ROS_INFO_STREAM_NAMED(__NAME, "In NonoJetsonHardwareInterface::NonoJetsonHardwareInterface...");
  ROS_INFO_STREAM_NAMED(__NAME, "Registering interfaces");
  // register joints
  for (int i=0; i<NB_JOINTS; i++) {
    joint_position_[i] = joint_velocity_[i] = joint_effort_[i] = 0.;
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
      joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
    joint_velocity_command_[i] = 0.;
    vj_interface_.registerHandle(hardware_interface::JointHandle(
	js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]));
  }
  registerInterface(&js_interface_);
  registerInterface(&vj_interface_);
}

NonoJetsonHardwareInterface::~NonoJetsonHardwareInterface() {
  ROS_INFO_STREAM_NAMED(__NAME, "in NonoJetsonHardwareInterface::~NonoJetsonHardwareInterface");
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
bool NonoJetsonHardwareInterface::start(const ros::Time& time) {
  ROS_INFO_STREAM_NAMED(__NAME, "in NonoJetsonHardwareInterface::start");
  bool res = _md25.open();
  if (res) _md25.reset();
  return res;
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
bool NonoJetsonHardwareInterface::shutdown() {
  ROS_INFO("in NonoJetsonHardwareInterface::shutdown");

  return true;
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
#define RAD_PER_TICK (2*M_PI/361)  // Encoder resolution FIXME, make that configurable
//#define RAD_PER_TICK 1  // Encoder resolution FIXME, make that configurable
void NonoJetsonHardwareInterface::read(const ros::Time& now) {
  double enc[2], enc_vel[2];
  double iqsp[2], iqm[2];
  //od_->readFeedback(enc, enc_vel, iqsp, iqm);
  _md25.read(now);
  joint_position_[0] = _md25._enc_l*RAD_PER_TICK*ODRV_LW_POL;//enc[ODRV_LW_CHAN];
  joint_position_[1] = _md25._enc_r*RAD_PER_TICK*ODRV_RW_POL;//enc[ODRV_RW_CHAN]*RAD_PER_TICK*ODRV_RW_POL;
  //joint_velocity_[0] = enc_vel[ODRV_LW_CHAN]*RAD_PER_TICK*ODRV_LW_POL;
  //joint_velocity_[1] = enc_vel[ODRV_RW_CHAN]*RAD_PER_TICK*ODRV_RW_POL;
}

/*******************************************************************************
 *
 *
 *******************************************************************************/
#define TICK_PER_RAD (1./RAD_PER_TICK)
void NonoJetsonHardwareInterface::write() {
  
  double sp_tps_left = joint_velocity_command_[0]*TICK_PER_RAD*ODRV_LW_POL;
  double sp_tps_right = joint_velocity_command_[1]*TICK_PER_RAD*ODRV_RW_POL;
  double vsps[2] = {sp_tps_left, sp_tps_right};
  double iffs[2] = {0., 0.};
  int8_t vl=int(joint_velocity_command_[0])*4;
  int8_t vr=int(joint_velocity_command_[1])*4;
  //od_->sendVelSetpoints(vsps, iffs);
  _md25.write(vl, vr);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, __NAME);
  ROS_INFO_STREAM_NAMED(__NAME, "nono jetson hardware node starting...");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~");
  double period;
  nh.param("hardware_interface_dt", period, 1./50);
  ROS_INFO_STREAM_NAMED(__NAME, "  period: " << period);
  
  NonoJetsonHardwareInterface hw;
  if (!hw.start(ros::Time::now())) {
    ROS_ERROR_STREAM_NAMED(__NAME, "Failed to initialize hardware. bailling out...");
    return -1;
  }
  
  controller_manager::ControllerManager cm(&hw, nh);
  ros::Duration _period(period);
  while (ros::ok()) {
    ros::Time now = ros::Time::now();
    hw.read(now);
    cm.update(now, _period);
    hw.write();
    _period.sleep();
  }
  
  ROS_INFO_STREAM_NAMED(__NAME, "nono jetson hardware node exiting...");
  hw.shutdown();
  return 0;
}
