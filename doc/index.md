
   * Robot in Gazebo

   roslaunch nono_simulator empty_world.launch

   world_name:=`rospack find common_simulations`/worlds/oval_01.world


   * teleop

   roslaunch nono_guidance  teleop.launch

   * open loop controller input

   rosrun two_d_guidance send_cmd_vel.py _cmd_topic:=/nono/nono_diff_drive_controller/cmd_vel _signal_type:=sine_2 _amp:=0.1 _period:=6.


   * move base demonstration

   roslaunch nono_guidance move_base_demo.launch

   * pure pursuit demonstration

   roslaunch nono_guidance pure_pursuit_demo.launch



   roslaunch nono_simulator empty_world.launch world_name:=`rospack find common_simulations`/worlds/oval_01.world
   roslaunch nono_guidance pure_pursuit_demo.launch start_gazebo:=false path_filename:=`rospack find two_d_guidance`/paths/demo_z/oval_01.npz

   roslaunch nono_simulator empty_world.launch world_name:=`rospack find common_simulations`/worlds/ethz_cam1.world
   roslaunch nono_guidance pure_pursuit_demo.launch start_gazebo:=false path_filename:=`rospack find two_d_guidance`/paths/demo_z/track_ethz_cam1_new.npz

   rosrun two_d_guidance follow_line_node.py _cameras:=nono_0/camera1


   roslaunch nono_simulator empty_world.launch world_name:=`rospack find common_simulations`/worlds/ethz_cam1.world
   roslaunch nono_guidance move_base_demo.launch map_path:=/home/poine/work/rosmip/rosmip/rosmip_worlds/maps/expe_z/track_ethz_cam1_new.yaml


   roslaunch nono_guidance move_base_demo.launch map_path:=/home/poine/work/rosmip/rosmip/rosmip_worlds/maps/enac_bench/track_test2.yaml