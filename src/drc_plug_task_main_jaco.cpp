#include <iostream>
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <kinova_driver/kinova_ros_types.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <geometry_msgs/PointStamped.h>
#include <kinova_msgs/PoseVelocity.h>  // velocity
#include <std_msgs/Float32.h>

#include <cmath>        // std::abs
#include <ctime>
#include <chrono>
using namespace std;
const double POSITION_THRESHOLD = 0.05;
const double ROTATION_THRESHOLD = 0.02;
geometry_msgs::Pose pose_new;

class SurpriseTask
{
  public:
    SurpriseTask()
    {
        display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        velocity_publisher = nh_.advertise<kinova_msgs::PoseVelocity>("/j2n6s300_driver/in/cartesian_velocity", 1, true);
    }


    void positionController(float x, float y, float z, float roll, float pitch, float yaw)
    {
        kinova_msgs::PoseVelocity msg;
        float k = 1; 
        float k_r = 1;
        
        msg.twist_linear_x = k*x;
        msg.twist_linear_y = k*y;
        msg.twist_linear_z = k*z;
        msg.twist_angular_x = k_r*roll;
        msg.twist_angular_y = k_r*pitch;
        msg.twist_angular_z = k_r*yaw;

        velocity_publisher.publish(msg);
    }

    void velocity_pub(float x, float y, float z, float roll, float pitch, float yaw)
    {
        kinova_msgs::PoseVelocity msg;       
        msg.twist_linear_x = x;
        msg.twist_linear_y = y;
        msg.twist_linear_z = z;
        msg.twist_angular_x = roll;
        msg.twist_angular_y = pitch;
        msg.twist_angular_z = yaw;


        velocity_publisher.publish(msg);
    }

    void getTargetPose()
    {
        listener_.waitForTransform("/root", "cable",ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("/root", "cable",ros::Time(0), transform_wire);

        target_pose_wire1.position.x = transform_wire.getOrigin().x();
        target_pose_wire1.position.y = transform_wire.getOrigin().y();
        target_pose_wire1.position.z = transform_wire.getOrigin().z();
        target_pose_wire1.orientation.x = transform_wire.getRotation().x(); 
        target_pose_wire1.orientation.y = transform_wire.getRotation().y(); 
        target_pose_wire1.orientation.z = transform_wire.getRotation().z(); 
        target_pose_wire1.orientation.w = transform_wire.getRotation().w(); 

        target_pose_wire2 = target_pose_wire1;
        target_pose_wire2.position.y = target_pose_wire1.position.y+F_DIS_GRASP;   
        
        target_pose_wire3 = target_pose_wire2;
        target_pose_wire3.position.y = target_pose_wire2.position.y+B_DIS_UNPLUG;

    }

    void getTipPose()
    {
        listener_.waitForTransform("socket2", "cable_tip",ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("socket2", "cable_tip",ros::Time(0), transform_tip);


        delta_x = transform_tip.getOrigin().x();
        delta_y = transform_tip.getOrigin().y();
        delta_z = transform_tip.getOrigin().z();

        tf::Quaternion q_tip(transform_tip.getRotation().x(),transform_tip.getRotation().y(),transform_tip.getRotation().z(),transform_tip.getRotation().w());
        tf::Matrix3x3 m_tip(q_tip);
        m_tip.getRPY(delta_roll, delta_pitch, delta_yaw);

        // get the position of the endeffector
        listener_.waitForTransform("/root", "j2n6s300_end_effector",ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("/root", "j2n6s300_end_effector",ros::Time(0), transform_ee);
        tf::Quaternion q_ee(transform_ee.getRotation().x(),transform_ee.getRotation().y(),transform_ee.getRotation().z(),transform_ee.getRotation().w());
        tf::Matrix3x3 m_ee(q_ee);
        double roll_ee, pitch_ee, yaw_ee;
        m_ee.getRPY(roll_ee, pitch_ee, yaw_ee);
        // get the target rotation
        double roll_target, pitch_target, yaw_target;
        roll_target = roll_ee + delta_roll;
        pitch_target = pitch_ee + delta_pitch;
        yaw_target = yaw_ee + delta_yaw;
        tf::Quaternion q_new = tf::createQuaternionFromRPY(roll_target, pitch_target, yaw_target); 

        pose_new.position.x = transform_ee.getOrigin().x();
        pose_new.position.y = transform_ee.getOrigin().y();
        pose_new.position.z = transform_ee.getOrigin().z();
        pose_new.orientation.x = q_new.x();
        pose_new.orientation.y = q_new.y();
        pose_new.orientation.z = q_new.z();
        pose_new.orientation.w = q_new.w();

    }

    bool gripper_action(double finger_turn)
    {

      if (finger_turn < 0)
      {
          finger_turn = 0.0;
      }
      else
      {
          finger_turn = std::min(finger_turn, FINGER_MAX);
      }

      kinova_msgs::SetFingersPositionGoal goal;
      goal.fingers.finger1 = finger_turn;
      goal.fingers.finger2 = goal.fingers.finger1;
      goal.fingers.finger3 = goal.fingers.finger1;

      actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> finger_client("/j2n6s300_driver/fingers_action/finger_positions" , false);
      while(!finger_client.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the finger action server to come up");
      } 


      finger_client.sendGoal(goal);

      if (finger_client.waitForResult(ros::Duration(5.0)))
      {
          finger_client.getResult();
          return true;
      }
      else
      {
          finger_client.cancelAllGoals();
          ROS_WARN_STREAM("The gripper action timed-out");
          return false;
      }
    }

    void Move_robot_once(geometry_msgs::Pose pose)
    {
      moveit::planning_interface::MoveGroupInterface group("arm");
      moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
     
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

      // Create a publisher for visualizing plans in Rviz.
      moveit_msgs::DisplayTrajectory display_trajectory;

      // Getting Basic Information
      // We can print the name of the reference frame for this robot.
      ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
      
      // We can also print the name of the end-effector link for this group.
      ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

      // Planning to a Pose goal
      group.setPoseTarget(pose);

      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan (pose goal) %s",success?"":"FAILED");    
      
      // move the robot 
      ROS_INFO("Attention: moving the arm");
      // gripper_action(0.0); // open the gripper
      group.move();
      // sleep(1.0);
    }

    void Grasp_and_Pullout()
    { 
      
      moveit::planning_interface::MoveGroupInterface group("arm");
      moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
     
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

      // Create a publisher for visualizing plans in Rviz.
      moveit_msgs::DisplayTrajectory display_trajectory;

      // Getting Basic Information
      // We can print the name of the reference frame for this robot.
      ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
      
      // We can also print the name of the end-effector link for this group.
      ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

      // Planning to a Pose goal
      group.setPoseTarget(target_pose_wire1);


      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
      
      // move the robot 
      ROS_INFO("Attention: moving the arm");
      gripper_action(0.0); // open the gripper
      group.move();
      // sleep(1.0);
  ////////////////////////////////////////////////////////////////////////////////////////
      // move the robot with the object
      group.setPoseTarget(target_pose_wire2);

      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success2 = (group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 2 (pose goal) %s",success2?"":"FAILED");    

      ROS_INFO("Attention: moving the arm with the object");
      group.move();
      gripper_action(FINGER_MAX); // close the gripper
      // sleep(1.0);
  /////////////////////////////////////////////////////////////////////////////////////////
      group.setPoseTarget(target_pose_wire3);

      // Now, we call the planner to compute the plan and visualize it.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
      // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
      bool success3 = (group.plan(my_plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan 3 (pose goal) %s",success3?"":"FAILED");    

      ROS_INFO("Attention: moving the arm with the object");
      group.move();
    }   

    void updateControlInput()
    {
      try{
          listener_.waitForTransform("preinsert", "cable_tip",ros::Time(0), ros::Duration(3.0));
          listener_.lookupTransform("preinsert", "cable_tip",ros::Time(0), transform_tip);
      }
      catch (tf::TransformException ex) {
          ROS_ERROR("%s",ex.what());
      }

      tf::Quaternion q_tip(transform_tip.getRotation().x(),transform_tip.getRotation().y(),transform_tip.getRotation().z(),transform_tip.getRotation().w());
      tf::Matrix3x3 m_tip(q_tip);
      m_tip.getEulerYPR(delta_yaw, delta_pitch, delta_roll);
      std::cout << "Pose error before convertion" << delta_x << " " << delta_y << " " << delta_z << " " << delta_roll << " " << delta_pitch << " " << delta_yaw << std::endl;
 

      delta_x = transform_tip.getOrigin().x();
      delta_y = transform_tip.getOrigin().y();   // to the pre-insert state
      delta_z = transform_tip.getOrigin().z();
    }

    void getEulerYPR_ee()
    {
      try{
          listener_.waitForTransform("j2n6s300_end_effector", "preinsert",ros::Time(0), ros::Duration(3.0));
          listener_.lookupTransform("j2n6s300_end_effector", "preinsert",ros::Time(0), transform_pre_ee);
      }
      catch (tf::TransformException ex) {
          ROS_ERROR("%s",ex.what());
      }
      try{
          listener_.waitForTransform("j2n6s300_end_effector", "cable_tip",ros::Time(0), ros::Duration(3.0));
          listener_.lookupTransform("j2n6s300_end_effector", "cable_tip",ros::Time(0), transform_tip_ee); 
      }
      catch (tf::TransformException ex) {
          ROS_ERROR("%s",ex.what());
      }
          
      tf::Quaternion q_pre(transform_pre_ee.getRotation().x(),transform_pre_ee.getRotation().y(),transform_pre_ee.getRotation().z(),transform_pre_ee.getRotation().w());
      tf::Matrix3x3 m_pre(q_pre);
      m_pre.getEulerYPR(yaw_pre_ee, pitch_pre_ee, roll_pre_ee);    
      tf::Quaternion q_tip(transform_tip_ee.getRotation().x(),transform_tip_ee.getRotation().y(),transform_tip_ee.getRotation().z(),transform_tip_ee.getRotation().w());
      tf::Matrix3x3 m_tip(q_tip);
      m_tip.getEulerYPR(yaw_tip_ee, pitch_tip_ee, roll_tip_ee); 

      std::cout << "pre to ee" << roll_pre_ee << ", " << pitch_pre_ee << ", " << yaw_pre_ee << std::endl;
      std::cout << "tip to ee" << roll_tip_ee << ", " << pitch_tip_ee << ", " << yaw_tip_ee << std::endl;

      roll_ee = (roll_pre_ee - roll_tip_ee);
      pitch_ee = (pitch_pre_ee - pitch_tip_ee);
      yaw_ee = (yaw_pre_ee - yaw_tip_ee);
    }

    float pid_calculation(float setpoint, float pv, std::string ss)
    {
      if (ss == "x"){
        _Kp = 2;      // 2
        _Kd = 0.2;    // 0.2
        _Ki = 0; 
      }
      else if (ss=="y"){
        _Kp = 2;      // 2
        _Kd = 0.2;    // 0.2
        _Ki = 0; 
      }
      else if (ss=="z"){
        _Kp = 2;      // 2
        _Kd = 0.2;    // 0.2
        _Ki = 0; 
      }
      else if (ss=="roll"){       // kp = 1, kd = 0.01, ki = 0 (roll, pitch, yaw)works for pose alignment control in short distance
        _Kp = 2;      // 2
        _Kd = 0.2;    // 0.2
        _Ki = 0;
      }
      else if (ss=="pitch"){
        _Kp = 2;      // 2    // for experiment 2 and 3  use 0.01  (roll, pitch, yaw)
        _Kd = 0.2;    // 0.2  // for experiment 2 and 3 use 0.01  (roll, pitch, yaw)
        _Ki = 0;             // for experiment 2 and 3 use 0.000001 (roll, pitch, yaw)
      }
      else if (ss=="yaw"){
        _Kp = 2;      // 2
        _Kd = 0.2;    // 0.2
        _Ki = 0;
      }


      // Calculate error
      float error = pv - setpoint;

      // Proportional term
      float Pout = _Kp * error;

      float Iout;

      // Integral term
      if (ss == "x"){
        _integral_x += error * _dt;
        Iout= _Ki * _integral_x;
      }
      else if (ss=="y"){
        _integral_y += error * _dt;
        Iout = _Ki * _integral_y;
      }
      else if (ss=="z"){
        _integral_z += error * _dt;
        Iout = _Ki * _integral_z;
      }
      else if (ss=="roll"){
        _integral_roll += error * _dt;
        Iout = _Ki * _integral_roll;
      }
      else if (ss=="pitch"){
        _integral_pitch += error * _dt;
        Iout = _Ki * _integral_pitch;
      }
      else if (ss=="yaw"){
        _integral_yaw += error * _dt;
        Iout = _Ki * _integral_yaw;
      }


      // Derivative term
      float derivative;
      if (ss=="x"){
        derivative = (error - _pre_error_x) / _dt;
      }
      else if (ss=="y"){
        derivative = (error - _pre_error_y) / _dt;
      }
      else if (ss=="z"){
        derivative = (error - _pre_error_z) / _dt;
      }
      else if (ss=="roll"){
        derivative = (error - _pre_error_roll) / _dt;
      }
      else if (ss=="pitch"){
        derivative = (error - _pre_error_pitch) / _dt;
      }
      else if (ss=="yaw"){
        derivative = (error - _pre_error_yaw) / _dt;
      }


      float Dout = _Kd * derivative;

      // Calculate total output
      float output = Pout + Iout + Dout;

      // Save error to previous error
      if (ss=="x"){
        _pre_error_x = error;
      }
      else if (ss=="y"){
        _pre_error_y = error;
      }
      else if (ss=="z"){
        _pre_error_z = error;
      }
      else if (ss=="roll"){
        _pre_error_roll = error;
      }
      else if (ss=="pitch"){
        _pre_error_pitch = error;
      }
      else if (ss=="yaw"){
        _pre_error_yaw = error;
      }
      

      return output;
    }


    void wire_terminal_pre_insertion_ready_with_position_control_pid()
    {
      updateControlInput();
      getEulerYPR_ee();

      while (abs(delta_roll) > POSITION_THRESHOLD || abs(delta_pitch) > POSITION_THRESHOLD || abs(delta_yaw) > POSITION_THRESHOLD || abs(delta_x) > ROTATION_THRESHOLD || abs(delta_y) > ROTATION_THRESHOLD || abs(delta_z) > ROTATION_THRESHOLD)
      {

        // ZYX  using zyx, xyz is incorrect  // tested getEulerYPR function, it is working correctly
        // http://personal.maths.surrey.ac.uk/T.Bridges/SLOSH/3-2-1-Eulerangles.pdf
        float av_yaw = -sin(roll_ee)*pitch_ee + cos(pitch_ee)*cos(roll_ee)*yaw_ee;
        float av_pitch = cos(roll_ee)*pitch_ee + cos(pitch_ee)*sin(roll_ee)*yaw_ee;
        float av_roll = roll_ee - sin(pitch_ee)*yaw_ee;

        // jaco
        float jaco_roll = -av_pitch;
        float jaco_pitch = av_roll;
        float jaco_yaw  = av_yaw;

        float control_step_x = (float) pid_calculation(0, (float)delta_y,"x");
        float control_step_y = (float) pid_calculation(0, (float)delta_x,"y");
        float control_step_z = (float) pid_calculation(0, (float)delta_z,"z");
        float control_step_roll = (float) pid_calculation(0, jaco_roll,"roll");
        float control_step_pitch = (float) pid_calculation(0, jaco_pitch,"pitch");
        float control_step_yaw = (float) pid_calculation(0, jaco_yaw,"yaw");

        // // add speed limitation *************************
        // std::cout << "control speed limitations (linear: 1.5m/s, angular: 0.5rad/s)" << std::endl;
        // if (abs(control_step_x) > 1.5)
        // {
        //   float temp = abs((float)1.5/control_step_x);
        //   if (control_step_x > 1.5)
        //   {
        //     control_step_x = 1.5;
        //   }
        //   else
        //   {
        //     control_step_x = -1.5;
        //   }          
        //   control_step_y = control_step_y * temp;
        //   control_step_z = control_step_z * temp;
        //   control_step_roll = control_step_roll * temp;
        //   control_step_pitch = control_step_pitch * temp;
        //   control_step_yaw = control_step_yaw * temp;
        // }
        // if (abs(control_step_y) > 1.5)
        // {
        //   float temp = abs((float)1.5/control_step_y);
        //   if (control_step_y > 1.5)
        //   {
        //     control_step_y = 1.5;
        //   }
        //   else
        //   {
        //     control_step_y = -1.5;
        //   } 
        //   control_step_x = control_step_x * temp;
        //   control_step_z = control_step_z * temp;
        //   control_step_roll = control_step_roll * temp;
        //   control_step_pitch = control_step_pitch * temp;
        //   control_step_yaw = control_step_yaw * temp;
        // }
        // if (abs(control_step_z) > 1.5)
        // {
        //   float temp = abs((float)1.5/control_step_z);
        //   if (control_step_z > 1.5)
        //   {
        //     control_step_z = 1.5;
        //   }
        //   else
        //   {
        //     control_step_z = -1.5;
        //   } 
        //   control_step_x = control_step_x * temp;
        //   control_step_y = control_step_y * temp;
        //   control_step_roll = control_step_roll * temp;
        //   control_step_pitch = control_step_pitch * temp;
        //   control_step_yaw = control_step_yaw * temp;
        // }
        // if (abs(control_step_roll) > 0.6)
        // {
        //   float temp = abs((float)0.6/control_step_roll);
        //   if (control_step_roll > 0.6)
        //   {
        //     control_step_roll = 0.6;
        //   }
        //   else
        //   {
        //     control_step_roll = -0.6;
        //   } 
        //   control_step_x = control_step_x * temp;
        //   control_step_y = control_step_y * temp;
        //   control_step_z = control_step_z * temp;
        //   control_step_pitch = control_step_pitch * temp;
        //   control_step_yaw = control_step_yaw * temp;
        // }
        // if (abs(control_step_pitch) > 0.6)
        // {
        //   float temp = abs((float)0.6/control_step_pitch);
        //   if (control_step_pitch > 0.6)
        //   {
        //     control_step_pitch = 0.6;
        //   }
        //   else
        //   {
        //     control_step_pitch = -0.6;
        //   }
        //   control_step_x = control_step_x * temp;
        //   control_step_y = control_step_y * temp;
        //   control_step_z = control_step_z * temp;
        //   control_step_roll = control_step_roll * temp;
        //   control_step_yaw = control_step_yaw * temp;
        // }
        // if (abs(control_step_yaw) > 0.6)
        // {
        //   float temp = abs((float)0.6/control_step_yaw);
        //   if (control_step_yaw > 0.6)
        //   {
        //     control_step_yaw = 0.6;
        //   }
        //   else
        //   {
        //     control_step_yaw = -0.6;
        //   }
        //   control_step_x = control_step_x * temp;
        //   control_step_y = control_step_y * temp;
        //   control_step_z = control_step_z * temp;
        //   control_step_roll = control_step_roll * temp;
        //   control_step_pitch = control_step_pitch * temp;
        // }



        velocity_pub(control_step_x,control_step_y,control_step_z,control_step_roll,control_step_pitch,control_step_yaw);
        std::cout << "control xyzrollpitchyaw: " << control_step_x << "," << control_step_y << "," << control_step_z << "," << control_step_roll << ","<< control_step_pitch << "," << control_step_yaw << std::endl;  
        
        getEulerYPR_ee();
        updateControlInput();
        std::cout << "Pose error data:" << delta_x << " " << delta_y << " " << delta_z << " " << delta_roll << " " << delta_pitch << " " << delta_yaw << std::endl;

      }
    }

    void wire_terminal_insertion()
    { 

      listener_.waitForTransform("/root", "j2n6s300_end_effector",ros::Time(0), ros::Duration(3.0));
      listener_.lookupTransform("/root", "j2n6s300_end_effector",ros::Time(0), transform_ee);

      pose_new.position.x = transform_ee.getOrigin().x();
      pose_new.position.y = transform_ee.getOrigin().y();
      pose_new.position.z = transform_ee.getOrigin().z();
      pose_new.orientation.x = transform_ee.getRotation().x();
      pose_new.orientation.y = transform_ee.getRotation().y();
      pose_new.orientation.z = transform_ee.getRotation().z();
      pose_new.orientation.w = transform_ee.getRotation().w();

      Move_robot_once(pose_new);
      pose_new.position.y = pose_new.position.y + F_DIS_INSERTION;
      Move_robot_once(pose_new);

    }


  private:
    ros::NodeHandle nh_;
    ros::Publisher display_publisher;
    ros::Publisher velocity_publisher;

    tf::TransformListener listener_;
    tf::StampedTransform transform_wire;
    tf::StampedTransform transform_tip;
    tf::StampedTransform transform_ee;
    tf::StampedTransform transform_tip_world;
    tf::StampedTransform transform_pre_world;
    tf::StampedTransform transform_tip_ee;
    tf::StampedTransform transform_pre_ee;   

    geometry_msgs::Pose target_pose_wire1, target_pose_wire2, target_pose_wire3;
    double delta_roll, delta_pitch, delta_yaw; 
    double delta_x, delta_y, delta_z;
    double roll_tip_ee, pitch_tip_ee, yaw_tip_ee;
    double roll_pre_ee, pitch_pre_ee, yaw_pre_ee;
    double roll_tip_world, pitch_tip_world, yaw_tip_world;
    double roll_pre_world, pitch_pre_world, yaw_pre_world;
    double roll_ee, pitch_ee, yaw_ee;
    double x_world, y_world, z_world;
    double roll_test, pitch_test, yaw_test;
    int number_of_adjustment;

    double FINGER_MAX = 6400;
    double F_DIS_GRASP = -0.075;
    double B_DIS_UNPLUG = 0.15;
    double F_DIS_INSERTION = -0.11;

    //pid
    float _dt = 0.1;
    float _Kp;      // 2
    float _Kd;    // 0.2
    float _Ki;    
    float _pre_error_x=0, _pre_error_y=0, _pre_error_z=0, _pre_error_roll=0, _pre_error_pitch=0, _pre_error_yaw=0;
    float _integral_x=0, _integral_y=0, _integral_z=0, _integral_roll=0, _integral_pitch=0, _integral_yaw=0;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Surprise_Task");
    ros::AsyncSpinner spinner(4);  // important
    ros::Rate r(10);
    spinner.start();


    SurpriseTask ST;
    ROS_INFO("Initialized");
    auto start = std::chrono::system_clock::now();  // time 

    ST.getTargetPose();                    
    ST.Grasp_and_Pullout();               
    ST.wire_terminal_pre_insertion_ready_with_position_control_pid();    
    ST.wire_terminal_insertion(); 

    auto end = std::chrono::system_clock::now();     // time

    std::chrono::duration<double> elapsed_seconds = end-start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    std::cout << "finished computation at " << std::ctime(&end_time)
              << "elapsed time: " << elapsed_seconds.count() << "s\n";
    return 0;
}