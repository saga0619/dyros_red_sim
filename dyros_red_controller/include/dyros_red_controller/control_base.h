#ifndef _CONTROL_BASE_H
#define _CONTROL_BASE_H

// STD Library
#include <array>
#include <vector>

// System Library
#include <termios.h>

// ROS Library
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

// ROS Messages
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <smach_msgs/SmachContainerStatus.h>

#include "dyros_red_msgs/JointSet.h"
#include "dyros_red_msgs/JointState.h"
#include <dyros_red_msgs/TaskCommand.h>
#include <dyros_red_msgs/JointCommand.h>
#include <dyros_red_msgs/WalkingCommand.h>
//#include "dyros_red_msgs/RecogCmd.h"
//#include "dyros_red_msgs/TaskCmdboth.h"

// User Library
#include "math_type_define.h"
#include "dyros_red_controller/dyros_red_model.h"
#include "wholebodycontroller.h"
#include "quadraticprogram.h"
#include <qpOASES.hpp>

namespace dyros_red_controller
{

using namespace qpOASES;
using namespace Eigen;
using namespace std;
using namespace DyrosMath;

class ControlBase
{

public:
  ControlBase(ros::NodeHandle &nh, double Hz);
  virtual ~ControlBase(){}
  // Default User Call function
  void parameterInitialize(); // initialize all parameter function(q,qdot,force else...)
  virtual void readDevice(); // read device means update all subscribed sensor data and user command
  virtual void update(); // update controller based on readdevice
  virtual void compute(); // compute algorithm and update all class object
  virtual void reflect(); // reflect next step actuation such as motor angle else
  virtual void writeDevice()=0; // publish to actuate devices
  virtual void wait()=0;  // wait

  bool checkStateChanged();
  void stateChangeEvent();
  const double getHz() { return Hz_; }


protected:

  //unsigned int joint_id_[DyrosRedModel::MODEL_DOF];
  //unsigned int joint_id_inversed_[DyrosRedModel::MODEL_DOF];
  unsigned int control_mask_[DyrosRedModel::MODEL_DOF];


  int ui_update_count_;
  bool is_first_boot_;

  bool _kinematics_update;
  bool _dynamics_update;
  bool _binit_variables;
  bool _bstate_change;

  VectorQd _q; // current q
  VectorQd _qdot; // current qdot
  VectorQd _torque; // current joint toruqe

  VectorVQd _q_virtual; //including virtual joints
  VectorVQd _qdot_virtual; //including virtual joints

  Vector6d _FT_left_foot; // current left ft sensor values
  Vector6d _FT_right_foot; // current right ft sensor values

  tf::Quaternion imu_data_; ///< IMU data with filter

  Vector3d gyro_; // current gyro sensor values
  Vector3d accelometer_; // current accelometer values

  Matrix3d pelvis_orientation_;
  Vector3d pelvis_position_;
  Vector3d Pelvis_linear_velocity_;
  Vector3d Pelvis_angular_velocity_;
  Quaterniond Pelvis_quaternion;

  VectorQd _q_d; // current desired joint values
  VectorQd _torque_d; //command for joint torque control
  VectorQd _position_d; //command for joint position control
  VectorQd torque_damping;

  int total_dof_;

  DyrosRedModel _model;

  //Variables for whole-body control
  void WBCInitialize();
  void WBCReset(const int &contactdof, const int &taskdof);
  MatrixXd setWBCContactJacobian(const int ContactState);
  MatrixXd setWBCTaskJacobian(const int TaskState);
  void DefineQP_TaskModificationContactConstraint(CQuadraticProgram &QPprog);
  void DefineQP_PrimalContactWrenchDistributionForMultiContact(CQuadraticProgram &QPprog, const int &plane_contact_num, const VectorXd &arb_P_dis, const MatrixXd &arb_Rot, const VectorXd Fc_LocalContactFrame);
  void DefineQP_ContactWrenchDistributionForMultiContactInequality(CQuadraticProgram &QPprog, const int &plane_contact_num, const VectorXd &arb_P_dis, const MatrixXd &arb_Rot, const VectorXd &friction_coeff, const VectorXd &CoP_boundary, const VectorXd Fc_LocalContactFrame, const VectorXd Fz_local_decided);
  void WrenchRedistributionTwoFootContact(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Vector3D P1, Vector3D P2, VectorXd &F12, VectorXd& ResultantForce, VectorXd& ForceRedistribution, double& eta);
  void WrenchRedistributionTwoFootContact_CustomEta(double Eta_Standard, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Vector3D P1, Vector3D P2, VectorXd &F12, VectorXd& ResultantForce,  VectorXd& ForceRedistribution);
  void ResultantWrenchTwoContact(const Vector3D& P1, const Vector3D& P2, const Matrix3d& Rot, const VectorXd& F12, MatrixXd &W, VectorXd& ResultantForce);
  int _ContactState;
  int _TaskState;
  int _preContactState;
  int _preTaskState;
  MatrixXd _Jc;
  MatrixXd _Jc_dot;  
  VectorXd _pc;
  VectorXd _Fc_LocalContactFrame;
  MatrixXd _J;
  VectorQd _torque_gravity;
  VectorQd _torque_task;
  VectorQd _torque_contact;
  VectorXd _fstar;

  //Variables for task set
  bool _bool_init;

  Vector3d _init_pelvis_position;
  Matrix3d _init_pelvis_rotation;
  Vector3d  _init_right_arm_position;
  Matrix3d  _init_right_arm_rotation;
  Vector3d  _init_left_arm_position;
  Matrix3d  _init_left_arm_rotation;

  Vector3d _desired_pelvis_position;
  Vector3d _desired_pelvis_velocity;
  Matrix3d _desired_pelvis_rotation;
  Vector3d _desired_left_arm_position;
  Vector3d _desired_left_arm_velocity;
  Matrix3d _desired_left_arm_rotation;
  Vector3d _desired_right_arm_position;
  Vector3d _desired_right_arm_velocity;
  Matrix3d _desired_right_arm_rotation;
  Vector3d _desired_left_foot_position;
  Matrix3d _desired_left_foot_rotation;
  Vector3d _desired_right_foot_position;
  Matrix3d _desired_right_foot_rotation;

  Vector3d _pelvis_position_error;
  Vector3d _pelvis_velocity_error;
  Vector3d _pelvis_orientation_error;
  Vector3d _left_arm_position_error;
  Vector3d _left_arm_velocity_error;
  Vector3d _left_arm_orientation_error;
  Vector3d _right_arm_position_error;
  Vector3d _right_arm_velocity_error;
  Vector3d _right_arm_orientation_error;
  Vector3d _left_foot_position_error;
  Vector3d _left_foot_orientation_error;
  Vector3d _right_foot_position_error;
  Vector3d _right_foot_orientation_error;

  VectorXd _xdot;  

  //for QP problem  
  void QPInitialize();
  void QPReset(const int &contactdof, const int &taskdof);
  int _nIter;
  CQuadraticProgram _QP_task_modification;
  MatrixXd _H_task_modification;
  VectorXd _g_task_modification;
  MatrixXd _A_task_modification;
  VectorXd _lbA_task_modification;
  VectorXd _ubA_task_modification;
  MatrixXd _kp_Matrix;
  VectorXd _xd_Vector;
  VectorXd _kpx_kdxdot_Vector;
  VectorXd _Sol_task_modification;

  CQuadraticProgram _QP_contactwrench_distribution;
  CQuadraticProgram _QP_contactwrench_distribution_mod;

  //TaskController task_controller_;



protected:
  string current_state_;
  realtime_tools::RealtimePublisher<dyros_red_msgs::JointState> joint_state_pub_;

private:
  double Hz_; ///< control
  unsigned long tick_;
  double control_time_;

  string previous_state_;


  // ROS
  ros::Subscriber task_cmd_sub_;
  ros::Subscriber joint_cmd_sub_;
  //ros::Subscriber task_comamnd_sub_;
  ros::Subscriber joint_command_sub_;
  ros::Subscriber walking_command_sub_;
  //ros::Subscriber recog_point_sub_;
  // ros::Subscriber recog_cmd_sub_;

  // State Machine (SMACH)
  realtime_tools::RealtimePublisher<std_msgs::String> smach_pub_;
  ros::Subscriber smach_sub_;




  void smachCallback(const smach_msgs::SmachContainerStatusConstPtr& msg);
  //void taskCommandCallback(const dyros_red_msgs::TaskCommandConstPtr& msg);
private:

  //void makeIDInverseList();

};

}

#endif
