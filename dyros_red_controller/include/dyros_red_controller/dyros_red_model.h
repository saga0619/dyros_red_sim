#ifndef dyros_red_MODEL_H
#define dyros_red_MODEL_H

#include <string>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "math_type_define.h"

using namespace Eigen;

namespace dyros_red_controller
{

class DyrosRedModel
{
public:
  DyrosRedModel();

  VectorXd _q;
  VectorXd _qdot;

  //dynamics info from RBDL
  MatrixXd _A; //inertia matrix
  VectorXd _G; //gravity vector
  VectorXd _b; //Coriolis/centrifugal vector
  Vector3d _G_direction; //gravity force direction vector
  Vector3d wCOM_position; //COM of whole robot
  MatrixXd J_wCOM; //COM Jacobian of whole robot

  struct Link{
    int id;
    double Mass;
    Vector3d COM_position;
    Matrix3d Inertia;
    Matrix3d RotationMatrix;
    Vector3d Link_position_global;
    Vector3d Contact_position;
    Vector3d Control_position;

    MatrixXd J_point; //Jacobian - point?
    MatrixXd J; //Jacobian
    MatrixXd J_COM; //Jacobian - CoM
    MatrixXd J_COM_p; //Jacobian - CoM Position
    MatrixXd J_COM_r; //Jacobian - CoM Orientation
    MatrixXd J_Contact; //Jacobian - Contact
  };

  void Link_initialize(int i, int id, double mass, Vector3d xipos);
  void Link_pos_Update(int i);
  void Link_rot_Update(int i);
  void Link_J_COM(int i);
  void Link_J_Contact_Position(int i, Vector3d Contact_position);
  void Link_J_Custom_Position(int i, Vector3d Jacobian_position);
  void COM_J_Kinematics_Update();


  static constexpr size_t MODEL_DOF = 31;
  static constexpr size_t LINK_NUMBER = 32;
  static constexpr size_t MODEL_DOF_VIRTUAL = 37;
  static constexpr size_t ARM_DOF = 8;
  static constexpr size_t LEG_DOF = 6;
  static constexpr size_t WAIST_DOF = 3;

  static constexpr size_t Pelvis = 0;
  static constexpr size_t Wasit = 1;
  static constexpr size_t Upper_Body = 3;
  static constexpr size_t Left_Leg = 4;
  static constexpr size_t Right_Leg = 10;
  static constexpr size_t Left_Arm = 16;
  static constexpr size_t Right_Arm = 24;

  static constexpr size_t joint_left_leg = 0;
  static constexpr size_t joint_right_leg = 6;
  static constexpr size_t joint_body = 12;
  static constexpr size_t joint_left_arm = 15;
  static constexpr size_t joint_right_arm = 23;


  //part part_[MODEL_DOF+1];
  Link link_[40];

  static const std::string JOINT_NAME[MODEL_DOF];
  static constexpr const char* EE_NAME[4] =
      {"L_AnkleRoll_Link", "R_AnkleRoll_Link",
       "L_HandYaw_Link", "R_HandYaw_Link" };

  /*static constexpr const char* LINK_NAME[32] =
      {"L0_Link", "L1_Link", "L2_Link", "L3_Link", "L4_Link", "L5_Link",
       "R0_Link", "R1_Link", "R2_Link", "R3_Link", "R4_Link", "R5_Link",
        "Waist1_Link","Waist2_Link","Upperbody_Link",
       "UL0_Link","UL1_Link","UL2_Link","UL3_Link","UL4_Link","UL5_Link","UL6_Link","UL7_Link",
       "UR0_Link","UR1_Link","UR2_Link","UR3_Link","UR4_Link","UR5_Link","UR6_Link","UR7_Link"};
  */
  static constexpr const char* LINK_NAME[32] =  {
    "Pelvis_Link","Waist1_Link","Waist2_Link","Upperbody_Link",
    "L_HipRoll_Link", "L_HipCenter_Link", "L_Thigh_Link", "L_Knee_Link", "L_AnkleCenter_Link", "L_AnkleRoll_Link",
    "R_HipRoll_Link", "R_HipCenter_Link", "R_Thigh_Link", "R_Knee_Link", "R_AnkleCenter_Link", "R_AnkleRoll_Link",
    "L_Shoulder1_Link","L_Shoulder2_Link","L_Shoulder3_Link","L_Armlink_Link","L_Elbow_Link","L_Forearm_Link","L_Wrist1_Link","L_Wrist2_Link",
    "R_Shoulder1_Link","R_Shoulder2_Link","R_Shoulder3_Link","R_Armlink_Link","R_Elbow_Link","R_Forearm_Link","R_Wrist1_Link","R_Wrist2_Link"
    };
  unsigned int _end_effector_id[4];
  unsigned int _link_id[40];


  std::map<std::string, size_t> joint_name_map_;
  size_t getIndex(const std::string& joint_name)
  {
    return joint_name_map_[joint_name];
  }

  // Calc Jacobian, Transformation
  bool updateKinematics(const VectorXd &q);
  bool updateKinematics(const VectorXd &q, const VectorXd &qdot);
  bool updateDynamics(const bool update_kinematics);

  //void getCenterOfMassPosition(Vector3d* position);
 // void setquat(Quaterniond& quat, const VectorXd& q);
 // const Vector3d getCurrentCom(){ return com_;}

 // VectorXd getGravityCompensation();
 // VectorXd GetDampingTorque(Eigen::VectorXd qdot, double damp_);

private:


  RigidBodyDynamics::Model _model;

};

typedef Matrix<double, DyrosRedModel::MODEL_DOF, 1> VectorQd;
typedef Matrix<double, DyrosRedModel::MODEL_DOF_VIRTUAL, 1> VectorVQd;
typedef Matrix<double, DyrosRedModel::MODEL_DOF, DyrosRedModel::MODEL_DOF> MatrixQd;
typedef Matrix<double, DyrosRedModel::MODEL_DOF_VIRTUAL, DyrosRedModel::MODEL_DOF_VIRTUAL> MAtrixVQd;


}
#endif // dyros_red_MODEL_H
