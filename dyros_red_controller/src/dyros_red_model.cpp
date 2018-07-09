#include "dyros_red_controller/dyros_red_model.h"



namespace dyros_red_controller
{

// https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
constexpr const char* DyrosRedModel::EE_NAME[4];
constexpr const char* DyrosRedModel::LINK_NAME[32];
constexpr const size_t DyrosRedModel::MODEL_DOF;


// These should be replaced by YAML or URDF or something
const std::string DyrosRedModel::JOINT_NAME[DyrosRedModel::MODEL_DOF] = {
      "L_HipRoll_Joint", "L_HipCenter_Joint", "L_Thigh_Joint", "L_Knee_Joint", "L_AnkleCenter_Joint", "L_AnkleRoll_Joint",
      "R_HipRoll_Joint", "R_HipCenter_Joint", "R_Thigh_Joint", "R_Knee_Joint", "R_AnkleCenter_Joint", "R_AnkleRoll_Joint",
      "Waist1_Joint","Waist2_Joint","Upperbody_Joint",
      "L_Shoulder1_Joint","L_Shoulder2_Joint","L_Shoulder3_Joint","L_Armlink_Joint","L_Elbow_Joint","L_Forearm_Joint","L_Wrist1_Joint","L_Wrist2_Joint",
      "R_Shoulder1_Joint","R_Shoulder2_Joint","R_Shoulder3_Joint","R_Armlink_Joint","R_Elbow_Joint","R_Forearm_Joint","R_Wrist1_Joint","R_Wrist2_Joint"};

// 0~6 Left leg
// 7~11 Right leg
// 12~14 Waist
// 15~22 Left arm
// 23~30 Right arm


void DyrosRedModel::Link_initialize(int i, int id, double mass, Vector3d xipos){
  link_[i].id = id;
  link_[i].Mass = mass;
  link_[i].COM_position = xipos;

  link_[i].RotationMatrix.setZero();
  link_[i].Inertia.setZero();
  link_[i].Contact_position.setZero();
  link_[i].Control_position.setZero();

  link_[i].J.resize(6,MODEL_DOF+6); link_[i].J.setZero();
  link_[i].J_COM.resize(6,MODEL_DOF+6); link_[i].J_COM.setZero();
  link_[i].J_COM_p.resize(3,MODEL_DOF+6); link_[i].J_COM_p.setZero();
  link_[i].J_COM_r.resize(3,MODEL_DOF+6); link_[i].J_COM_r.setZero();
  link_[i].J_Contact.resize(6,MODEL_DOF+6); link_[i].J_Contact.setZero();
  link_[i].J_point.resize(6,MODEL_DOF+6); link_[i].J_point.setZero();
}

void DyrosRedModel::Link_pos_Update(int i){
  link_[i].Link_position_global = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model,_q,link_[i].id,Eigen::Vector3d::Zero(),false); //link position in base coordinate
}

void DyrosRedModel::Link_rot_Update(int i){
  link_[i].RotationMatrix = RigidBodyDynamics::CalcBodyWorldOrientation(_model,_q,link_[i].id,false); //link rotation matrix in world coordinate
}

void DyrosRedModel::Link_J_COM(int i){ //CoM Jacobian of link[i] update (J_COM_p, J_COM_r, J_COM)

  MatrixXd j_p_(3,MODEL_DOF+6), j_r_(3,MODEL_DOF+6);
  MatrixXd j_(6,MODEL_DOF+6);
  MatrixXd fj_(6,MODEL_DOF+6);
  fj_.setZero();

  RigidBodyDynamics::CalcPointJacobian6D(_model, _q, link_[i].id, link_[i].COM_position, fj_, false);

  j_p_=fj_.block<3,MODEL_DOF+6>(3,0);
  j_r_=fj_.block<3,MODEL_DOF+6>(0,0);

  link_[i].J_COM_p = j_p_;
  link_[i].J_COM_r = j_r_;

  j_.block<3,MODEL_DOF+6>(0,0) = link_[i].J_COM_p;
  j_.block<3,MODEL_DOF+6>(3,0) = link_[i].J_COM_r;

  link_[i].J_COM = j_;

}

void DyrosRedModel::Link_J_Custom_Position(int i, Vector3d Jacobian_position){
  MatrixXd fj_(6,MODEL_DOF+6);
  fj_.setZero();

  RigidBodyDynamics::CalcPointJacobian6D(_model,_q,link_[i].id,Jacobian_position,fj_,false);

  //link_[i].Jac.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0)*E_T_;
  //link_[i].Jac.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0)*E_T_;
  link_[i].J.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0);
  link_[i].J.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0);

  link_[i].Control_position = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model,_q,link_[i].id,Jacobian_position,false); //link position in base coordinate
}

void DyrosRedModel::Link_J_Contact_Position(int i, Vector3d Contact_position){

  MatrixXd fj_(6,MODEL_DOF+6);
  fj_.setZero();

  RigidBodyDynamics::CalcPointJacobian6D(_model,_q,link_[i].id,Contact_position,fj_,false);

  //swap position and orientation term
  link_[i].J_Contact.block<3,MODEL_DOF+6>(0,0)=fj_.block<3,MODEL_DOF+6>(3,0);
  link_[i].J_Contact.block<3,MODEL_DOF+6>(3,0)=fj_.block<3,MODEL_DOF+6>(0,0);

  link_[i].Contact_position = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model,_q,link_[i].id,Contact_position,false); //link position in base coordinate
}


DyrosRedModel::DyrosRedModel() //initialize
{
  //kinematics
  _q.resize(MODEL_DOF+6);
  _q.setZero();
  _qdot.resize(MODEL_DOF+6);
  _qdot.setZero();

  //dynamics
  _A.resize(MODEL_DOF+6,MODEL_DOF+6);
  _A.setZero();
  _G.resize(MODEL_DOF+6);
  _G.setZero();
  _b.resize(MODEL_DOF+6);
  _b.setZero();
  //direction of gravity
  _G_direction.setZero();
  _G_direction(2) = GRAVITY;

  wCOM_position.setZero();
  J_wCOM.resize(6,MODEL_DOF+6);
  J_wCOM.setZero();

  //load urdf model
  std::string desc_package_path = ros::package::getPath("dyros_red_description");
  std::string urdf_path = desc_package_path + "/robots/dyros_red_robot.urdf";

  ROS_INFO("Loading DYROS JET description from = %s",urdf_path.c_str());
  RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &_model, true, true);

  ROS_INFO("Successfully loaded.");

  ROS_INFO("MODEL DOF COUNT = %d", _model.dof_count);
  ROS_INFO("MODEL Q SIZE = %d", _model.q_size);
  //_model.mJoints[0].)
  if(_model.dof_count != MODEL_DOF+6)
  {
    ROS_WARN("The DoF in the model file and the code do not match.");
    ROS_WARN("Model file = %d, Code = %d", _model.dof_count, (int)MODEL_DOF+6);
  }
  else{
    //ROS_INFO("id:0 name is : %s",_model.GetBodyName(0));
    for (int i=0; i<MODEL_DOF+1; i++)
    {
      _link_id[i] = _model.GetBodyId(LINK_NAME[i]);
      ROS_INFO("%s: \t\t id = %d \t parent link = %d",LINK_NAME[i], _link_id[i],_model.GetParentBodyId(_link_id[i]));
      //ROS_INFO("%dth parent %d",_link_id[i],_model.GetParentBodyId(_link_id[i]));
      //std::cout << _model.mBodies[_link_id[i]].mCenterOfMass << std::endl;
      //joint_name_map_[JOINT_NAME[i]] = i;
    }
    for(int i=0; i<MODEL_DOF+1;i++)
    {
      Link_initialize(i,_link_id[i],_model.mBodies[_link_id[i]].mMass,_model.mBodies[_link_id[i]].mCenterOfMass);
    }

    for(int i=0;i<MODEL_DOF+6;i++){

      ROS_INFO("Joint type %d : %d", i, _model.mJoints[i].mJointType);
    }
  }
}


bool DyrosRedModel::updateKinematics(const VectorXd& q)
{
  _q = q; //joint angle including virtual joints

  RigidBodyDynamics::UpdateKinematicsCustom(_model, &_q, NULL, NULL);

  bool update_kinematics = true;
  return update_kinematics;
}

bool DyrosRedModel::updateKinematics(const VectorXd& q, const VectorXd& qdot)
{
  _q = q; //joint angle including virtual joints
  _qdot = qdot; //joint velocity including virtual joints

  RigidBodyDynamics::UpdateKinematicsCustom(_model, &_q, &_qdot, NULL);

  bool update_kinematics = true;
  return update_kinematics;
}

bool DyrosRedModel::updateDynamics(const bool update_kinematics)
{
  if(update_kinematics == true)
  {
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(_model, _q, _A, true);
    VectorXd zero_vec(MODEL_DOF+6);
    zero_vec.setZero();
    RigidBodyDynamics::InverseDynamics(_model, _q, zero_vec, zero_vec, _G);
    VectorXd Bias(MODEL_DOF+6);
    Bias.setZero();
    RigidBodyDynamics::InverseDynamics(_model, _q, _qdot, zero_vec, Bias);
    _b = Bias - _G;
  }
  else
  {
    ROS_WARN("Error!! Kinematics should be updated!!");
  }

  bool update_dynamics = true;
  return update_dynamics;
}


void DyrosRedModel::COM_J_Kinematics_Update()
{
  Vector3d tot_mass_pos;
  tot_mass_pos.setZero();
  double tot_mass = 0.0;
  MatrixXd J_COM_tmp(6,MODEL_DOF+6);
  J_COM_tmp.setZero();
  for(int i=0; i<MODEL_DOF+1; i++)
  {
    tot_mass = tot_mass + _model.mBodies[_link_id[i]].mMass;
    tot_mass_pos = tot_mass_pos + RigidBodyDynamics::CalcBodyToBaseCoordinates(_model,_q,link_[i].id,link_[i].COM_position,false)*_model.mBodies[_link_id[i]].mMass; //(link com global pos)* mass

    Link_J_COM(i);
    J_COM_tmp = J_COM_tmp + _model.mBodies[_link_id[i]].mMass*link_[i].J_COM;
  }
  wCOM_position = tot_mass_pos/tot_mass;
  J_wCOM = J_COM_tmp/tot_mass;
}



//{
  //A_=E_T_.transpose()*A_temp_*E_T_;

 // _A = A_temp_; //full A matrix including base inertia

 // getCenterOfMassPosition(&com_);

  //R_temp_.block<3,3>(0,0) = -base_rotation_;
  //R_temp_.block<3,3>(3,3) = -base_rotation_;
  //R_temp_.block<MODEL_DOF,MODEL_DOF>(6,6) = Eigen::MatrixXd::Identity(MODEL_DOF,MODEL_DOF);
  //A_= R_temp_.transpose()*A_temp_*R_temp_;

//  for(int i=0;i<MODEL_DOF+1;i++){
 //   Link_pos_Update(i);
//  }



//  t_temp = ros::Time::now();
//  for(int i=0;i<MODEL_DOF+1;i++){
//    Link_J_Update(i);
//  }
 // double ju_time = ros::Time::now().toSec() - t_temp.toSec();


  /*
  std::cout<<"LINK xpos :::::::::: x : "<<std::endl<<link_[Left_Arm+7].xpos(0)<<" y : "<<link_[Left_Arm+7].xpos(1) <<" z : "<<link_[Left_Arm+7].xpos(2)<<std::endl<<std::endl;
  std::cout<<"euler:::::::::: x : "<<std::endl<<q_virtual_(3)<<" y : "<<q_virtual_(4) <<" z : "<<q_virtual_(5)<<std::endl<<std::endl;
  std::cout<<"Er : " <<std::endl<<Er<<std::endl<<std::endl;
  std::cout<<"Eri : " <<std::endl<<Eri<<std::endl<<std::endl;
  std::cout<<"Jacobian ::: " <<std::endl <<link_[Left_Arm+7].Jac<<std::endl;
  std::cout<<"Jacobian*E_T_ ::: " <<std::endl <<link_[Left_Arm+7].Jac_COM<<std::endl;
  */


//  ROS_INFO("Update time - detail \n updatekinematicsCustum Time : % 3.4f ms\n compositeRigid time : %3.4f ms\n jac_update time : %3.4f ms",uk_time*1000,cr_time*1000,ju_time*1000);
//}

/*
Eigen::VectorXd DyrosRedModel::getGravityCompensation(){


  ros::Time time_temp = ros::Time::now();

  Eigen::Vector3d left_leg_contact, right_leg_contact;

  left_leg_contact<<0,0,-0.1368;
  right_leg_contact<<0,0,-0.1368;

  Link_Set_Contact(Left_Leg+5,left_leg_contact);
  Link_Set_Contact(Right_Leg+5,right_leg_contact);


  Eigen::Vector3d Grav_ref;

  Grav_ref.setZero(3);
  Grav_ref(2) = -9.81;


  Eigen::VectorXd G,Gtemp;
  G.setZero(MODEL_DOF+6);
  Gtemp.setZero(MODEL_DOF+6);

  for(int i=0;i<MODEL_DOF+1;i++){
    Gtemp = G - link_[i].Jac_COM_p.transpose()*link_[i].Mass*Grav_ref;
    G=Gtemp;
  }




  double g_calc_time = ros::Time::now().toSec() - time_temp.toSec();

  time_temp = ros::Time::now();

  //std::cout<<"G Vector"<<std::endl<<G<<std::endl;

  Eigen::MatrixXd A_matrix(MODEL_DOF+6,MODEL_DOF+6);
  Eigen::MatrixXd A_matrix_inverse(MODEL_DOF+6,MODEL_DOF+6);

  A_matrix.setZero();

  A_matrix = A_;
  A_matrix_inverse = A_matrix.inverse();

  Eigen::MatrixXd J_g;
  J_g.setZero(MODEL_DOF, MODEL_DOF+6);
  J_g.block(0, 6, MODEL_DOF, MODEL_DOF).setIdentity();

  Eigen::MatrixXd J_C, J_C_INV_T;
  J_C.setZero(12, MODEL_DOF+6);
  J_C.block(0, 0, 6, MODEL_DOF+6) = link_[Left_Leg+5].Jac_Contact;
  J_C.block(6, 0, 6, MODEL_DOF+6) = link_[Right_Leg+5].Jac_Contact;

  Eigen::MatrixXd Lambda_c;
  Lambda_c=(J_C*A_matrix_inverse*(J_C.transpose())).inverse();
  J_C_INV_T = Lambda_c*J_C*A_matrix_inverse;

  Eigen::MatrixXd N_C;
  N_C.setZero(MODEL_DOF+6, MODEL_DOF+6);
  Eigen::MatrixXd I37;
  I37.setIdentity(MODEL_DOF+6, MODEL_DOF+6);
  N_C =I37-J_C.transpose()*J_C_INV_T;


  double middle_time = ros::Time::now().toSec() - time_temp.toSec();








  Eigen::VectorXd torque_grav(MODEL_DOF);
  Eigen::MatrixXd aa = J_g*A_matrix_inverse*N_C*J_g.transpose();



  time_temp = ros::Time::now();



  double epsilon = 1e-6;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(aa ,Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(aa.cols(), aa.rows()) *svd.singularValues().array().abs()(0);
  Eigen::MatrixXd ppinv = svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();



 // Eigen::MatrixXd ppinv = aa.completeOrthogonalDecomposition().pseudoInverse();

  double pinv_time = ros::Time::now().toSec() - time_temp.toSec();





  time_temp = ros::Time::now();





  //torque_grav = (J_g*A_matrix.inverse()*N_C*J_g.transpose()).completeOrthogonalDecomposition().pseudoInverse()*J_g*A_matrix.inverse()*N_C*G;





  //torque_grav.setZero();
  Eigen::MatrixXd tg_temp;
  tg_temp = ppinv*J_g*A_matrix_inverse*N_C;
  torque_grav = tg_temp*G;






  double tg_time = ros::Time::now().toSec() - time_temp.toSec();


  ROS_INFO("\n\n Gravity Compensation : Calc time :::::::  gcalctime : %4.2f mtime : %4.2f  pinv calc time : %4.2f   tg time : %4.2f",g_calc_time*1000,middle_time*1000,pinv_time*1000,tg_time*1000);
  return torque_grav;
}
*/

/*
Eigen::VectorXd DyrosRedModel::GetDampingTorque(Eigen::VectorXd qdot, double damp_){

  Eigen::VectorXd T_damping;
  T_damping = -qdot*damp_;

  return T_damping;
}
*/


/*
void DyrosRedModel::setquat(Eigen::Quaterniond& quat,const Eigen::VectorXd& q)
{
  Eigen::VectorXd q_virtual;
  q_virtual.resize(MODEL_DOF+6);
  q_virtual = q;
  std::cout<<"q is : "<<std::endl;
  std::cout<<q<<std::endl;
  ROS_INFO_ONCE("t1");
  std::cout<<"quaterniond : " <<quat.x() <<"  "<< quat.y() <<"  "<<quat.z() <<"  "<< quat.w()<<std::endl;
  RigidBodyDynamics::Math::Quaternion quat_temp(quat.x(),quat.y(),quat.z(),quat.w());
  std::cout<<"quaternionv : " <<quat_temp<<std::endl;
  ROS_INFO_ONCE("t2");
  _model.SetQuaternion(link_[0].id, quat_temp, q_virtual);
  ROS_INFO_ONCE("t4");
  std::cout<<"after q is : "<<std::endl;
  std::cout<<q<<std::endl;
}

void DyrosRedModel::getCenterOfMassPosition(Eigen::Vector3d* position)
{
  RigidBodyDynamics::Math::Vector3d position_temp;
  position_temp.setZero();
  Eigen::VectorXd qdot(31);
  qdot.setZero();
  Eigen::Vector3d com_vel;
  Eigen::Vector3d angular_momentum;
  double mass;

  //RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_, qdot, mass, position_temp, NULL, NULL, false);
  //RigidBodyDynamics::Utils::CalcCenterOfMass(model_, q_, qdot, mass, position_temp);

  *position = position_temp;
}
*/



/*

void DyrosRedModel::getTransformEndEffector // must call updateKinematics before calling this function
(EndEffector ee, Eigen::Isometry3d* transform_matrix)
{
  Eigen::Vector3d gghg = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_,end_effector_id_[ee], base_position_, false);
  transform_matrix->translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates
      (model_, q_,end_effector_id_[ee], base_position_, false);
  transform_matrix->linear() = RigidBodyDynamics::CalcBodyWorldOrientation(
        model_, q_, end_effector_id_[ee], false).transpose();
}

void DyrosRedModel::getTransformEndEffector // must call updateKinematics before calling this function
(EndEffector ee, Eigen::Vector3d* position, Eigen::Matrix3d* rotation)
{
  *position = RigidBodyDynamics::CalcBodyToBaseCoordinates
      (model_, q_,end_effector_id_[ee], base_position_, false);
  *rotation = RigidBodyDynamics::CalcBodyWorldOrientation(
        model_, q_, end_effector_id_[ee], false).transpose();
}

void DyrosRedModel::getTransformEndEffector
(EndEffector ee, const Eigen::VectorXd& q, bool update_kinematics,
 Eigen::Vector3d* position, Eigen::Matrix3d* rotation)
{
  Eigen::Vector28d q_new;
  q_new = q_;
  switch (ee)
  {
  case EE_LEFT_FOOT:
  case EE_RIGHT_FOOT:
    q_new.segment<6>(joint_start_index_[ee]) = q;
    break;
  case EE_LEFT_HAND:
  case EE_RIGHT_HAND:
    q_new.segment<8>(joint_start_index_[ee]) = q;
    break;
  }
  if (update_kinematics)
  {
    q_ = q_new;
  }
  *position = RigidBodyDynamics::CalcBodyToBaseCoordinates
      (model_,q_new,end_effector_id_[ee], base_position_, update_kinematics);
  *rotation = RigidBodyDynamics::CalcBodyWorldOrientation(
        model_, q_new, end_effector_id_[ee], update_kinematics).transpose();
  // RigidBodyDynamics::Calcpo
  // model_.mBodies[0].mCenterOfMass
}


void DyrosRedModel::getJacobianMatrix6DoF
(EndEffector ee, Eigen::Matrix<double, 6, 6> *jacobian)
{
  Eigen::MatrixXd full_jacobian(6,MODEL_DOF);
  full_jacobian.setZero();
  RigidBodyDynamics::CalcPointJacobian6D(model_, q_, end_effector_id_[ee], Eigen::Vector3d::Zero(), full_jacobian, false);

  switch (ee)
  {
  case EE_LEFT_FOOT:
  case EE_RIGHT_FOOT:
    // swap
    jacobian->block<3, 6>(0, 0) = full_jacobian.block<3, 6>(3, joint_start_index_[ee]);
    jacobian->block<3, 6>(3, 0) = full_jacobian.block<3, 6>(0, joint_start_index_[ee]);
    break;
  case EE_LEFT_HAND:
  case EE_RIGHT_HAND:
  //*jacobian = full_jacobian.block<6, 7>(0, joint_start_index_[ee]);
    ROS_ERROR("Arm is 7 DoF. Please call getJacobianMatrix7DoF");
    break;
  }
}

void DyrosRedModel::getJacobianMatrix8DoF
(EndEffector ee, Eigen::Matrix<double, 6, 8> *jacobian)
{
  Eigen::MatrixXd full_jacobian(6,MODEL_DOF);
  full_jacobian.setZero();
  RigidBodyDynamics::CalcPointJacobian6D(model_, q_, end_effector_id_[ee],
                                         Eigen::Vector3d::Zero(), full_jacobian, false);

  switch (ee)
  {
  case EE_LEFT_FOOT:
  case EE_RIGHT_FOOT:
  // swap
  ROS_ERROR("Leg is 6 DoF. Please call getJacobianMatrix7DoF");
  break;
  case EE_LEFT_HAND:
  case EE_RIGHT_HAND:
  //*jacobian = full_jacobian.block<6, 7>(0, joint_start_index_[ee]);
  jacobian->block<3, 8>(0, 0) = full_jacobian.block<3, 8>(3, joint_start_index_[ee]);
  jacobian->block<3, 8>(3, 0) = full_jacobian.block<3, 8>(0, joint_start_index_[ee]);
  break;
  }
}

*/



}
