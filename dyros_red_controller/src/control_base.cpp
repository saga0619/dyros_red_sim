
#include "dyros_red_controller/control_base.h"

namespace dyros_red_controller
{

// Constructor
ControlBase::ControlBase(ros::NodeHandle &nh, double Hz) :
  ui_update_count_(0), is_first_boot_(true), Hz_(Hz), control_mask_{}, total_dof_(DyrosRedModel::MODEL_DOF)
{
  //walking_cmd_sub_ = nh.subscribe
  //makeIDInverseList();

  joint_state_pub_.init(nh, "/torque_jet/joint_state", 3);
  joint_state_pub_.msg_.name.resize(DyrosRedModel::MODEL_DOF);
  joint_state_pub_.msg_.angle.resize(DyrosRedModel::MODEL_DOF);
  joint_state_pub_.msg_.velocity.resize(DyrosRedModel::MODEL_DOF);
  joint_state_pub_.msg_.current.resize(DyrosRedModel::MODEL_DOF);
  joint_state_pub_.msg_.error.resize(DyrosRedModel::MODEL_DOF);
  //joint_state_pub_.msg_.effort.resize(DyrosRedModel::MODEL_DOF);

  for (int i=0; i< DyrosRedModel::MODEL_DOF; i++)
  {
    joint_state_pub_.msg_.name[i] = DyrosRedModel::JOINT_NAME[i];
    //joint_state_pub_.msg_.id[i] = DyrosRedModel::JOINT_ID[i];
  }

  smach_pub_.init(nh, "/transition", 1);
  smach_sub_ = nh.subscribe("/Jimin_machine/smach/container_status", 3, &ControlBase::smachCallback, this);
  //smach_sub_ = nh.subscribe("/torque_jet/smach/container_status", 3, &ControlBase::smachCallback, this);
  //task_comamnd_sub_ = nh.subscribe("/torque_jet/task_command", 3, &ControlBase::taskCommandCallback, this);

  parameterInitialize();
  WBCInitialize();
  QPInitialize();
  _binit_variables = false;
  _bstate_change = true;
}

bool ControlBase::checkStateChanged()
{
  if(previous_state_ != current_state_)
  {
    previous_state_ = current_state_;
    return true;
  }
  //task_controller_.compute();
  //task_controller_.updateControlMask(control_mask_);
  //task_controller_.writeDesired(control_mask_, desired_q_);



  return false;
}

void ControlBase::update()
{
  for(int i=0;i<DyrosRedModel::MODEL_DOF+1;i++)
  {
    _model.Link_pos_Update(i);
    _model.Link_rot_Update(i);
  }
  _kinematics_update = _model.updateKinematics(_q_virtual,_qdot_virtual);  // Update kinematics
  _dynamics_update = _model.updateDynamics(_kinematics_update); //update dynamics
}

void ControlBase::stateChangeEvent()
{
  if(checkStateChanged())
  {
    if(current_state_ == "move1")
    {

      /*
      task_controller_.setEnable(DyrosRedModel::EE_LEFT_HAND, true);
      task_controller_.setEnable(DyrosRedModel::EE_RIGHT_HAND, false);
      task_controller_.setEnable(DyrosRedModel::EE_LEFT_FOOT, false);
      task_controller_.setEnable(DyrosRedModel::EE_RIGHT_FOOT, false);

      Eigen::Isometry3d target;
      target.linear() = Eigen::Matrix3d::Identity();
      target.translation() << 1.0, 0.0, 1.0;
      task_controller_.setTarget(DyrosRedModel::EE_LEFT_HAND, target, 5.0);
      */
    }
  }
}

void ControlBase::compute()
{  

  //task_controller_.compute();
  //task_controller_.updateControlMask(control_mask_);
  //task_controller_.writeDesired(control_mask_, desired_q_);

  /* ---------------------- keep initial position -------------------------- */
  if(control_time_ < 3){

    _position_d(2) = (double)-30.0/180.0*3.141592;
    _position_d(3) = (double)60.0/180.0*3.141592;
    _position_d(4) = (double)-30.0/180.0*3.141592;
    _position_d(8) = (double)-30.0/180.0*3.141592;
    _position_d(9) = (double)60.0/180.0*3.141592;
    _position_d(10) = (double)-30.0/180.0*3.141592;


    _position_d(DyrosRedModel::joint_left_arm+1) = (double)0.0/180.0*3.141592;
    _position_d(DyrosRedModel::joint_left_arm+2) = (double)60.0/180.0*3.141592;
    _position_d(DyrosRedModel::joint_left_arm+3) = -(double)90.0/180.0*3.141592;
    _position_d(DyrosRedModel::joint_left_arm+4) = -(double)90.0/180.0*3.141592;

    _position_d(DyrosRedModel::joint_right_arm+1) = -(double)0.0/180*3.141592;
    _position_d(DyrosRedModel::joint_right_arm+2) = -(double)60.0/180*3.141592;
    _position_d(DyrosRedModel::joint_right_arm+3) = (double)90.0/180*3.141592;
    _position_d(DyrosRedModel::joint_right_arm+4) = (double)90.0/180*3.141592;

  }



  if(control_time_ > 0.6)
  {
    /* --------------------------------------------------------- */
    /* --------------------- torque control -------------------- */
    /* --------------------------------------------------------- */

    /* --------------------------------------------------------- */
    /* ----------------------- Initialize ---------------------- */
    /* --------------------------------------------------------- */
    int contactdof = 12;
    int taskdof = 18;
    if(_ContactState == 0) //double foot contact
    {
      contactdof = 12;
    }
    else if(_ContactState == 1 ||_ContactState == 2) //single foot contact
    {
      contactdof = 6;
    }
    const int contactdof_QP = contactdof;

    if(_TaskState == 0) //double foot contact
    {
      taskdof= 18;
    }
    else if(_TaskState == 1 ||_TaskState == 2) //single foot contact
    {
      taskdof= 25;
    }
    const int taskdof_QP = taskdof;


    //reset variables
    if(_preContactState != _ContactState || _preTaskState != _TaskState || _binit_variables == false)
    {
      //set Contact State
      _binit_variables = true;
      _bstate_change = true;

      WBCReset(contactdof,taskdof);
      QPReset(contactdof,taskdof);      
    }

    /* --------------------------------------------------------- */
    /* ------------------- whole-body dynamics ----------------- */
    /* --------------------------------------------------------- */

    if(_dynamics_update == true)
    {
      CWholeBodyControl WBCMain(31);
      WBCMain.updateGlobalDynamicsParameters(_model._G,_model._b,_model._A); //update dynamic info
      _Jc = setWBCContactJacobian(_ContactState);//Get predefined Contact Jacobian
      WBCMain.updateContactParameters(_Jc,_Jc_dot,_ContactState); //update contact Jacobian
      WBCMain.calculateContactSpaceInvDynamics(); //calculate contact space inverse dynamics      
      _pc = WBCMain._pc;
      WBCMain.calculateGravityCoriolisCompensationTorque();//calculate gravity compensation torque and coriolis centrifugal compensation torque
      _torque_gravity = WBCMain._GravityCompensationTorque;//get gravity compensation torque
      _J = setWBCTaskJacobian(_TaskState);
      _xdot = _J*_qdot_virtual;
      WBCMain.updateTaskParameters(_J,_TaskState);
      WBCMain.calculateContactConstrainedTaskJacobian(); //get Task Jacobian and Lambda matrix
      WBCMain.calculateSVDofWeightingMatrix();


      /* --------------------------------------------------------- */
      /* ------------------------ set tasks ---------------------- */
      /* --------------------------------------------------------- */

      //current state (position and orientation)
      if(_bool_init == false)
      {
        _init_pelvis_position = _model.link_[_model.Upper_Body].Link_position_global;
        _init_pelvis_rotation = _model.link_[_model.Upper_Body].RotationMatrix;

        _init_right_arm_position = _model.link_[_model.Right_Arm+7].Link_position_global;
        _init_right_arm_rotation = _model.link_[_model.Right_Arm+7].RotationMatrix;

        _init_left_arm_position = _model.link_[_model.Left_Arm+7].Link_position_global;
        _init_left_arm_rotation = _model.link_[_model.Left_Arm+7].RotationMatrix;

        _bool_init = true;
      }

      Matrix3d eye3;
      eye3.setZero();
      eye3(0,0) = 0.0;
      eye3(1,2) = 0.0;
      eye3(2,2) = 0.0;
      _desired_pelvis_rotation = eye3;//_init_pelvis_rotation;
      _desired_right_arm_rotation = eye3;//_init_right_arm_rotation;
      _desired_left_arm_rotation = eye3;//_init_left_arm_rotation;

      _desired_right_arm_position = _init_right_arm_position;
      _desired_left_arm_position = _init_left_arm_position;
      _desired_right_arm_velocity.setZero();
      _desired_left_arm_velocity.setZero();

      if(control_time_ < 0.8)
      {
        _desired_pelvis_position = _init_pelvis_position;
        _desired_pelvis_velocity.setZero();
      }
      else if(control_time_ >=0.8 && control_time_ <1.0)
      {
        _desired_pelvis_position(0) = cubic(control_time_,0.8,1.0,_init_pelvis_position(0),_init_pelvis_position(0)+0.0,0.0,0.0);
        _desired_pelvis_position(1) = cubic(control_time_,0.8,1.0,_init_pelvis_position(1),_init_pelvis_position(1)+0.05,0.0,0.0);
        _desired_pelvis_position(2) = cubic(control_time_,0.8,1.0,_init_pelvis_position(2),_init_pelvis_position(2),0.0,0.0);

        _desired_pelvis_velocity.setZero();
        //_desired_pelvis_velocity(0) = cubicDot(control_time_,0.8,1.0,_init_pelvis_position(0),_init_pelvis_position(0)+0.03,0.0,0.0,Hz_);
        //_desired_pelvis_velocity(1) = cubicDot(control_time_,0.8,1.0,_init_pelvis_position(1),_init_pelvis_position(1)+0.0,0.0,0.0,Hz_);
        //_desired_pelvis_velocity(2) = cubicDot(control_time_,0.8,1.0,_init_pelvis_position(2),_init_pelvis_position(2),0.0,0.0,Hz_);
      }
      else
      {
        _desired_pelvis_position(0) = _init_pelvis_position(0) + 0.0;
        _desired_pelvis_position(1) = _init_pelvis_position(1) + 0.05;
        _desired_pelvis_position(2) = _init_pelvis_position(2);
        _desired_pelvis_velocity.setZero();
      }

      _pelvis_position_error = _desired_pelvis_position - _model.link_[_model.Upper_Body].Link_position_global;
      _pelvis_orientation_error = -getPhi(_model.link_[_model.Upper_Body].RotationMatrix, _desired_pelvis_rotation);

      _right_arm_position_error = _desired_right_arm_position - _model.link_[_model.Right_Arm+7].Link_position_global;
      _right_arm_orientation_error = -getPhi(_model.link_[_model.Right_Arm+7].RotationMatrix, _desired_right_arm_rotation);

      _left_arm_position_error = _desired_left_arm_position - _model.link_[_model.Left_Arm+7].Link_position_global;
      _left_arm_orientation_error = -getPhi(_model.link_[_model.Left_Arm+7].RotationMatrix, _desired_left_arm_rotation);

      //cout << "desired pos" << endl << _desired_pelvis_position << endl <<endl;
      //cout << "current pos" << endl << _model.link_[_model.Pelvis].Link_position_global << endl <<endl;
      //cout << "error pos" << endl << _pelvis_position_error << endl <<endl;

      double kp = 400.0;
      double kd = 40.0;

      if(_TaskState == 0)
      {
        for(int i=0; i<3; i++)
        {
          _fstar(i) = kp*_pelvis_position_error(i) + kd*(_desired_pelvis_velocity(i) - _xdot(i));
          _fstar(i+3) = kp*_pelvis_orientation_error(i) - kd*_xdot(i+3);
          _fstar(i+6) = kp*_right_arm_position_error(i) + kd*(_desired_right_arm_velocity(i) - _xdot(i+6));
          _fstar(i+9) = kp*_right_arm_orientation_error(i) - kd*_xdot(i+9);
          _fstar(i+12) = kp*_left_arm_position_error(i) + kd*(_desired_left_arm_velocity(i) - _xdot(i+12));
          _fstar(i+15) = kp*_left_arm_orientation_error(i) - kd*_xdot(i+15);
        }

        for(int i=0; i<3; i++)
        {
          _kpx_kdxdot_Vector(i) = kp*_model.link_[_model.Upper_Body].Link_position_global(i) +kd*_xdot(i);
          _kpx_kdxdot_Vector(i+3) = kp*_pelvis_orientation_error(i) +kd*_xdot(i+3);
          _kpx_kdxdot_Vector(i+6) = kp*_model.link_[_model.Right_Arm+7].Link_position_global(i) +kd*_xdot(i+6);
          _kpx_kdxdot_Vector(i+9) = kp*_right_arm_orientation_error(i) +kd*_xdot(i+9);
          _kpx_kdxdot_Vector(i+12) = kp*_model.link_[_model.Left_Arm+7].Link_position_global(i) +kd*_xdot(i+12);
          _kpx_kdxdot_Vector(i+15) = kp*_left_arm_orientation_error(i) +kd*_xdot(i+15);

          _xd_Vector(i) = _desired_pelvis_position(i);
          _xd_Vector(i+3) = 0.0;
          _xd_Vector(i+6) = _desired_right_arm_position(i);
          _xd_Vector(i+9) = 0.0;
          _xd_Vector(i+12) = _desired_left_arm_position(i);
          _xd_Vector(i+15) = 0.0;
        }
        _kp_Matrix.resize(taskdof,taskdof);
        _kp_Matrix.setZero();
        for(int i=0; i<18; i++)
        {
          _kp_Matrix(i,i) = kp;
        }

      }

      /* --------------------------------------------------------- */
      /* -------------  Task Modification - QP base -------------- */
      /* --------------------------------------------------------- */

      ros::Time time_temp = ros::Time::now();

      _torque_task = WBCMain._J_k_T_Lambda*_fstar;
      WBCMain.calculateContactWrenchLocalContactFrame(_torque_gravity + _torque_task); //caculate contact wrench
      _Fc_LocalContactFrame = WBCMain._Fc_LocalContactFrame;

      if(_ContactState == 0) //double foot contact
      {
        Vector3d P1;
        P1 = _model.link_[_model.Right_Leg+5].Contact_position - (_model.link_[_model.Right_Leg+5].Contact_position +_model.link_[_model.Left_Leg+5].Contact_position)/2.0;
        Vector3d P2;
        P2 = _model.link_[_model.Left_Leg+5].Contact_position - (_model.link_[_model.Right_Leg+5].Contact_position +_model.link_[_model.Left_Leg+5].Contact_position)/2.0;
        Matrix3d RotFoot;
        RotFoot.setZero();
        double thetaTwoFeet;
        if(P1(0)>P2(0))
        {
          thetaTwoFeet = atan(abs(P1(0) - P2(0))/abs(P1(1) - P2(1)));
        }
        else //-RotateTwoFeetžžÅ­ ÈžÀü
        {
          thetaTwoFeet = -atan(abs(P1(0) - P2(0))/abs(P1(1) - P2(1)));
        }
        RotFoot(0,0) = cos(-thetaTwoFeet);
        RotFoot(0,1) = -sin(-thetaTwoFeet);
        RotFoot(1,0) = sin(-thetaTwoFeet);
        RotFoot(1,1) = cos(-thetaTwoFeet);
        RotFoot(2,2) = 1.0;

        MatrixXd Res(6,12);
        Res.setZero();

        double foot_length = 0.25;
        double foot_width = 0.12;
        Vector3d difference_leftfoot_to_rightfoot;
        difference_leftfoot_to_rightfoot = _model.link_[_model.Right_Leg+5].Contact_position - _model.link_[_model.Left_Leg+5].Contact_position;
        double feet_distance = sqrt(difference_leftfoot_to_rightfoot(0)*difference_leftfoot_to_rightfoot(0) + difference_leftfoot_to_rightfoot(1)*difference_leftfoot_to_rightfoot(1) + difference_leftfoot_to_rightfoot(2)*difference_leftfoot_to_rightfoot(2));

        double width_boundary_cop = feet_distance/2.0 + foot_width * 0.8; //0.8: safety ratio
        double length_boundary_cop = foot_length/2.0 * 0.8; //0.8: safety ratio

        VectorXd ResultantWrench(6);
        ResultantWrench.setZero();
        ResultantWrenchTwoContact(P1,P2,RotFoot,WBCMain._Fc_LocalContactFrame,Res,ResultantWrench);        

        double Fz = ResultantWrench(2);
        double static_fric_coeff = 0.8;

        MatrixXd Res_Jc_bar_T_S_k_T(6,DyrosRedModel::MODEL_DOF);
        Res_Jc_bar_T_S_k_T = Res*WBCMain._Jc_bar_T_S_k_T;
        MatrixXd Res_Jc_bar_T_S_k_T_J_k_T_Lambda(6,taskdof);
        Res_Jc_bar_T_S_k_T_J_k_T_Lambda = Res_Jc_bar_T_S_k_T*WBCMain._J_k_T*WBCMain._Lambda;

        _A_task_modification = Res_Jc_bar_T_S_k_T_J_k_T_Lambda*_kp_Matrix;

        VectorXd qp_b1(6);
        qp_b1 = -Res_Jc_bar_T_S_k_T_J_k_T_Lambda*_kpx_kdxdot_Vector;
        VectorXd qp_b2(6);
        qp_b2 = Res_Jc_bar_T_S_k_T*_torque_gravity;
        VectorXd qp_b3(6);
        qp_b3 = -Res*(WBCMain._pc);
        VectorXd qp_b(6);
        qp_b = qp_b1 + qp_b2 + qp_b3;

        _H_task_modification.setZero();
        for(int i=0; i<3; i++) //trunk pos
        {
          _H_task_modification(i,i) = 1000.0;
        }
        for(int i=3; i<taskdof; i++) //else
        {
          _H_task_modification(i,i) = 500.0;
        }        

        //_g_task_modification = -1.0*qp_H*_fstar;
        _g_task_modification = -1.0*_H_task_modification*_xd_Vector;

        _ubA_task_modification(0) = -static_fric_coeff*Fz-qp_b(0);
        _ubA_task_modification(1) = -static_fric_coeff*Fz-qp_b(1);
        _ubA_task_modification(2) = Fz-qp_b(2);
        _ubA_task_modification(3) = -width_boundary_cop*Fz-qp_b(3);
        _ubA_task_modification(4) = -length_boundary_cop*Fz-qp_b(4);
        _ubA_task_modification(5) = -static_fric_coeff*Fz-qp_b(5);

        _lbA_task_modification(0) = static_fric_coeff*Fz-qp_b(0);
        _lbA_task_modification(1) = static_fric_coeff*Fz-qp_b(1);
        _lbA_task_modification(2) = Fz-qp_b(2);
        _lbA_task_modification(3) = width_boundary_cop*Fz-qp_b(3);
        _lbA_task_modification(4) = length_boundary_cop*Fz-qp_b(4);
        _lbA_task_modification(5) = static_fric_coeff*Fz-qp_b(5);
      }
      else //single foot contact
      {
      }

      double QP_setup_time = ros::Time::now().toSec() - time_temp.toSec();
      cout <<"setup time" << QP_setup_time << endl;      

      _QP_task_modification.UpdateMinProblem(_H_task_modification,_g_task_modification);
      _QP_task_modification.UpdateSubjectToAx(_A_task_modification,_lbA_task_modification,_ubA_task_modification);

      time_temp = ros::Time::now();
      _Sol_task_modification = _QP_task_modification.SolveQPoases(_nIter);
      double QP_test_time = ros::Time::now().toSec() - time_temp.toSec();
      cout <<"qpoases time" << QP_test_time << endl;
      //cout << "qpoases" <<endl << _Sol_task_modification.transpose() <<endl;
      //cout << "xd" <<endl << _xd_Vector.transpose() << endl <<endl;

      /* --------------------------------------------------------- */
      /* --------------------------------------------------------- */


      _torque_task = WBCMain._J_k_T_Lambda*_fstar;
      //_torque_task =WBCMain._J_k_T*WBCMain._Lambda*fstar_Opt;

      /* --------------------------------------------------------- */
      /* ------------ Contact Wrench Redistribution -------------- */
      /* --------------------------------------------------------- */

      _torque_contact.setZero();
      if(_ContactState == 0) //for double foot contact
      {
        //get redistributed contact wrench
        double foot_length = 0.3;
        double foot_width = 0.15;
        Vector3d P1;
        P1 = _model.link_[_model.Right_Leg+5].Contact_position - (_model.link_[_model.Right_Leg+5].Contact_position +_model.link_[_model.Left_Leg+5].Contact_position)/2.0;
        Vector3d P2;
        P2 = _model.link_[_model.Left_Leg+5].Contact_position - (_model.link_[_model.Right_Leg+5].Contact_position +_model.link_[_model.Left_Leg+5].Contact_position)/2.0;

        int contact_num = 2; //two plane contact
        Matrix3d Rot1;
        Rot1 = _model.link_[_model.Right_Leg+5].RotationMatrix;
        Matrix3d Rot2;
        Rot2 = _model.link_[_model.Left_Leg+5].RotationMatrix;

        VectorXd P_contact;
        P_contact.resize(6);
        P_contact.head(3) = P1;
        P_contact.tail(3) = P2;

        MatrixXd Rot_contact;
        Rot_contact.resize(3*contact_num,3);
        Rot_contact.block<3,3>(0,0) = Rot1;
        Rot_contact.block<3,3>(3,0) = Rot2;

        VectorXd frictionCoeff;
        frictionCoeff.resize(contact_num);
        frictionCoeff(0) = 0.8;
        frictionCoeff(1) = 0.8;

        VectorXd CoP_boundary;
        CoP_boundary.resize(contact_num*2);
        CoP_boundary(0) = foot_length/2.0;
        CoP_boundary(1) = foot_width/2.0;
        CoP_boundary(2) = foot_length/2.0;
        CoP_boundary(3) = foot_width/2.0;

        VectorXd Fc_redeistributed;
        Fc_redeistributed.resize(contact_num*6);

        time_temp = ros::Time::now();
        DefineQP_PrimalContactWrenchDistributionForMultiContact(_QP_contactwrench_distribution,contact_num,P_contact,Rot_contact,_Fc_LocalContactFrame);
        Fc_redeistributed = _QP_contactwrench_distribution.SolveQPoases(_nIter);
        VectorXd Fz_redistributed(contact_num);
        for(int i=0; i<contact_num; i++)
        {
          Fz_redistributed(i) = Fc_redeistributed(2+i*6);
        }
        DefineQP_ContactWrenchDistributionForMultiContactInequality(_QP_contactwrench_distribution_mod,contact_num,P_contact,Rot_contact,frictionCoeff,CoP_boundary,_Fc_LocalContactFrame,Fz_redistributed);
        Fc_redeistributed = _QP_contactwrench_distribution_mod.SolveQPoases(_nIter);
        double QP_Fc_time = ros::Time::now().toSec() - time_temp.toSec();
        cout << "QP_Fc time"<< QP_Fc_time << endl<<endl;

        VectorXd ResultantWrench(6);
        ResultantWrench.setZero();
        VectorXd RedistributionWrench(12);
        RedistributionWrench.setZero();
        double eta_Fc = 0.0;

        time_temp = ros::Time::now();
        WrenchRedistributionTwoFootContact(0.95,foot_length,foot_width,1.0,0.8,0.8,P1,P2,WBCMain._Fc_LocalContactFrame, ResultantWrench, RedistributionWrench, eta_Fc);
        double QP_Fc_time2 = ros::Time::now().toSec() - time_temp.toSec();
        cout << "Fc_time2 time"<< QP_Fc_time2 << endl<<endl;

        VectorXd Fc_compensation_d(12);
        Fc_compensation_d.setZero();
        MatrixXd Scw(6,12); //contact wrench selection matrix
        Scw.setZero();
        for(int i=0; i<6; i++)
        {
          Fc_compensation_d(i+6) = -WBCMain._Fc_LocalContactFrame(i+6)+RedistributionWrench(i+6);
          Scw(i,i+6) = 1.0;
        }
        VectorXd torque_contact(DyrosRedModel::MODEL_DOF+6);
        WBCMain.calculateContactWrenchRedistributionTorque(Scw,Fc_compensation_d,torque_contact);
        _torque_contact = torque_contact;

        //cout << "RedistributionWrench"  << endl << RedistributionWrench <<endl<<endl;
        //cout << "Fc_redeistributed"  << endl << Fc_redeistributed <<endl<<endl;


      }

       /* --------------------------------------------------------- */
       /* --------------------------------------------------------- */
    }

    _torque_d = _torque_gravity + _torque_task + _torque_contact;// - _qdot*1.0;

    cout <<"torque" <<endl << _torque_d.transpose() <<endl <<endl;
  }

  /* --------------------------------------------------------------------- */

  _preContactState = _ContactState;
  _preTaskState = _TaskState;
  _bstate_change = false;


  tick_ ++;
  control_time_ = tick_ / Hz_;


  if ((tick_ % 200) == 0 )
  {
    ROS_INFO ("control time, %lf sec", control_time_);
  }
}

/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */

void ControlBase::reflect()
{
  for (int i=0; i<DyrosRedModel::MODEL_DOF; i++)
  {
    joint_state_pub_.msg_.angle[i] = _q(i);
    joint_state_pub_.msg_.velocity[i] = _qdot(i);
    joint_state_pub_.msg_.current[i] = _torque(i);
  }

  if(joint_state_pub_.trylock())
  {
    joint_state_pub_.unlockAndPublish();
  }
}

/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */

void ControlBase::parameterInitialize()
{
  ROS_INFO_ONCE("parameter initialize");
  _q.setZero();
  _qdot.setZero();
  _q_virtual.setZero();
  _qdot_virtual.setZero();
  _torque.setZero();
  _FT_left_foot.setZero();
  _FT_right_foot.setZero();
  _q_d.setZero();
  _torque_d.setZero();
  _position_d.setZero();  

  _init_pelvis_position.setZero();
  _init_pelvis_rotation.setZero();
  _init_right_arm_position.setZero();
  _init_right_arm_rotation.setZero();
  _init_left_arm_position.setZero();
  _init_left_arm_rotation.setZero();

  _desired_pelvis_position.setZero();
  _desired_pelvis_velocity.setZero();
  _desired_pelvis_rotation.setZero();
  _desired_left_arm_position.setZero();
  _desired_left_arm_velocity.setZero();
  _desired_left_arm_rotation.setZero();
  _desired_right_arm_position.setZero();
  _desired_right_arm_velocity.setZero();
  _desired_right_arm_rotation.setZero();
  _desired_left_foot_position.setZero();
  _desired_left_foot_rotation.setZero();
  _desired_right_foot_position.setZero();
  _desired_right_foot_rotation.setZero();

  _pelvis_position_error.setZero();
  _pelvis_velocity_error.setZero();
  _pelvis_orientation_error.setZero();
  _left_arm_position_error.setZero();
  _left_arm_velocity_error.setZero();
  _left_arm_orientation_error.setZero();
  _right_arm_position_error.setZero();
  _right_arm_velocity_error.setZero();
  _right_arm_orientation_error.setZero();
  _left_foot_position_error.setZero();
  _left_foot_orientation_error.setZero();
  _right_foot_position_error.setZero();
  _right_foot_orientation_error.setZero();

  _xdot.resize(6);
  _xdot.setZero();
}

/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */

void ControlBase::WBCInitialize()
{
  ROS_INFO_ONCE("WBC initialize");

  _ContactState = 0;
  _TaskState = 0;
  _preContactState = 1000000; //initial value of_preContactState and _ContactState should be diffrent
  _preTaskState = 1000000;//initial value _preTaskState and _TaskState should be diffrent
  _Jc.resize(12,DyrosRedModel::MODEL_DOF+6);
  _Jc.setZero();
  _Jc_dot.resize(12,DyrosRedModel::MODEL_DOF+6);
  _Jc_dot.setZero();
  _J.resize(6,DyrosRedModel::MODEL_DOF+6);
  _J.setZero();  

  _pc.resize(12);
  _pc.setZero();
  _fstar.resize(6);
  _fstar.setZero();  
  _Fc_LocalContactFrame.resize(12);
  _Fc_LocalContactFrame.setZero();

  _torque_gravity.setZero();
  _torque_task.setZero();
  _torque_contact.setZero();
}

void ControlBase::WBCReset(const int &contactdof, const int &taskdof) //call this function when contact and task state change
{
  _Jc.resize(contactdof,DyrosRedModel::MODEL_DOF+6);
  _Jc.setZero();
  _Jc_dot.resize(contactdof,DyrosRedModel::MODEL_DOF+6);
  _Jc_dot.setZero();
  _J.resize(taskdof,DyrosRedModel::MODEL_DOF+6);
  _J.setZero();
  _pc.resize(contactdof);
  _pc.setZero();
  _fstar.resize(taskdof);
  _fstar.setZero();
  _Fc_LocalContactFrame.resize(contactdof);
  _Fc_LocalContactFrame.setZero();
}

/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */

void ControlBase::QPInitialize() //call this function once
{  
  _nIter = 1000;

  _H_task_modification.resize(18,18);
  _H_task_modification.setZero();
  _g_task_modification.resize(18);
  _g_task_modification.setZero();
  _A_task_modification.resize(6,18);
  _A_task_modification.setZero();
  _lbA_task_modification.resize(6);
  _lbA_task_modification.setZero();
  _ubA_task_modification.resize(6);
  _ubA_task_modification.setZero();
  _kp_Matrix.resize(18,18);
  _kp_Matrix.setZero();
  _kpx_kdxdot_Vector.resize(18);
  _kpx_kdxdot_Vector.setZero();
  _xd_Vector.resize(18);
  _xd_Vector.setZero();
  _Sol_task_modification.resize(18);
  _Sol_task_modification.setZero();
}

void ControlBase::QPReset(const int &contactdof, const int &taskdof) //call this function when contact and task state change
{
  _QP_task_modification.InitializeProblemSize(taskdof,6);
  _H_task_modification.resize(taskdof,taskdof);
  _H_task_modification.setZero();
  _g_task_modification.resize(taskdof);
  _g_task_modification.setZero();
  _A_task_modification.resize(6,taskdof);
  _A_task_modification.setZero();
  _lbA_task_modification.resize(6);
  _lbA_task_modification.setZero();
  _ubA_task_modification.resize(6);
  _ubA_task_modification.setZero();
  _kp_Matrix.resize(taskdof,taskdof);
  _kp_Matrix.setZero();
  _kpx_kdxdot_Vector.resize(taskdof);
  _kpx_kdxdot_Vector.setZero();
  _xd_Vector.resize(taskdof);
  _xd_Vector.setZero();
  _Sol_task_modification.resize(taskdof);
  _Sol_task_modification.setZero();

  _QP_contactwrench_distribution.InitializeProblemSize(contactdof,6);
  _QP_contactwrench_distribution_mod.InitializeProblemSize(contactdof,6);
}

/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */

MatrixXd ControlBase::setWBCContactJacobian(const int ContactState)//-0.1368;
{
  MatrixXd Jc;
  Vector3d left_foot_contact;
  left_foot_contact(0) = 0.0;
  left_foot_contact(1) = 0.0;
  left_foot_contact(2) = -0.1675;//-0.1368;
  Vector3d right_foot_contact;
  right_foot_contact(0) = 0.0;//0.03;
  right_foot_contact(1) = 0.0;
  right_foot_contact(2) = -0.1675;//-0.1368;

  int contact_dof = 12;
  if(ContactState == 0) //double foot contact
  {
    contact_dof = 12;
    Jc.resize(12,DyrosRedModel::MODEL_DOF+6);
    Jc.setZero();
    //translate coordiante
    _model.Link_J_Contact_Position(_model.Right_Leg+5, right_foot_contact);
    _model.Link_J_Contact_Position(_model.Left_Leg+5, left_foot_contact);
    //
    Jc.block(0, 0, 6, DyrosRedModel::MODEL_DOF+6) = _model.link_[_model.Right_Leg+5].J_Contact;
    Jc.block(6, 0, 6, DyrosRedModel::MODEL_DOF+6) = _model.link_[_model.Left_Leg+5].J_Contact;
  }
  else if(ContactState == 1) //left foot contact
  {
    contact_dof = 6;
    Jc.resize(6,DyrosRedModel::MODEL_DOF+6);
    Jc.setZero();
    _model.Link_J_Contact_Position(_model.Left_Leg+5, left_foot_contact);
    //
    Jc.block(0, 0, 6, DyrosRedModel::MODEL_DOF+6) = _model.link_[_model.Left_Leg+5].J_Contact;
  }
  else if(ContactState == 2) //right foot contact
  {
    contact_dof = 6;
    Jc.resize(6,DyrosRedModel::MODEL_DOF+6);
    Jc.setZero();
    _model.Link_J_Contact_Position(_model.Right_Leg+5, right_foot_contact);
    //
    Jc.block(0, 0, 6, DyrosRedModel::MODEL_DOF+6) = _model.link_[_model.Right_Leg+5].J_Contact;    
  }
  //_Jc_dot.resize(contact_dof,DyrosRedModel::MODEL_DOF+6);
  //_pc.resize(contact_dof);
  //_pc.setZero();
  //Fc_LocalContactFrame.resize(contact_dof);
  //_Fc_LocalContactFrame.setZero();

  return Jc;
}

/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */

MatrixXd ControlBase::setWBCTaskJacobian(const int TaskState)
{
  MatrixXd J;
  int task_dof = 18;

  if(TaskState == 0)
  {
    task_dof= 18;

    J.resize(task_dof,DyrosRedModel::MODEL_DOF+6);
    J.setZero();
    Vector3d control_local_position_pelvis;
    control_local_position_pelvis.setZero();

    _model.Link_J_Custom_Position(_model.Upper_Body,control_local_position_pelvis);
    J.block(0, 0, 3, 3) = _model.link_[_model.Upper_Body].J.block(0,0,3,3);
    J.block(3, 0, 3, 6) = _model.link_[_model.Upper_Body].J.block(3,0,3,6);
    J.block(0, 6, 6, DyrosRedModel::MODEL_DOF) = _model.link_[_model.Upper_Body].J.block(0, 6, 6, DyrosRedModel::MODEL_DOF) - _model.link_[_model.Right_Leg+5].J_Contact.block(0, 6, 6, DyrosRedModel::MODEL_DOF);
    J.block(0, 3, 3, 3) = _model.link_[_model.Upper_Body].J.block(0, 3, 3, 3) - _model.link_[_model.Right_Leg+5].J_Contact.block(0, 3, 3, 3);

    Vector3d control_local_position_rightarm;
    control_local_position_rightarm.setZero();

    _model.Link_J_Custom_Position(_model.Right_Arm+7,control_local_position_rightarm);
    J.block(6, 0, 3, 3) = _model.link_[_model.Right_Arm+7].J.block(0,0,3,3);
    J.block(9, 0, 3, 6) = _model.link_[_model.Right_Arm+7].J.block(3,0,3,6);
    J.block(6, 6, 6, DyrosRedModel::MODEL_DOF) = _model.link_[_model.Right_Arm+7].J.block(0, 6, 6, DyrosRedModel::MODEL_DOF) - _model.link_[_model.Right_Leg+5].J_Contact.block(0, 6, 6, DyrosRedModel::MODEL_DOF);
    J.block(6, 3, 3, 3) = _model.link_[_model.Right_Arm+7].J.block(0, 3, 3, 3) - _model.link_[_model.Right_Leg+5].J_Contact.block(0, 3, 3, 3);

    Vector3d control_local_position_leftarm;
    control_local_position_leftarm.setZero();

    _model.Link_J_Custom_Position(_model.Left_Arm+7,control_local_position_leftarm);
    J.block(12, 0, 3, 3) = _model.link_[_model.Left_Arm+7].J.block(0,0,3,3);
    J.block(15, 0, 3, 6) = _model.link_[_model.Left_Arm+7].J.block(3,0,3,6);
    J.block(12, 6, 6, DyrosRedModel::MODEL_DOF) = _model.link_[_model.Left_Arm+7].J.block(0, 6, 6, DyrosRedModel::MODEL_DOF) - _model.link_[_model.Right_Leg+5].J_Contact.block(0, 6, 6, DyrosRedModel::MODEL_DOF);
    J.block(12, 3, 3, 3) = _model.link_[_model.Left_Arm+7].J.block(0, 3, 3, 3) - _model.link_[_model.Right_Leg+5].J_Contact.block(0, 3, 3, 3);
    //cout << J << endl <<endl;
  }
  else if(TaskState == 1)
  {
    task_dof = 24;
    J.resize(task_dof,DyrosRedModel::MODEL_DOF+6);
    J.setZero();
  }
  else if(TaskState == 2)
  {
    task_dof = 24;
    J.resize(task_dof,DyrosRedModel::MODEL_DOF+6);
    J.setZero();
  }

  //_fstar.resize(task_dof);
  //_xdot.resize(task_dof);

  return J;
}

/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */

void ControlBase::readDevice()
{
  ros::spinOnce();
}

/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */

void ControlBase::smachCallback(const smach_msgs::SmachContainerStatusConstPtr& msg)
{
  current_state_ = msg->active_states[0];
}

/*
void ControlBase::taskCommandCallback(const dyros_red_msgs::TaskCommandConstPtr& msg)
{
  for(unsigned int i=0; i<4; i++)
  {
    if(msg->end_effector[i])
    {
      Eigen::Isometry3d target;
      tf::poseMsgToEigen(msg->pose[i], target);

      if(msg->mode[i] == dyros_red_msgs::TaskCommand::RELATIVE)
      {
        const auto &current =  model_.getCurrentTrasmfrom((DyrosRedModel::EndEffector)i);
        target.translation() = target.translation() + current.translation();
        target.linear() = current.linear() * target.linear();
      }
      task_controller_.setTarget((DyrosRedModel::EndEffector)i, target, msg->duration[i]);
      task_controller_.setEnable((DyrosRedModel::EndEffector)i, true);
    }
  }
}
*/

/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */

void ControlBase::ResultantWrenchTwoContact(const Vector3D& P1, const Vector3D& P2, const Matrix3d& Rot, const VectorXd& F12, MatrixXd &W, VectorXd& ResultantForce)
{
  W.resize(6,12);
  W.setZero();
  VectorXd F1F2(12);
  for(int i =0; i<3; i++)
  {
    W(i,i) = 1.0;
    W(i,i+6) = 1.0;
    W(i+3,i+3) = 1.0;
    W(i+3,i+9) = 1.0;

    F1F2(i) = F12(i);
    F1F2(i+3) = F12(i+3);
    F1F2(i+6) = F12(i+6);
    F1F2(i+9) = F12(i+9);
  }

  Matrix3d P1_hat;
  P1_hat = skm(P1);
  Matrix3d P2_hat;
  P2_hat = skm(P2);
  for(int i=0; i<3; i++)
  {
    for(int j=0; j<3; j++)
    {
      W(i+3,j) = P1_hat(i,j);
      W(i+3,j+6) = P2_hat(i,j);
    }
  }

  Matrix6d RotMat;
  RotMat.setZero();
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3;j++)
    {
      RotMat(i,j)= Rot(i,j);
      RotMat(i+3,j+3)=Rot(i,j);
    }
  }
  W = RotMat*W;

  ResultantForce.resize(6);
  ResultantForce = W*F1F2;
}

/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */

void ControlBase::DefineQP_PrimalContactWrenchDistributionForMultiContact(CQuadraticProgram &QPprog, const int &plane_contact_num, const VectorXd &arb_P_dis, const MatrixXd &arb_Rot, const VectorXd Fc_LocalContactFrame)
{
    const int Fc_dof = plane_contact_num*6;
    MatrixXd Wmat;
    Wmat.resize(6,Fc_dof);
    Wmat.setZero();
    Vector6d Fc_res;
    Fc_res.setZero();

    Matrix3d Phat;
    Phat.setZero();
    Matrix3d PhatR;
    for(int k=0; k<plane_contact_num; k++)
    {
      Wmat.block<3,3>(0,k*6) = arb_Rot.block<3,3>(3*k,0);
      Wmat.block<3,3>(3,k*6+3) = arb_Rot.block<3,3>(3*k,0);

      Phat(0,1) = -arb_P_dis(2+k*3);
      Phat(0,2) = arb_P_dis(1+k*3);
      Phat(1,0) = arb_P_dis(2+k*3);
      Phat(1,2) = -arb_P_dis(0+k*3);
      Phat(2,0) = -arb_P_dis(1+k*3);
      Phat(2,1) = arb_P_dis(0+k*3);
      PhatR = Phat*arb_Rot.block<3,3>(3*k,0);
      Wmat.block<3,3>(3,k*6) = PhatR;
    }

    Fc_res = Wmat*Fc_LocalContactFrame;

    //set QP problem
    MatrixXd qp_H(Fc_dof,Fc_dof);
    qp_H.setZero();
    for(int i=0; i<Fc_dof; i++)
    {
      qp_H(i,i) = 1.0;
    }
    for(int i=0; i<plane_contact_num; i++)
    {
      qp_H(0+6*i,0+6*i) = 100.0;//*friction_coeff(i);//contact force x
      qp_H(1+6*i,1+6*i) = 100.0;//*friction_coeff(i);//contact force y
      qp_H(2+6*i,2+6*i) = 0.0;//*friction_coeff(i);//contact force z
      qp_H(3+6*i,3+6*i) = 10000000.0;//*CoP_boundary(i*2);//contact moment x
      qp_H(4+6*i,4+6*i) = 10000000.0;//*CoP_boundary(1+i*2);//contact moment y
      qp_H(5+6*i,5+6*i) = 10000000.0;//*friction_coeff(i);//contact moment z
    }
    MatrixXd qp_A(6,Fc_dof);
    qp_A = Wmat;
    VectorXd qp_g(Fc_dof);
    qp_g.setZero();
    VectorXd qp_lbA(6);
    qp_lbA = Fc_res;
    VectorXd qp_ubA(6);
    qp_ubA = Fc_res;

    VectorXd qp_lb(Fc_dof);
    VectorXd qp_ub(Fc_dof);
    for(int i=0; i<plane_contact_num; i++)
    {
      qp_lb(0+6*i) = -1000.0;
      qp_lb(1+6*i) = -1000.0;
      qp_lb(2+6*i) = -2000.0;
      qp_lb(3+6*i) = -1000.0;
      qp_lb(4+6*i) = -1000.0;
      qp_lb(5+6*i) = -1000.0;

      qp_ub(0+6*i) = 1000.0;
      qp_ub(1+6*i) = 1000.0;
      qp_ub(2+6*i) = 0.0;//0.0;
      qp_ub(3+6*i) = 1000.0;
      qp_ub(4+6*i) = 1000.0;
      qp_ub(5+6*i) = 1000.0;
    }

    QPprog.EnableEqualityCondition(0.0001);
    QPprog.UpdateMinProblem(qp_H,qp_g);
    QPprog.UpdateSubjectToAx(qp_A,qp_lbA,qp_ubA);
    QPprog.UpdateSubjectToX(qp_lb,qp_ub);
}

void ControlBase::DefineQP_ContactWrenchDistributionForMultiContactInequality(CQuadraticProgram &QPprog, const int &plane_contact_num, const VectorXd &arb_P_dis, const MatrixXd &arb_Rot, const VectorXd &friction_coeff, const VectorXd &CoP_boundary, const VectorXd Fc_LocalContactFrame, const VectorXd Fz_local_decided)
{
  const int Fc_dof = plane_contact_num*6;
  MatrixXd Wmat;
  Wmat.resize(6,Fc_dof);
  Wmat.setZero();
  Vector6d Fc_res;
  Fc_res.setZero();

  Matrix3d Phat;
  Phat.setZero();
  Matrix3d PhatR;
  for(int k=0; k<plane_contact_num; k++)
  {
    Wmat.block<3,3>(0,k*6) = arb_Rot.block<3,3>(3*k,0);
    Wmat.block<3,3>(3,k*6+3) = arb_Rot.block<3,3>(3*k,0);

    Phat(0,1) = -arb_P_dis(2+k*3);
    Phat(0,2) = arb_P_dis(1+k*3);
    Phat(1,0) = arb_P_dis(2+k*3);
    Phat(1,2) = -arb_P_dis(0+k*3);
    Phat(2,0) = -arb_P_dis(1+k*3);
    Phat(2,1) = arb_P_dis(0+k*3);
    PhatR = Phat*arb_Rot.block<3,3>(3*k,0);
    Wmat.block<3,3>(3,k*6) = PhatR;
  }

  Fc_res = Wmat*Fc_LocalContactFrame;


  //set QP problem
  double tot_Fz = 0.0;
  for(int i=0; i<plane_contact_num; i++)
  {
    tot_Fz = tot_Fz + Fz_local_decided(i);
  }
  VectorXd Fz_ratio(plane_contact_num);
  VectorXd qp_b(Fc_dof);
  qp_b.setZero();
  for(int i=0; i<plane_contact_num; i++)
  {
    Fz_ratio(i) = (1.0- Fz_local_decided(i)/tot_Fz)*10.0;
    qp_b(2+i*6) = Fz_local_decided(i);
  }

  MatrixXd qp_H(Fc_dof,Fc_dof);
  qp_H.setZero();
  for(int i=0; i<Fc_dof; i++)
  {
    qp_H(i,i) = 1.0;
  }
  for(int i=0; i<plane_contact_num; i++)
  {
    qp_H(0+6*i,0+6*i) = 10000.0* Fz_ratio(i);//*friction_coeff(i);//contact force x
    qp_H(1+6*i,1+6*i) = 10000.0* Fz_ratio(i);//*friction_coeff(i);//contact force y
    qp_H(2+6*i,2+6*i) = 10000.0 * 10.0;//*friction_coeff(i);//contact force z
    qp_H(3+6*i,3+6*i) = 100.0* Fz_ratio(i);//*CoP_boundary(i*2);//contact moment x
    qp_H(4+6*i,4+6*i) = 100.0* Fz_ratio(i);//*CoP_boundary(1+i*2);//contact moment y
    qp_H(5+6*i,5+6*i) = 100.0* Fz_ratio(i);//*friction_coeff(i);//contact moment z
  }

  MatrixXd qp_A(6,Fc_dof);
  qp_A = Wmat;
  VectorXd qp_g(Fc_dof);
  qp_g.setZero();  
  qp_g = -1.0*qp_H*qp_b;

  VectorXd qp_lbA(6);
  qp_lbA = Fc_res;
  VectorXd qp_ubA(6);
  qp_ubA = Fc_res;


  VectorXd qp_lb(Fc_dof);
  VectorXd qp_ub(Fc_dof);

  for(int i=0; i<plane_contact_num; i++)
  {
    qp_lb(0+6*i) = friction_coeff(i)*Fz_local_decided(i);
    qp_lb(1+6*i) = friction_coeff(i)*Fz_local_decided(i);
    qp_lb(2+6*i) = Fz_local_decided(i)*1.5; //margin 1.5
    qp_lb(3+6*i) = CoP_boundary(i*2)*Fz_local_decided(i);
    qp_lb(4+6*i) = CoP_boundary(1+i*2)*Fz_local_decided(i);
    qp_lb(5+6*i) = friction_coeff(i)*Fz_local_decided(i);

    qp_ub(0+6*i) = -friction_coeff(i)*Fz_local_decided(i);
    qp_ub(1+6*i) = -friction_coeff(i)*Fz_local_decided(i);
    qp_ub(2+6*i) = Fz_local_decided(i)*0.9;//0.0;
    qp_ub(3+6*i) = -CoP_boundary(i*2)*Fz_local_decided(i);
    qp_ub(4+6*i) = -CoP_boundary(1+i*2)*Fz_local_decided(i);
    qp_ub(5+6*i) = -friction_coeff(i)*Fz_local_decided(i);

  }

  QPprog.EnableEqualityCondition(0.0001);
  QPprog.UpdateMinProblem(qp_H,qp_g);
  QPprog.UpdateSubjectToAx(qp_A,qp_lbA,qp_ubA);
  QPprog.UpdateSubjectToX(qp_lb,qp_ub);
}


void ControlBase::WrenchRedistributionTwoFootContact(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Vector3D P1, Vector3D P2, VectorXd &F12, VectorXd& ResultantForce, VectorXd& ForceRedistribution, double& eta)
{
  MatrixXd W(6,12);
  W.setZero();

  Matrix3D P1_hat;
  P1_hat = skm(P1);
  Matrix3D P2_hat;
  P2_hat = skm(P2);
  for(int i =0; i<3; i++)
  {
    W(i,i) = 1.0;
    W(i+3,i+3) = 1.0;
    W(i,i+6) = 1.0;
    W(i+3,i+9) = 1.0;

    for(int j=0; j<3; j++)
    {
      W(i+3,j) = P1_hat(i,j);
      W(i+3,j+6) = P2_hat(i,j);
    }
  }
  ResultantForce.resize(6);
  ResultantForce = W*F12;//F1F2;

  double eta_lb = 1.0 - eta_cust;
  double eta_ub = eta_cust;
  double A_threshold = 0.001;
  ////printf("1 lb %f ub %f\n",eta_lb,eta_ub);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////boundary of eta Mx, A*eta + B < 0
  double A = (P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2);
  double B = ResultantForce(3) + P2(2)*ResultantForce(1) -P2(1)*ResultantForce(2);
  double C = ratio_y*footwidth/2.0*abs(ResultantForce(2));
  double a = A*A;
  double b = 2.0*A*B;
  double c = B*B-C*C;

  if(abs(A) < A_threshold)
  {
    if(B*B - C*C < 0) //eta¿Í ¹«°üÇÏ°Ô Ç×»ó žžÁ·, boundary ŒöÁ€ÇÏÁö ŸÊÀœ
    {
    }
    else // B*B-C*C >= 0ÀÌžé no solution, ÃßÈÄ task ŒöÁ€ °úÁ€À» ³ÖŸîŸß ÇÔ
    {
      //printf("0.");
    }
  }
  else
  {
    double sol_eta1 = (-b+sqrt(b*b-4.0*a*c))/2.0/a;
    double sol_eta2 = (-b-sqrt(b*b-4.0*a*c))/2.0/a;
    if(sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
    {
      if(sol_eta1 < eta_ub && sol_eta1 > eta_lb)
      {
        eta_ub = sol_eta1;
      }
      else if(sol_eta1 > eta_ub) // ¹®ÁŠ ŸøÀœ, ±âÁž ub À¯Áö
      {
      }
      else
      {
        //printf("1.");
      }

      if(sol_eta2 > eta_lb && sol_eta2 < eta_ub)
      {
        eta_lb = sol_eta2;
      }
      else if(sol_eta2 < eta_lb) // ¹®ÁŠ ŸøÀœ, ±âÁž lb À¯Áö
      {
      }
      else
      {
        //printf("2.");
      }
    }
    else//sol_eta2 ÀÌ upper boundary
    {
      if(sol_eta2 < eta_ub && sol_eta2 > eta_lb)
      {
        eta_ub = sol_eta2;
      }
      else if(sol_eta2 > eta_ub) // ¹®ÁŠ ŸøÀœ, ±âÁž ub À¯Áö
      {
      }
      else
      {
        //printf("3.");
      }

      if(sol_eta1 > eta_lb && sol_eta1 < eta_ub)
      {
        eta_lb = sol_eta1;
      }
      else if(sol_eta1 < eta_lb) // ¹®ÁŠ ŸøÀœ, ±âÁž lb À¯Áö
      {
      }
      else
      {
        //printf("4.");
      }
    }
  }


  ////printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////boundary of eta My, A*eta + B < 0
  A = - (P1(2)-P2(2))*ResultantForce(0) + (P1(0)-P2(0))*ResultantForce(2);
  B = ResultantForce(4) - P2(2)*ResultantForce(0) + P2(0)*ResultantForce(2);
  C = ratio_x*footlength/2.0*abs(ResultantForce(2));
  a = A*A;
  b = 2.0*A*B;
  c = B*B-C*C;

  if(abs(A) < A_threshold)
  {
    if(B*B - C*C < 0) //eta¿Í ¹«°üÇÏ°Ô Ç×»ó žžÁ·, boundary ŒöÁ€ÇÏÁö ŸÊÀœ
    {
    }
    else // B*B-C*C >= 0ÀÌžé no solution, ÃßÈÄ task ŒöÁ€ °úÁ€À» ³ÖŸîŸß ÇÔ
    {
      //printf("0;");
    }
  }
  else
  {
    double sol_eta1 = (-b+sqrt(b*b-4.0*a*c))/2.0/a;
    double sol_eta2 = (-b-sqrt(b*b-4.0*a*c))/2.0/a;
    if(sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
    {
      if(sol_eta1 < eta_ub && sol_eta1 > eta_lb)
      {
        eta_ub = sol_eta1;
      }
      else if(sol_eta1 > eta_ub) // ¹®ÁŠ ŸøÀœ, ±âÁž ub À¯Áö
      {
      }
      else
      {
        //printf("1;");
      }

      if(sol_eta2 > eta_lb && sol_eta2 < eta_ub)
      {
        eta_lb = sol_eta2;
      }
      else if(sol_eta2 < eta_lb) // ¹®ÁŠ ŸøÀœ, ±âÁž lb À¯Áö
      {
      }
      else
      {
        //printf("2;");
      }
    }
    else//sol_eta2 ÀÌ upper boundary
    {
      if(sol_eta2 < eta_ub && sol_eta2 > eta_lb)
      {
        eta_ub = sol_eta2;
      }
      else if(sol_eta2 > eta_ub) // ¹®ÁŠ ŸøÀœ, ±âÁž ub À¯Áö
      {
      }
      else
      {
        //printf("3;");
      }

      if(sol_eta1 > eta_lb && sol_eta1 < eta_ub)
      {
        eta_lb = sol_eta1;
      }
      else if(sol_eta1 < eta_lb) // ¹®ÁŠ ŸøÀœ, ±âÁž lb À¯Áö
      {
      }
      else
      {
        //printf("4;");
      }
    }
  }

  //printf("5 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
  A = - (P1(0) - P2(0))*ResultantForce(1) + (P1(1) - P2(1))*ResultantForce(0);
  B = ResultantForce(5) + P2(1)*ResultantForce(0) - P2(0)*ResultantForce(1);
  C = staticFrictionCoeff*abs(ResultantForce(2));
  a = A*A;
  b = 2.0*A*B;
  c = B*B-C*C;

  if(abs(A) < A_threshold)
  {
    if(B*B - C*C < 0) //eta¿Í ¹«°üÇÏ°Ô Ç×»ó žžÁ·, boundary ŒöÁ€ÇÏÁö ŸÊÀœ
    {
    }
    else // B*B-C*C >= 0ÀÌžé no solution, ÃßÈÄ task ŒöÁ€ °úÁ€À» ³ÖŸîŸß ÇÔ
    {
      //printf("0,");
    }
  }
  else
  {
    double sol_eta1 = (-b+sqrt(b*b-4.0*a*c))/2.0/a;
    double sol_eta2 = (-b-sqrt(b*b-4.0*a*c))/2.0/a;
    if(sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
    {
      if(sol_eta1 < eta_ub && sol_eta1 > eta_lb)
      {
        eta_ub = sol_eta1;
      }
      else if(sol_eta1 > eta_ub) // ¹®ÁŠ ŸøÀœ, ±âÁž ub À¯Áö
      {
      }
      else
      {
        //printf("1,");
      }

      if(sol_eta2 > eta_lb && sol_eta2 < eta_ub)
      {
        eta_lb = sol_eta2;
      }
      else if(sol_eta2 < eta_lb) // ¹®ÁŠ ŸøÀœ, ±âÁž lb À¯Áö
      {
      }
      else
      {
        //printf("2,");
      }
    }
    else//sol_eta2 ÀÌ upper boundary
    {
      if(sol_eta2 < eta_ub && sol_eta2 > eta_lb)
      {
        eta_ub = sol_eta2;
      }
      else if(sol_eta2 > eta_ub) // ¹®ÁŠ ŸøÀœ, ±âÁž ub À¯Áö
      {
      }
      else
      {
        //printf("3,");
      }

      if(sol_eta1 > eta_lb && sol_eta1 < eta_ub)
      {
        eta_lb = sol_eta1;
      }
      else if(sol_eta1 < eta_lb) // ¹®ÁŠ ŸøÀœ, ±âÁž lb À¯Áö
      {
      }
      else
      {
        //printf("4,");
      }
    }
  }

  double eta_s = (-ResultantForce(3) - P2(2)*ResultantForce(1) + P2(1)*ResultantForce(2))/((P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2));

  if(eta_s > eta_ub)
  {
    eta = eta_ub;
  }
  else if(eta_s < eta_lb)
  {
    eta = eta_lb;
  }
  else
  {
    eta = eta_s;
  }

  if(eta_ub < eta_lb) //ÀÓœÃ...roundoff error·Î Á€È®ÇÑ ÇØ°¡ ŸÈ³ª¿Ã¶§
  {
    //printf("-");
  }
  else if(sqrt(eta_ub*eta_ub + eta_lb*eta_lb) > 1.0) //³Ê¹« Å« °æ°è°ªÀÌ Œ¯¿© ÀÖÀ» ¶§
  {
    //printf("_");
  }

  ForceRedistribution(0) = eta*ResultantForce(0);
  ForceRedistribution(1) = eta*ResultantForce(1);
  ForceRedistribution(2) = eta*ResultantForce(2);
  ForceRedistribution(3) = ((P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2))*eta*eta + (ResultantForce(3) + P2(2)*ResultantForce(1)-P2(1)*ResultantForce(2))*eta;
  ForceRedistribution(4) = (- (P1(2)-P2(2))*ResultantForce(0) + (P1(0)-P2(0))*ResultantForce(2))*eta*eta + (ResultantForce(4) - P2(2)*ResultantForce(0) + P2(0)*ResultantForce(2))*eta;
  ForceRedistribution(5) = (- (P1(0)-P2(0))*ResultantForce(1) + (P1(1)-P2(1))*ResultantForce(0))*eta*eta + (ResultantForce(5) + P2(1)*ResultantForce(0) - P2(0)*ResultantForce(1))*eta;
  ForceRedistribution(6) = (1.0-eta)*ResultantForce(0);
  ForceRedistribution(7) = (1.0-eta)*ResultantForce(1);
  ForceRedistribution(8) = (1.0-eta)*ResultantForce(2);
  ForceRedistribution(9) = (1.0-eta)*(((P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2))*eta + (ResultantForce(3) + P2(2)*ResultantForce(1)-P2(1)*ResultantForce(2)));
  ForceRedistribution(10) = (1.0-eta)*((- (P1(2)-P2(2))*ResultantForce(0) + (P1(0)-P2(0))*ResultantForce(2))*eta + (ResultantForce(4) - P2(2)*ResultantForce(0) + P2(0)*ResultantForce(2)));
  ForceRedistribution(11) = (1.0-eta)*((- (P1(0)-P2(0))*ResultantForce(1) + (P1(1)-P2(1))*ResultantForce(0))*eta + (ResultantForce(5) + P2(1)*ResultantForce(0) - P2(0)*ResultantForce(1)));
}

/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------------------ */

void ControlBase::WrenchRedistributionTwoFootContact_CustomEta(double Eta_Standard, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Vector3D P1, Vector3D P2, VectorXd &F12, VectorXd& ResultantForce,  VectorXd& ForceRedistribution)
{
  MatrixXd W(6,12);
  W.setZero();

  Matrix3D P1_hat;
  P1_hat = skm(P1);
  Matrix3D P2_hat;
  P2_hat = skm(P2);
  for(int i =0; i<3; i++)
  {
    W(i,i) = 1.0;
    W(i+3,i+3) = 1.0;
    W(i,i+6) = 1.0;
    W(i+3,i+9) = 1.0;

    for(int j=0; j<3; j++)
    {
      W(i+3,j) = P1_hat(i,j);
      W(i+3,j+6) = P2_hat(i,j);
    }
  }
  ResultantForce.resize(6);
  ResultantForce = W*F12;//F1F2;

  double eta_lb = 0.0;
  double eta_ub = 1.0;
  //printf("1 lb %f ub %f\n",eta_lb,eta_ub);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  //boundary of eta Mx, A*eta + B < 0
  double A = (P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2);
  double B = ResultantForce(3) + P2(2)*ResultantForce(1) -P2(1)*ResultantForce(2);
  double C = ratio_y*footwidth/2.0*abs(ResultantForce(2));
  double a = A*A;
  double b = 2.0*A*B;
  double c = B*B-C*C;
  double sol_eta1 = (-b+sqrt(b*b-4.0*a*c))/2.0/a;
  double sol_eta2 = (-b-sqrt(b*b-4.0*a*c))/2.0/a;
  if(sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
  {
    if(sol_eta1 < eta_ub)
    {
      eta_ub = sol_eta1;
    }

    if(sol_eta2 > eta_lb)
    {
      eta_lb = sol_eta2;
    }
  }
  else//sol_eta2 ÀÌ upper boundary
  {
    if(sol_eta2 < eta_ub)
    {
      eta_ub = sol_eta2;
    }

    if(sol_eta1 > eta_lb)
    {
      eta_lb = sol_eta1;
    }
  }

  //printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  //boundary of eta My, A*eta + B < 0
  A = - (P1(2)-P2(2))*ResultantForce(0) + (P1(0)-P2(0))*ResultantForce(2);
  B = ResultantForce(4) - P2(2)*ResultantForce(0) + P2(0)*ResultantForce(2);
  C = ratio_x*footlength/2.0*abs(ResultantForce(2));
  a = A*A;
  b = 2.0*A*B;
  c = B*B-C*C;
  sol_eta1 = (-b+sqrt(b*b-4.0*a*c))/2.0/a;
  sol_eta2 = (-b-sqrt(b*b-4.0*a*c))/2.0/a;
  if(sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
  {
    if(sol_eta1 < eta_ub)
    {
      eta_ub = sol_eta1;
    }

    if(sol_eta2 > eta_lb)
    {
      eta_lb = sol_eta2;
    }
  }
  else//sol_eta2 ÀÌ upper boundary
  {
    if(sol_eta2 < eta_ub)
    {
      eta_ub = sol_eta2;
    }

    if(sol_eta1 > eta_lb)
    {
      eta_lb = sol_eta1;
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  //boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
  A = - (P1(0) - P2(0))*ResultantForce(1) + (P1(1) - P2(1))*ResultantForce(0);
  B = ResultantForce(5) + P2(1)*ResultantForce(0) - P2(0)*ResultantForce(1);
  C = staticFrictionCoeff*abs(ResultantForce(2));
  a = A*A;
  b = 2.0*A*B;
  c = B*B-C*C;
  sol_eta1 = (-b+sqrt(b*b-4.0*a*c))/2.0/a;
  sol_eta2 = (-b-sqrt(b*b-4.0*a*c))/2.0/a;
  if(sol_eta1 > sol_eta2) //sol_eta1 ÀÌ upper boundary
  {
    if(sol_eta1 < eta_ub)
    {
      eta_ub = sol_eta1;
    }

    if(sol_eta2 > eta_lb)
    {
      eta_lb = sol_eta2;
    }
  }
  else//sol_eta2 ÀÌ upper boundary
  {
    if(sol_eta2 < eta_ub)
    {
      eta_ub = sol_eta2;
    }

    if(sol_eta1 > eta_lb)
    {
      eta_lb = sol_eta1;
    }
  }

  double eta = Eta_Standard;
  if(Eta_Standard > eta_ub)
  {
    eta = eta_ub;
  }
  else if(Eta_Standard < eta_lb)
  {
    eta = eta_lb;
  }
  ForceRedistribution(0) = eta*ResultantForce(0);
  ForceRedistribution(1) = eta*ResultantForce(1);
  ForceRedistribution(2) = eta*ResultantForce(2);
  ForceRedistribution(3) = ((P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2))*eta*eta + (ResultantForce(3) + P2(2)*ResultantForce(1)-P2(1)*ResultantForce(2))*eta;
  ForceRedistribution(4) = (- (P1(2)-P2(2))*ResultantForce(0) + (P1(0)-P2(0))*ResultantForce(2))*eta*eta + (ResultantForce(4) - P2(2)*ResultantForce(0) + P2(0)*ResultantForce(2))*eta;
  ForceRedistribution(5) = (- (P1(0)-P2(0))*ResultantForce(1) + (P1(1)-P2(1))*ResultantForce(0))*eta*eta + (ResultantForce(5) + P2(1)*ResultantForce(0) - P2(0)*ResultantForce(1))*eta;
  ForceRedistribution(6) = (1.0-eta)*ResultantForce(0);
  ForceRedistribution(7) = (1.0-eta)*ResultantForce(1);
  ForceRedistribution(8) = (1.0-eta)*ResultantForce(2);
  ForceRedistribution(9) = (1.0-eta)*(((P1(2)-P2(2))*ResultantForce(1) - (P1(1)-P2(1))*ResultantForce(2))*eta + (ResultantForce(3) + P2(2)*ResultantForce(1)-P2(1)*ResultantForce(2)));
  ForceRedistribution(10) = (1.0-eta)*((- (P1(2)-P2(2))*ResultantForce(0) + (P1(0)-P2(0))*ResultantForce(2))*eta + (ResultantForce(4) - P2(2)*ResultantForce(0) + P2(0)*ResultantForce(2)));
  ForceRedistribution(11) = (1.0-eta)*((- (P1(0)-P2(0))*ResultantForce(1) + (P1(1)-P2(1))*ResultantForce(0))*eta + (ResultantForce(5) + P2(1)*ResultantForce(0) - P2(0)*ResultantForce(1)));
}




}
