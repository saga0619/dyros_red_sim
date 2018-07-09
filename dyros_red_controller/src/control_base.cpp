
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
    /* ------------------- whole-body dynamics ----------------- */
    /* --------------------------------------------------------- */

    if(_dynamics_update == true)
    {
      CWholeBodyControl WBCMain(31);
      WBCMain.updateGlobalDynamicsParameters(_model._G,_model._b,_model._A); //update dynamic info

      //set Contact State
      int contactdof = 12;      
      if(_ContactState == 0) //double foot contact
      {
        contactdof = 12;        
        _Jc.resize(contactdof,DyrosRedModel::MODEL_DOF+6);
      }
      else if(_ContactState == 1 ||_ContactState == 2) //single foot contact
      {
        contactdof = 6;
        _Jc.resize(contactdof,DyrosRedModel::MODEL_DOF+6);
      }
      const int contactdof_QP = contactdof;
      _Jc_dot.setZero();
      _Jc = setWBCContactJacobian(_ContactState);//Get predefined Contact Jacobian

      WBCMain.updateContactParameters(_Jc,_Jc_dot,_ContactState); //update contact Jacobian
      WBCMain.calculateContactSpaceInvDynamics(); //calculate contact space inverse dynamics
      _Jc_bar_T_S_k_T = WBCMain._Jc_bar_T*WBCMain._S_k_T;
      _pc = WBCMain._pc;
      WBCMain.calculateGravityCoriolisCompensationTorque();//calculate gravity compensation torque and coriolis centrifugal compensation torque

       _torque_gravity = WBCMain._GravityCompensationTorque;

       //set Task State
       int taskdof = 0;//task space dof       
       int nulldof = 0;//null space dof
       if(_TaskState == 0) //double foot contact
       {
          taskdof= 18;
         _J.resize(taskdof,DyrosRedModel::MODEL_DOF+6);
       }
       else if(_TaskState == 1 ||_TaskState == 2) //single foot contact
       {
         taskdof= 25;         
         _J.resize(taskdof,DyrosRedModel::MODEL_DOF+6);
       }
       const int taskdof_QP = taskdof;
       _J = setWBCTaskJacobian(_TaskState);
       _xdot = _J*_qdot_virtual;
       WBCMain.updateTaskParameters(_J,_TaskState);
       WBCMain.calculateContactConstrainedTaskJacobian();
       WBCMain.calculateSVDofWeightingMatrix();

       _J_wbc_T =  WBCMain._J_k_T;
       _lambda_wbc = WBCMain._Lambda;

       /* --------------------------------------------------------- */
       /* --------------------------------------------------------- */

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

      _desired_pelvis_rotation = _init_pelvis_rotation;
      _desired_right_arm_rotation = _init_right_arm_rotation;
      _desired_left_arm_rotation = _init_left_arm_rotation;

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
        _desired_pelvis_position(0) = cubic(control_time_,0.8,1.0,_init_pelvis_position(0),_init_pelvis_position(0)+0.03,0.0,0.0);
        _desired_pelvis_position(1) = cubic(control_time_,0.8,1.0,_init_pelvis_position(1),_init_pelvis_position(1)+0.0,0.0,0.0);
        _desired_pelvis_position(2) = cubic(control_time_,0.8,1.0,_init_pelvis_position(2),_init_pelvis_position(2),0.0,0.0);

        _desired_pelvis_velocity(0) = cubicDot(control_time_,0.8,1.0,_init_pelvis_position(0),_init_pelvis_position(0)+0.03,0.0,0.0,Hz_);
        _desired_pelvis_velocity(1) = cubicDot(control_time_,0.8,1.0,_init_pelvis_position(1),_init_pelvis_position(1)+0.0,0.0,0.0,Hz_);
        _desired_pelvis_velocity(2) = cubicDot(control_time_,0.8,1.0,_init_pelvis_position(2),_init_pelvis_position(2),0.0,0.0,Hz_);
      }
      else
      {
        _desired_pelvis_position(0) = _init_pelvis_position(0) + 0.03;
        _desired_pelvis_position(1) = _init_pelvis_position(1) + 0.0;
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

      double kp = 10000.0;
      double kd = 100.0;
      for(int i=0; i<3; i++)
      {
        _fstar(i) = kp*_pelvis_position_error(i) + kd*(_desired_pelvis_velocity(i) - _xdot(i));
        _fstar(i+3) = kp*_pelvis_orientation_error(i) - kd*_xdot(i+3);
        _fstar(i+6) = kp*_right_arm_position_error(i) + kd*(_desired_right_arm_velocity(i) - _xdot(i+6));
        _fstar(i+9) = kp*_right_arm_orientation_error(i) - kd*_xdot(i+9);
        _fstar(i+12) = kp*_left_arm_position_error(i) + kd*(_desired_left_arm_velocity(i) - _xdot(i+12));
        _fstar(i+15) = kp*_left_arm_orientation_error(i) - kd*_xdot(i+15);
      }

      /* --------------------------------------------------------- */
      /* -------------  Task Modification - QP base -------------- */
      /* --------------------------------------------------------- */

      ros::Time time_temp = ros::Time::now();

      //setup data of first QP
      real_t H[taskdof_QP*taskdof_QP]; // H in min eq 1/2x'Hx + x'g
      real_t A[6*taskdof_QP]; // A in s.t. eq lbA<= Ax <=ubA
      real_t g[taskdof_QP]; //g in min eq 1/2x'Hx + x'g
      real_t lbA[6]; // lbA in s.t. eq lbA<= Ax <=ubA
      real_t ubA[6]; // ubA in s.t. eq lbA<= Ax <=ubA
      real_t lb[taskdof_QP]; //lb in s.t. eq lb <= x <= ub
      real_t ub[taskdof_QP]; //ub in s.t. eq lb <= x <= ub

      _torque_task =_J_wbc_T*_lambda_wbc*_fstar;      
      ros::Time time_s = ros::Time::now();
      WBCMain.calculateContactWrenchLocalContactFrame(_torque_gravity + _torque_task); //caculate contact wrench
      double s_time = ros::Time::now().toSec() - time_s.toSec();
      cout <<"calc time" << s_time << endl;

      if(_ContactState == 0) //double foot contact
      {        
        Vector3d P1;
        P1 = _model.link_[_model.Right_Leg+5].Contact_position - (_model.link_[_model.Right_Leg+5].Contact_position +_model.link_[_model.Left_Leg+5].Contact_position)/2.0;
        Vector3d P2;
        P2 = _model.link_[_model.Left_Leg+5].Contact_position - (_model.link_[_model.Right_Leg+5].Contact_position +_model.link_[_model.Left_Leg+5].Contact_position)/2.0;
        Matrix3d RotFoot;
        RotFoot.setZero();
        double thetaTwoFeet;
        if(P1(0)>P2(0)) //RotateTwoFeetžžÅ­ ÈžÀü, À§Ä¡žž ÈžÀü
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


        MatrixXd qp_A(6,taskdof);
        qp_A = Res*WBCMain._Jc_bar_T*WBCMain._S_k_T*WBCMain._J_k_T*WBCMain._Lambda;
        VectorXd qp_b(6);
        qp_b = Res*(WBCMain._Jc_bar_T*WBCMain._S_k_T*_torque_gravity - WBCMain._pc);

        MatrixXd qp_H(taskdof,taskdof);
        qp_H.setZero();
        for(int i=0; i<3; i++) //trunk pos
        {
          qp_H(i,i) = 1000.0;
        }
        for(int i=3; i<taskdof; i++) //else
        {
          qp_H(i,i) = 500.0;
        }
        VectorXd qp_g(taskdof);
        qp_g = -1.0*qp_H*_fstar;

        VectorXd qp_ubA(6);
        qp_ubA(0) = -static_fric_coeff*Fz-qp_b(0);
        qp_ubA(1) = -static_fric_coeff*Fz-qp_b(1);
        qp_ubA(2) = Fz-qp_b(2);
        qp_ubA(3) = -width_boundary_cop*Fz-qp_b(3);
        qp_ubA(4) = -length_boundary_cop*Fz-qp_b(4);
        qp_ubA(5) = -static_fric_coeff*Fz-qp_b(5);
        VectorXd qp_lbA(6);
        qp_lbA(0) = static_fric_coeff*Fz-qp_b(0);
        qp_lbA(1) = static_fric_coeff*Fz-qp_b(1);
        qp_lbA(2) = Fz-qp_b(2);
        qp_lbA(3) = width_boundary_cop*Fz-qp_b(3);
        qp_lbA(4) = length_boundary_cop*Fz-qp_b(4);
        qp_lbA(5) = static_fric_coeff*Fz-qp_b(5);

        VectorXd qp_ub(taskdof);
        VectorXd qp_lb(taskdof);
        for(int i=0; i<taskdof; i++)
        {
          if(_fstar(i) > 0.0) //if positive
          {
            qp_ub(i) = _fstar(i);
            qp_lb(i) = 0.0;
          }
          else //if negative
          {
            qp_ub(i) = 0.0;
            qp_lb(i) = _fstar(i);
          }
        }
       cout <<"qp_lb" <<endl << qp_lb.transpose() << endl <<endl;
       cout <<"qp_ub" <<endl << qp_ub.transpose() << endl <<endl;

        for(int i=0; i<taskdof*taskdof; i++)
        {
          H[i] = 0.0; //initialize
        }

        for(int i=0; i<taskdof; i++)
        {
          A[0+i] = qp_A(0,i);
          A[taskdof+i] = qp_A(1,i);
          A[taskdof*2+i] = qp_A(2,i);
          A[taskdof*3+i] = qp_A(3,i);
          A[taskdof*4+i] = qp_A(4,i);
          A[taskdof*5+i] = qp_A(5,i);

          g[i] = qp_g(i);

          H[i*taskdof+i] = qp_H(i,i);

          ub[i] = qp_ub(i);
          lb[i] = qp_lb(i);
        }

        for(int i=0; i<6; i++)
        {
          lbA[i] = qp_lbA(i);
          ubA[i] = qp_ubA(i);
        }        

      }
      else if(_ContactState == 1 ||_ContactState == 2) //single foot contact
      {
      }
      else //else
      {
      }

      double QP_setup_time = ros::Time::now().toSec() - time_temp.toSec();
      cout <<"setup time" << QP_setup_time << endl;


      time_temp = ros::Time::now();

      SQProblem QPModifyTask(taskdof_QP,6); //number of variables (number of tasks), number of constraints (dof of resultant contact wrench)
      int_t nWSR = 1000;
      Options options;
      options.printLevel = PL_NONE;
      QPModifyTask.setOptions(options);

      //first solve QP
      returnValue m_status;
      //m_status = QPModifyTask.init(H,g,A,0,0,lbA,ubA,nWSR); //Only once after changing contact & task state
      m_status = QPModifyTask.init(H,g,A,lb,ub,lbA,ubA,nWSR); //Only once after changing contact & task state

      real_t xOpt[taskdof_QP];
      QPModifyTask.getPrimalSolution(xOpt);

      double QP_init_time = ros::Time::now().toSec() - time_temp.toSec();
      cout <<"init time" << QP_init_time <<endl;

      VectorXd fstar_Opt(taskdof);
      for(int i=0; i<taskdof; i++)
      {
        fstar_Opt(i) = xOpt[i];
      }
      cout <<"fstar" <<endl << _fstar.transpose() << endl <<endl;
      cout << "qp fstar" <<endl << fstar_Opt.transpose() << endl <<endl;
      cout << "status " << m_status << endl <<endl;


      //cout<<"Fc original"<<endl << WBCMain._Fc_LocalContactFrame.transpose() << endl <<endl;
      //VectorXd torque_test(DyrosRedModel::MODEL_DOF+6);
      //torque_test =_J_wbc_T*_lambda_wbc*fstar_Opt;
      //WBCMain.calculateContactWrenchLocalContactFrame(_torque_gravity + torque_test); //caculate contact wrench
      //cout<<"Fc qp"<<endl << WBCMain._Fc_LocalContactFrame.transpose() << endl <<endl;

      //QPModifyTask.reset();

      time_temp = ros::Time::now();
      //QPModifyTask.hotstart(H,g,A,0,0,lbA,ubA,nWSR);
      QPModifyTask.hotstart(H,g,A,lb,ub,lbA,ubA,nWSR);
      QPModifyTask.getPrimalSolution(xOpt);
      double QP_hotstart_time = ros::Time::now().toSec() - time_temp.toSec();
      cout << "hotstart time"<< QP_hotstart_time << endl<<endl;
      //for(int i=0; i<taskdof; i++)
      //{
      //  qp_xOpt(i) = xOpt[i];
      //}
      //cout << "hotstart" <<endl << qp_xOpt.transpose() << endl <<endl;

      //getchar();

      /* Setup data of first QP. */
/*
        real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
        real_t A[1*2] = { 1.0, 1.0 };
        real_t g[2] = { 1.5, 1.0 };
        real_t lb[2] = { 0.5, -2.0 };
        real_t ub[2] = { 5.0, 2.0 };
        real_t lbA[1] = { -1.0 };
        real_t ubA[1] = { 2.0 };
*/
        /* Setup data of second QP. */
      /*
        real_t g_new[2] = { 1.0, 1.5 };
        real_t lb_new[2] = { 0.0, -1.0 };
        real_t ub_new[2] = { 5.0, -0.5 };
        real_t lbA_new[1] = { -2.0 };int_t
        real_t ubA_new[1] = { 1.0 };
*/

        /* Setting up QProblem object. */
      /*
      QProblem example( 2,1 );
      Options options;
      options.printLevel = PL_NONE;

      typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, RowMajor> MatrixXdRow;

      MatrixXd H2(2,2);
      H2.setZero();
      H2(0, 0) = 1.0;
      H2(1, 1) = 0.5;
      MatrixXdRow H2_row = H2;int_t
      MatrixXdRow A2(1, 2);
      A2.setOnes();


      example.setOptions( options );
*/

      /* Solve first QP. */
      /*
      int nWSR = 10;
      example.init( H2_row.data() , g, A2.data(), lb, ub,lbA,ubA, nWSR );
*/
      /* Get and print solution of first QP. */
      /*
      real_t xOpt[2];
      VectorXd x_sol(2);
      real_t yOpt[2+1];
      example.getPrimalSolution( x_sol.data() );
      // example.getDualSolution( yOpt );
      cout << x_sol.transpose() << endl;
*/
      /* --------------------------------------------------------- */
      /* --------------------------------------------------------- */


      //_torque_task =_J_wbc_T*_lambda_wbc*_fstar;
      _torque_task =_J_wbc_T*_lambda_wbc*fstar_Opt;

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
        VectorXd ResultantWrench(6);
        ResultantWrench.setZero();
        VectorXd RedistributionWrench(12);
        RedistributionWrench.setZero();
        double eta_Fc = 0.0;

        WrenchRedistributionTwoFootContact(0.95,foot_length,foot_width,1.0,0.8,0.8,P1,P2,WBCMain._Fc_LocalContactFrame, ResultantWrench, RedistributionWrench, eta_Fc);

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

      }

       /* --------------------------------------------------------- */
       /* --------------------------------------------------------- */
    }

    _torque_d = _torque_gravity + _torque_task + _torque_contact;// - _qdot*1.0;
  }

  /* --------------------------------------------------------------------- */


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
  _Jc.resize(12,DyrosRedModel::MODEL_DOF+6);
  _Jc.setZero();
  _Jc_dot.resize(12,DyrosRedModel::MODEL_DOF+6);
  _Jc_dot.setZero();
  _J.resize(6,DyrosRedModel::MODEL_DOF+6);
  _J.setZero();
  _torque_gravity.setZero();  

  _Jc_bar_T_S_k_T.resize(12,DyrosRedModel::MODEL_DOF);
  _Jc_bar_T_S_k_T.setZero();
  _pc.resize(12);
  _pc.setZero();
  _J_wbc_T.resize(6,DyrosRedModel::MODEL_DOF);
  _J_wbc_T.setZero();
  _lambda_wbc.resize(6,6);
  _lambda_wbc.setZero();
  _fstar.resize(6);
  _fstar.setZero();
  _torque_task.setZero();
  _Fc_LocalContactFrame.resize(12);
  _Fc_LocalContactFrame.setZero();

  _torque_contact.setZero();
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
  _Jc_dot.resize(contact_dof,DyrosRedModel::MODEL_DOF+6);
  _pc.resize(contact_dof);
  _pc.setZero();
  _Fc_LocalContactFrame.resize(contact_dof);
  _Fc_LocalContactFrame.setZero();
  _Jc_bar_T_S_k_T.resize(contact_dof, DyrosRedModel::MODEL_DOF);
  _Jc_bar_T_S_k_T.setZero();


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

  _fstar.resize(task_dof);
  _J_wbc_T.resize(DyrosRedModel::MODEL_DOF,task_dof);
  _lambda_wbc.resize(task_dof,task_dof);
  _xdot.resize(task_dof);

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
