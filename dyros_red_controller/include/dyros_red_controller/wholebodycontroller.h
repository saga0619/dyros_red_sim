#ifndef WHOLEBODYCONTROLLER_H
#define WHOLEBODYCONTROLLER_H

//#define EIGEN_3_3 //if the eigen version is latest version after 3.3, use this definition

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <math_type_define.h>

using namespace Eigen;
using namespace std;

namespace Eigen
{
  typedef Matrix<rScalar, 3, 1>	Vector3D;
  typedef Matrix<rScalar, 3, 3>	Matrix3D;
  typedef Matrix<rScalar, 6, 1>	Vector6D;
  typedef Matrix<rScalar, 6, 6>	Matrix6D;
  typedef Matrix<rScalar, 4, 3>	HTransform;
}

class CWholeBodyControl
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  CWholeBodyControl(int DOF);
  virtual ~CWholeBodyControl();

public:

  void updateLocaltoGlobalDynamicsParameters(const HTransform &Tbase, const VectorXd &localG, const MatrixXd &localC, const MatrixXd &localA);
  void updateGlobalDynamicsParameters(const VectorXd &globalG, const VectorXd &globalC, const MatrixXd &globalA);
  void updateLocaltoGlobalKinematicsParameters(const Vector6D &xdot_base, const VectorXd &qdot); //require only when calculating Coriolis/centrifugal force compensation torque
  void updateGlobalKinematicsParameters(const Vector6D &xdot_base, const VectorXd &qdot); //require only when calculating Coriolis/centrifugal force compensation torque
  void updateContactParameters(const MatrixXd &Jc,const MatrixXd &Jc_dot,const int &ContactState);
  void updateTaskParameters(const MatrixXd &J, const int &TaskState);
  //void calculateFloatingBaseGlobalDynamics();
  void calculateContactSpaceInvDynamics();
  void calculateGravityCoriolisCompensationTorque(); //Gravity & Coriolis/centrifugal compensation torque
  void calculateContactConstrainedTaskJacobian();
  void calculateSVDofWeightingMatrix();
  void calculateNullSpaceJacobian();
  void calculateContactWrenchLocalContactFrame(const VectorXd &Torque);
  void calculateContactWrenchRedistributionTorque(const MatrixXd &Scw, const VectorXd &_Fc_compensate_d, VectorXd &Torque);  

  HTransform _globalT; //HTransform Matrix 4x3, upper 3x3 is rotation, lower 1x3 is position //(matrix 4x4), .linear() - rotation matrix 3x3, .translation() - position vector 3x1
  VectorXd _globalG;
  VectorXd _globalb;
  MatrixXd _globalA;
  MatrixXd _globalAinv;
  MatrixXd _S_k;
  MatrixXd _S_k_T;
  MatrixXd _Jc_bar_T;
  MatrixXd _Jc_bar_T_S_k_T;
  VectorXd _pc;
  MatrixXd _Pc;
  VectorXd _GravityCompensationTorque;
  VectorXd _CoriolisCompensationTorque;
  MatrixXd _Lambda;
  MatrixXd _LambdaJointSpace;
  MatrixXd	_Jjoint_bar_T;
  MatrixXd _W;
  MatrixXd _J_bar_T;
  MatrixXd _J_k_T;
  MatrixXd _J_k_T_Lambda;
  MatrixXd _V2forContactWrenchRedistribution;
  MatrixXd _N_k_T;
  MatrixXd _Jc;
  MatrixXd _Jc_dot;
  MatrixXd _lambda_c;
  VectorXd _Fc_LocalContactFrame;
  int _V2row;
  int _V2col;
  int _ContactState;

private:

  int _JOINTNUM;
  bool _bDynamicParameter;
  bool _bContactParameter;
  bool _bTaskParameter;
  bool _bContactWrenchCalc;
  VectorXd _localG;
  VectorXd _localb;
  MatrixXd _localC;
  MatrixXd _localA;
  int _Jcrow;
  int _Jccol;
  MatrixXd _J;
  int _Jrow;
  int _Jcol;
  int _TaskState;
  VectorXd _qdot;

  void initialize();  
  MatrixXd OneSidedInverse(const MatrixXd &A);
  void calculateFloatingBaseGlobalGravityVector();
  void calculateFloatingBaseGlobalCoriolisVector();
  void calculateFloatingBaseGlobalInertiaMatrix();
  MatrixXd GenInvSemiPosDef(const int &ContactState, const MatrixXd &Q, const MatrixXd &W);
};
#endif // WHOLEBODYCONTROLLER_H

