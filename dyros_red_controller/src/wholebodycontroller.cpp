#include "dyros_red_controller/wholebodycontroller.h"


CWholeBodyControl::CWholeBodyControl(int DOF)
{
  _JOINTNUM = DOF;
  initialize();
}
CWholeBodyControl::~CWholeBodyControl()
{
}

void CWholeBodyControl::initialize()
{
  _localG.resize(_JOINTNUM+6);
  _localG.setZero();
  _globalb.resize(_JOINTNUM+6);
  _globalb.setZero();
  _globalG.resize(_JOINTNUM+6);
  _globalG.setZero();
  _localC.resize(_JOINTNUM+6,_JOINTNUM+6);
  _localC.setZero();
  _localA.resize(_JOINTNUM+6,_JOINTNUM+6);
  _localA.setZero();
  _localb.resize(_JOINTNUM+6);
  _localb.setZero();
  _globalA.resize(_JOINTNUM+6,_JOINTNUM+6);
  _globalA.setZero();
  _globalAinv.resize(_JOINTNUM+6,_JOINTNUM+6);
  _globalAinv.setZero();
  _Jc_bar_T.resize(_JOINTNUM,_JOINTNUM+6);
  _Jc_bar_T.setZero();
  _pc.resize(12);
  _pc.setZero();
  _Pc.resize(_JOINTNUM+6,_JOINTNUM+6);
  _Pc.setZero();
  _GravityCompensationTorque.resize(_JOINTNUM);
  _GravityCompensationTorque.setZero();
  _CoriolisCompensationTorque.resize(_JOINTNUM);
  _CoriolisCompensationTorque.setZero();
  _LambdaJointSpace.resize(_JOINTNUM,_JOINTNUM);
  _LambdaJointSpace.setZero();
  _Jjoint_bar_T.resize(_JOINTNUM, _JOINTNUM+6);
  _Jjoint_bar_T.setZero();
  _W.resize(_JOINTNUM,_JOINTNUM);
  _W.setZero();
  _J_bar_T.resize(6,_JOINTNUM+6);
  _J_bar_T.setZero();
  _J_k_T.resize(6,_JOINTNUM);
  _J_k_T.setZero();
  _N_k_T.resize(_JOINTNUM,_JOINTNUM);
  _N_k_T.setZero();
  _Fc_LocalContactFrame.resize(12);
  _Fc_LocalContactFrame.setZero();

  _qdot.resize(_JOINTNUM+6);
  _qdot.setZero();

  _bDynamicParameter = false;
  _bContactParameter = false;
  _bTaskParameter = false;

  _S_k.resize(_JOINTNUM, _JOINTNUM+6);//Selection Matrix to eliminate virtual joint torque from torque solution
  _S_k.setZero();
  for(int i=0; i<_JOINTNUM; i++)
  {
    _S_k(i,i+6) = 1.0;
  }
  _S_k_T.resize(_JOINTNUM+6,_JOINTNUM);
  _S_k_T = _S_k.transpose();

  _ContactState = 0;
  _TaskState = 0;
  _Jcrow=12;
  _Jccol=_JOINTNUM+6;
  _Jc.resize(_Jcrow,_Jccol);
  _Jc.setZero();
  _Jrow=6;
  _Jcol=_JOINTNUM+6;
  _J.resize(_Jrow,_Jcol);
  _J.setZero();
  _Jc_dot.resize(_Jcrow,_Jccol);
  _Jc_dot.setZero();
  _lambda_c.resize(_Jcrow,_Jcrow);
  _lambda_c.setZero();
}


MatrixXd CWholeBodyControl::OneSidedInverse(const MatrixXd &A)
{
  MatrixXd Ainv(A.cols(),A.rows());

  if(A.cols() > A.rows()) //left inverse
  {
    MatrixXd AAT(A.rows(),A.rows());
    AAT = A*A.transpose();
    MatrixXd AATinv(A.rows(),A.rows());
    AATinv = AAT.inverse();

    Ainv = A.transpose()*AATinv;
  }
  else //right inverse
  {
    MatrixXd ATA(A.cols(),A.cols());
    ATA = A.transpose()*A;
    MatrixXd ATAinv(A.cols(),A.cols());
    ATAinv = ATA.inverse();
    Ainv = ATAinv*A.transpose();
  }

  return Ainv;
}


MatrixXd pinv_SVD(const MatrixXd &A, double epsilon = numeric_limits<double>::epsilon())
{
  JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
  double tolerance = epsilon * max(A.cols(), A.rows()) * svd.singularValues().array().abs()(0);
  return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(),0).matrix().asDiagonal() * svd.matrixU().adjoint();
}


 MatrixXd pinv_QR(const MatrixXd &A) //faster than pinv_SVD,
{
    FullPivLU<MatrixXd>lu_decomp(A);

    int rank = lu_decomp.rank();
    if (rank == 0) {
      cout << "WARN::Input Matrix seems to be zero matrix" << endl;
      return A;
    }
    else {
     // CompleteOrthogonalDecomposition<MatrixXd> cod(A); //need recent version of Eigen
     // return cod.pseudoInverse();

      /*
      if (A.rows() > A.cols()) {
        ColPivHouseholderQR<MatrixXd> qr(A);
        qr.compute(A);
        MatrixXd R = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Upper>();
        MatrixXd Rpsinv2(A.cols(), A.rows());

        Rpsinv2.setZero();
        Rpsinv2.topLeftCorner(rank, rank) = R.inverse();

        return qr.colsPermutation() * Rpsinv2 * qr.householderQ().inverse();
      }
      else if (A.cols() > A.rows()) {
        ColPivHouseholderQR<MatrixXd> qr(A.transpose());
        qr.compute(A.transpose());
        MatrixXd R = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Upper>();
        MatrixXd Rpsinv2(A.rows(), A.cols());

        Rpsinv2.setZero();
        Rpsinv2.topLeftCorner(rank, rank) = R.inverse();
        return (qr.colsPermutation() * Rpsinv2 * qr.householderQ().inverse()).transpose();
      }
      else if (A.cols() == A.rows()) {
        CompleteOrthogonalDecomposition<MatrixXd> cod(A);
        return cod.pseudoInverse();
      }
      */
  }
}




MatrixXd CWholeBodyControl::GenInvSemiPosDef(const int &ContactState, const MatrixXd &Q, const MatrixXd &W)
{
  int Q_row = Q.rows();
  int Q_col = Q.cols();

  JacobiSVD<MatrixXd> svd(Q, ComputeFullU | ComputeFullV);

  MatrixXd Singluar(Q_row,Q_col);//vector·Î ÁŠ°øµÇŽÂ SžŠ matrix ÇüÅÂ·Î º¯È¯
  MatrixXd U(Q_row,Q_row);
  U = svd.matrixU();
  MatrixXd V_T(Q_col,Q_col);
  V_T = svd.matrixV().transpose();
  //V_T = svd.matrixV();

  int S_val_n=0;
  double threshold = 0.0001;
  Singluar.setZero();
  int min_col_row = Q_col;
  if(min_col_row > Q_row)
  {
    min_col_row = Q_row;
  }

  for(int i=0; i<min_col_row; i++)
  {
    if(svd.singularValues()(i) > threshold)
    {
      Singluar(i,i) = svd.singularValues()(i);
      S_val_n = S_val_n+1;
    }
    else
    {
      Singluar(i,i) = 0.0;
    }
  }

  MatrixXd invQ(Q_col,Q_row);

  //U, V_T, S žŠ SÀÇ À¯È¿ŒººÐ(?)ÀÇ Œö¿¡ žÂÃßŸî ºÐÇØ ÇÔ
  MatrixXd U1(Q_row,S_val_n);
  for(int i=0; i<Q_row; i++)
  {
    for(int j=0; j<S_val_n; j++)
    {
      U1(i,j) = U(i,j);
    }
  }
  MatrixXd U1_T(S_val_n,Q_row);
  U1_T = U1.transpose();
  MatrixXd V1_T(S_val_n,Q_col);
  for(int i=0; i<S_val_n; i++)
  {
    for(int j=0; j<Q_col; j++)
    {
      V1_T(i,j) = V_T(i,j);
    }
  }
  MatrixXd V2_T((Q_col-S_val_n),Q_col);
  for(int i=0; i<(Q_col-S_val_n); i++)
  {
    for(int j=0; j<Q_col; j++)
    {
      V2_T(i,j) = V_T(i+S_val_n,j);
    }
  }
  MatrixXd V1(Q_col,S_val_n);
  V1= V1_T.transpose();
  MatrixXd V2(Q_col,(Q_col-S_val_n));
  V2= V2_T.transpose();
  MatrixXd S1(S_val_n,S_val_n);
  S1.setZero();
  for(int i=0; i<S_val_n; i++)
  {
    S1(i,i) = Singluar(i,i);
  } 
  //QÀÇ ÇàÀÌ ¿­ºžŽÙ Žõ Å©Áö ŸÊŽÙŽÂ °¡Á€ÇÏ¿¡ °è»ê
  if(S_val_n == Q_col) // QÀÇ Çà°ú S vector ŒººÐÀÇ ŒýÀÚ°¡ °°À» ¶§(¿©À¯ ÀÚÀ¯µµ°¡ ŸøÀ» ¶§, V2_T°¡ ÁžÀçÇÏÁö ŸÊÀœ)¿Í Q°¡ thin matrixÀÏ ¶§
  {
    MatrixXd inv_S1(S_val_n,S_val_n);
    inv_S1 = S1.inverse();

    invQ = V1*inv_S1*U1_T;
  }
  else if(S_val_n < Q_col) // QÀÇ Çà ºžŽÙ S vector ŒººÐÀÇ ŒýÀÚ°¡ ÀûÀ» ¶§(¿©À¯ ÀÚÀ¯µµ°¡ ÀÖÀœ) - when Q is fat matrix
  {
    MatrixXd Id(Q_col,Q_col);
    Id.setIdentity();
    MatrixXd tmp((Q_col-S_val_n),(Q_col-S_val_n));
    tmp = V2_T*W*V2;
    MatrixXd tmp_inv((Q_col-S_val_n),(Q_col-S_val_n));

    if(ContactState == 0) //double contact
    {
      //tmp_inv = OneSidedInverse(tmp);
      tmp_inv = pinv_SVD(tmp,1.e-10);
     // tmp_inv = pinv_QR(tmp);
    }
    else if(ContactState == 1 || ContactState == 2) //single contact
    {
      tmp_inv = tmp.inverse();
    }
    else //else.. require for arm contact for humanods
    {
      //tmp_inv = OneSidedInverse(tmp);
      tmp_inv = pinv_SVD(tmp,1.e-10);
      //tmp_inv = pinv_QR(tmp);
    }

    MatrixXd inv_S1(S_val_n,S_val_n);
    inv_S1 = S1.inverse();

    invQ = (Id-V2*tmp_inv*V2_T*W)*V1*inv_S1*U1_T;
  }
  return invQ;
}

void CWholeBodyControl::updateLocaltoGlobalDynamicsParameters(const HTransform &Tbase, const VectorXd &localG, const MatrixXd &localC, const MatrixXd &localA)
{
  _globalT = Tbase;
  //get local values
  _localG = localG;
  _localC = localC;
  _localA = localA;
  _bDynamicParameter = true;
  //calc global values
  calculateFloatingBaseGlobalGravityVector();
  calculateFloatingBaseGlobalInertiaMatrix();
}

void CWholeBodyControl::updateLocaltoGlobalKinematicsParameters(const Vector6D &xdot_base, const VectorXd &qdot)
{
  for(int i=0; i<6; i++)
  {
    _qdot(i) = xdot_base(i);
  }
  for(int i=0; i<_JOINTNUM; i++)
  {
    _qdot(i+6) = qdot(i);
  }
  _localb = _localC*_qdot;
  calculateFloatingBaseGlobalCoriolisVector();
}

void CWholeBodyControl::calculateFloatingBaseGlobalGravityVector()
{
  _globalG = -_localG;

  Vector6D tmp1Vec6;
  tmp1Vec6.setZero();
  for(int i=0; i<6; i++)
  {
    tmp1Vec6(i) = -_localG(i);
  }

  Matrix6D Rotation;
  Rotation.setZero();

  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3;j++)
    {
      Rotation(i,j)= _globalT(i,j);
      Rotation(i+3,j+3)=_globalT(i,j);
    }
  }

  Vector6D tmp2Vec6;
  tmp2Vec6.setZero();
  tmp2Vec6 = Rotation*tmp1Vec6;

  for(int i=0; i<6; i++)
  {
    _globalG(i) = tmp2Vec6(i);
  }
}


void CWholeBodyControl::calculateFloatingBaseGlobalCoriolisVector()
{
  _globalb = -_localb;

  Vector6D tmp1Vec6;
  tmp1Vec6.setZero();
  for(int i=0; i<6; i++)
  {
    tmp1Vec6(i) = -_localb(i);
  }

  Matrix6D Rotation;
  Rotation.setZero();
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3;j++)
    {
      Rotation(i,j)= _globalT(i,j);
      Rotation(i+3,j+3)=_globalT(i,j);
    }
  }

  Vector6D tmp2Vec6;
  tmp2Vec6.setZero();
  tmp2Vec6 = Rotation*tmp1Vec6;

  for(int i=0; i<6; i++)
  {
    _globalb(i) = tmp2Vec6(i);
  }
}

void CWholeBodyControl::calculateFloatingBaseGlobalInertiaMatrix()
{
  MatrixXd Rotation(6+_JOINTNUM,6+_JOINTNUM);
  Rotation.setZero();

  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3;j++)
    {
      Rotation(i,j)= _globalT(i,j);
      Rotation(i+3,j+3)=_globalT(i,j);
    }
  }

  for(int i=6; i<(6+_JOINTNUM); i++)
  {
    Rotation(i,i) = 1.0;
  }

  MatrixXd Rotation_T(6+_JOINTNUM,6+_JOINTNUM);

  Rotation_T=Rotation.transpose();

  _globalA = Rotation*_localA*Rotation_T;

  _globalAinv = _globalA.inverse();
}
/*
void CWholeBodyControl::calculateFloatingBaseGlobalDynamics()
{
  if(_bDynamicParameter == true)
  {
    calculateFloatingBaseGlobalGravityVector();
    calculateFloatingBaseGlobalInertiaMatrix();
    calculateFloatingBaseGlobalCoriolisVector();
  }
  else
  {
    //error massage
  }
}
*/

void CWholeBodyControl::updateGlobalDynamicsParameters(const VectorXd &globalG, const VectorXd &globalC, const MatrixXd &globalA)
{
  _globalG = globalG;
  _globalb = globalC;
  _globalA = globalA;
  _globalAinv = _globalA.inverse();
  _bDynamicParameter = true;
}

void CWholeBodyControl::updateGlobalKinematicsParameters(const Vector6D &xdot_base, const VectorXd &qdot)
{
  for(int i=0; i<6; i++)
  {
    _qdot(i) = xdot_base(i);
  }
  for(int i=0; i<_JOINTNUM; i++)
  {
    _qdot(i+6) = qdot(i);
  }
}

void CWholeBodyControl::updateContactParameters(const MatrixXd &Jc,const MatrixXd &Jc_dot,const int &ContactState)
{

  _Jcrow = Jc.rows();
  _Jccol = Jc.cols();
  //_Jcrow = Jcrow;
  //_Jccol = Jccol;

  _Jc.resize(_Jcrow,_Jcrow);
  _Jc = Jc;
//	_Jc.setZero();
//	for(int i=0; i<Jcrow; i++)
//	{
//		for(int j=0; j<Jccol; j++)
//		{
//			_Jc(i,j) = Jc(i,j);
//		}
//	}

  _Jc_dot.resize(_Jcrow,_Jcrow);
  _Jc_dot = Jc_dot;
//	_Jc_dot.setZero();
//	for(int i=0; i<Jcrow; i++)
//	{
//		for(int j=0; j<Jccol; j++)
//		{
//			_Jc_dot(i,j) = Jc_dot(i,j);
//		}
//	}
  _ContactState = ContactState;
  _bContactParameter = true;
}

void CWholeBodyControl::updateTaskParameters(const MatrixXd &J, const int &TaskState)
{
  _Jrow = J.rows();
  _Jcol = J.cols();
  //_Jrow = Jrow;
  //_Jcol = Jcol;
  _J.resize(_Jrow,_Jcol);
  _J = J;

  _TaskState = TaskState;
  _bTaskParameter = true;
}


void CWholeBodyControl::calculateContactSpaceInvDynamics()
{
  if(_bContactParameter == true)
  {
    ////////////////////////////////////// claculate Jc bar transpose
    MatrixXd Jc_T(_Jccol, _Jcrow);
    Jc_T = _Jc.transpose();// calculate Jcg transpose

    MatrixXd Tmp_c(_Jcrow, _Jcrow);
    Tmp_c = _Jc*_globalAinv*Jc_T;

    _lambda_c.resize(_Jcrow, _Jcrow);
    _lambda_c.setZero();
    if(_ContactState == 0) //two foot contact
    {
      //_lambda_c = OneSidedInverse(Tmp_c);
      _lambda_c = pinv_SVD(Tmp_c,1.e-10);
      //_lambda_c = pinv_QR(Tmp_c);
    }
    else if(_ContactState == 1 || _ContactState == 2) //one foot contact
    {
      _lambda_c = Tmp_c.inverse();//calculate lamba c, À§ÀÇ Tmp_cgžŠ inverse
    }
    else
    {
      //_lambda_c = OneSidedInverse(Tmp_c);//calculate lamba c, À§ÀÇ Tmp_cgžŠ inverse(pseudo inverse »ç¿ë)
      _lambda_c = pinv_SVD(Tmp_c,1.e-10);
      //_lambda_c = pinv_QR(Tmp_c);
    }

    _Jc_bar_T.resize(_Jcrow, _Jccol);
    _Jc_bar_T.setZero();
    _Jc_bar_T = _lambda_c*_Jc*_globalAinv; // claculate Jc bar transpose

    _pc.resize(_Jcrow);
    _pc = _Jc_bar_T*_globalG; // calculate pc(projection of the gravity forces at the contact)

    //////////////////////////////////////// claculate Pc
    _Pc.resize(_Jccol, _Jccol);
    _Pc = Jc_T*_Jc_bar_T;
  }
  else
  {
    //error massage
  }
}

void CWholeBodyControl::calculateGravityCoriolisCompensationTorque()
{
  ////////////////////////////////////////////////////////// Constrained Dynamics of the constraint operational space
  MatrixXd Id_Jg(_JOINTNUM, _JOINTNUM);//form Identity matrix for total joints
  Id_Jg.setIdentity();

  MatrixXd Id_tot(_JOINTNUM+6, _JOINTNUM+6);//form Identity matrix for total joints including virtual joints
  Id_tot.setIdentity();

  //////////////////////////////////////// calculate J for gravity & Coriolis/centrifugal compensate
  MatrixXd Jg(_JOINTNUM, _JOINTNUM+6); //Jacobian
  Jg.setZero();

  for (int i=0; i<_JOINTNUM; i++)
  {
    for (int j=0; j<_JOINTNUM; j++)
    {
      Jg(i,j+6) = Id_Jg(i,j);
    }
  }

  //////////////////////////////////////// calculate transpose J
  MatrixXd Jg_T(_JOINTNUM+6, _JOINTNUM);
  Jg_T = Jg.transpose();

  //////////////////////////////////////// claculate J bar transpose
  MatrixXd Tmpg(_JOINTNUM, _JOINTNUM);
  Tmpg = Jg*_globalAinv*(Id_tot-_Pc)*Jg_T;

  //////////////////////////////////////// calculate lambda for task, À§ÀÇ _TmpgÀÇ inverse
  if(_ContactState == 0) //two foot contact
  {
    //_LambdaJointSpace = OneSidedInverse(Tmpg);
    _LambdaJointSpace = pinv_SVD(Tmpg,1.e-10);
    //_LambdaJointSpace = pinv_QR(Tmpg);
  }
  else if(_ContactState == 1 || _ContactState == 2) //single support
  {
    _LambdaJointSpace = Tmpg.inverse();
  }
  else //±× ¿ÜÀÇ °æ¿ì, ŸÈÀüÀ» À§ÇØ pinv ŒöÇà
  {
    //_LambdaJointSpace = OneSidedInverse(Tmpg);
    _LambdaJointSpace = pinv_SVD(Tmpg,1.e-10);
    //_LambdaJointSpace = pinv_QR(Tmpg);
  }
  _Jjoint_bar_T.setZero();
  _Jjoint_bar_T = _LambdaJointSpace*Jg*_globalAinv*(Id_tot-_Pc);

  //////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////// calculate gravity compensation torque
  //////////////////////////////////////////////////////////////////////////////////////////////

  _GravityCompensationTorque.resize(_JOINTNUM);
  _GravityCompensationTorque.setZero();
  _GravityCompensationTorque = _Jjoint_bar_T*_globalG; // torque for gravity compensation

  //////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////// calculate Coriolis/centrifugal compensation torque
  //////////////////////////////////////////////////////////////////////////////////////////////

  _CoriolisCompensationTorque.resize(_JOINTNUM);
  _CoriolisCompensationTorque.setZero();
  _CoriolisCompensationTorque = _Jjoint_bar_T*_globalb + _LambdaJointSpace*Jg*_globalAinv*_Jc.transpose()*_lambda_c*_Jc_dot*_qdot;
}

void CWholeBodyControl::calculateContactConstrainedTaskJacobian()
{
  MatrixXd Id_tot(_JOINTNUM+6, _JOINTNUM+6);//form Identity matrix for total joints including virtual joints
  Id_tot.setIdentity();

  //////////////////////////////////////// calculate transpose J
  MatrixXd J_T(_JOINTNUM+6, _Jrow);
  J_T = _J.transpose();

  //////////////////////////////////////// claculate J bar transpose
  MatrixXd Tmp(_Jrow, _Jrow);
  Tmp = _J*_globalAinv*(Id_tot-_Pc)*J_T;

  _Lambda.resize(_Jrow, _Jrow);
  _Lambda.setZero();

  if(_TaskState == 0) //comžž ÁŠŸî
  {
    _Lambda = Tmp.inverse();
  }
  else if(_TaskState == 1 || _TaskState == 2) //com + foot ÁŠŸî
  {
    //_Lambda = OneSidedInverse(Tmp);
    _Lambda = pinv_SVD(Tmp,1.e-10);
    //_Lambda = pinv_QR(Tmp);

  }
  else
  {
    //_Lambda = OneSidedInverse(Tmp);
    _Lambda = pinv_SVD(Tmp,1.e-10);
    //_Lambda = pinv_QR(Tmp);
  }

  _J_bar_T.resize(_Jrow, _JOINTNUM+6);
  _J_bar_T = _Lambda*_J*_globalAinv*(Id_tot-_Pc);
  ///////////////////////////////////////////////////////////calculate Jk transpose

  MatrixXd J_bar_T_S_k_T(_Jrow,_JOINTNUM);
  J_bar_T_S_k_T = _J_bar_T*_S_k_T;

  _W = _S_k*_globalAinv*(Id_tot-_Pc)*_S_k_T;

  _J_k_T.resize(_JOINTNUM,_Jrow);
  _J_k_T = GenInvSemiPosDef(_ContactState, J_bar_T_S_k_T,_W); //if W is semi positive definite, W°¡ positive definiteÀÏ ¶§µµ »ç¿ë °¡ŽÉ
}

void CWholeBodyControl::calculateNullSpaceJacobian()
{
  MatrixXd Id_joints(_JOINTNUM,_JOINTNUM);
  Id_joints.setZero();
  for(int i=0; i<_JOINTNUM; i++)
  {
    Id_joints(i,i) = 1.0;
  }

  _N_k_T = Id_joints- _J_k_T*_J_bar_T*_S_k_T;
}

void CWholeBodyControl::calculateSVDofWeightingMatrix()
{
  MatrixXd W1 = _W;
  if(_Jrow >= _JOINTNUM)
  {
    //printf("Overdetermined...\n");
  }
  else
  {
    MatrixXd W_eigen(_JOINTNUM,_JOINTNUM);
    for(int i=0; i<_JOINTNUM; i++)
    {
      for(int j=0; j<_JOINTNUM; j++)
      {
        W_eigen(i,j) = _W(i,j);
      }
    }

    JacobiSVD<MatrixXd> svd(W_eigen, ComputeFullU | ComputeFullV);

    int S_val_n=0;
    double threshold = 0.0001;
    for(int i=0; i<_JOINTNUM; i++)
    {
      if(svd.singularValues()(i) > threshold)
      {
        S_val_n = S_val_n+1;
      }
    }

    //dMatrix U(jointnum,jointnum);
    //U = svd.matrixU();
    MatrixXd V_T(_JOINTNUM,_JOINTNUM);
    V_T = svd.matrixV().transpose();

    _V2row = _JOINTNUM;
    _V2col = _JOINTNUM - S_val_n;

    //V_T.rreduce(0,S_val_n); //null space ºÎºÐžž ³²±â°í ÁŠ°Å
    MatrixXd V2_T(_V2col,_V2row);
    for(int i=0; i<_V2col; i++)
    {
      for(int j=0; j<_V2row; j++)
      {
        V2_T(i,j) = V_T(i+S_val_n,j);
      }
    }

    _V2forContactWrenchRedistribution.resize(_V2row,_V2col);
    //V2.setZero();updateTaskParameters
    _V2forContactWrenchRedistribution = V2_T.transpose();


    //dMatrix tmp(_Jrow,V2_col);
    //tmp = _Scf_Jc_bar_T_S_k_T*V2;

    //dMatrix tmp_inv(V2_col,_Jrow);
    //tmp.pinv(tmp_inv);
    //tmp.inv(tmp_inv);
    //GenInvSemiPosDef(tmp, tmp_row, tmp_col, W, tmp_inv); //inversežŠ ÇØŸßÇÏŽÂ matrix(input Á€ºž), QÀÇ ÇàÀÇ Œö, QÀÇ ¿­ÀÇ Œö, W matrix(input Á€ºž) , inverseµÈ °á°ú matrix(output Á€ºž)

    //dVector alpha(V2_col);
    //alpha = tmp_inv*desiredValue;

    //solutionValue.resize(_JOINTNUM);
    //solutionValue = V2*alpha;
  }
}

void CWholeBodyControl::calculateContactWrenchLocalContactFrame(const VectorXd &Torque)
{
  _Fc_LocalContactFrame.resize(_ContactState);
  _Fc_LocalContactFrame.setZero();
  _Fc_LocalContactFrame = _Jc_bar_T*_S_k_T*(Torque) - _pc; //expected contact wrench
}


void CWholeBodyControl::calculateContactWrenchRedistributionTorque(const MatrixXd &Scw, const VectorXd &_Fc_compensate_d, VectorXd &Torque)
{

  int Scwrow = Scw.rows();
  int V2col = _V2forContactWrenchRedistribution.cols();
  MatrixXd tmp(Scwrow,V2col);
  tmp = Scw*_Jc_bar_T*_S_k_T*_V2forContactWrenchRedistribution;

  MatrixXd tmp_inv(V2col,Scwrow);
  tmp_inv.setZero();

  if(V2col == Scwrow)
  {
    tmp_inv = tmp.inverse();
  }
  else
  {
    tmp_inv = pinv_SVD(tmp,1.e-10);
  }

  VectorXd alpha(V2col);
  alpha = tmp_inv*Scw*_Fc_compensate_d;

  //Torque.resize(_JOINTNUM);
  Torque = _V2forContactWrenchRedistribution*alpha;  
}

