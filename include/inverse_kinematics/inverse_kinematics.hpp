//~ Copyright (C) 2023 Łukasz Woliński
//~ You may use, distribute and modify this code under the terms of the BSD-3-Clause License.

#ifndef INV_KIN_H
#define INV_KIN_H

#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
#include "helper_functions.hpp"

namespace rrlib
{
class InverseKinematics
{
public:
    InverseKinematics();
    void SetParameters( double dt, size_t DOF, double clik_gain_pos, double clik_gain_ori );
    bool areParametersOK();
    size_t GetDOF();
    void SolveIK( const Eigen::VectorXd& pose_FK, const Eigen::MatrixXd& J, const Eigen::VectorXd& pose_des, const Eigen::VectorXd& vel_des, Eigen::VectorXd* q, Eigen::VectorXd* dqdt );
    Eigen::VectorXd PoseError( const Eigen::VectorXd& pose_des, const Eigen::VectorXd& pose_FK );
    
private:
    double dt_;                     // time step
    size_t DOF_;                    // numbmer of degrees of freedom of the manipulator
    Eigen::MatrixXd clik_gain_;     // gain for the Closed Inverse Kinematics (CLIK)
    bool PARAMETERS_OK_;            // a flag to be set to true if the parameters are viable for the inverse kinematics computation, and false otherwise
};

InverseKinematics::InverseKinematics()
{
    /* Default parameters are zero. User shall use setParameters to assign them the proper values before computing the solution to the inverse kinematics problem.*/
    dt_ = 0.0;
    clik_gain_ = Eigen::MatrixXd::Zero(6, 6);
    PARAMETERS_OK_ = false;
}

void InverseKinematics::SetParameters( double dt, size_t DOF, double clik_gain_pos, double clik_gain_ori )
{
    dt_ = dt;
    DOF_ = DOF;
    clik_gain_ = Eigen::MatrixXd::Zero(6, 6);
    clik_gain_.block(0, 0, 3, 3) = clik_gain_pos * Eigen::Matrix3d::Identity();
    clik_gain_.block(3, 3, 3, 3) = clik_gain_ori * Eigen::Matrix3d::Identity();
    PARAMETERS_OK_ = true;
}

bool InverseKinematics::areParametersOK()
{
    return PARAMETERS_OK_;
}

size_t InverseKinematics::GetDOF()
{
    return DOF_;
}

void InverseKinematics::SolveIK( const Eigen::VectorXd& pose_FK, const Eigen::MatrixXd& J, const Eigen::VectorXd& pose_des, const Eigen::VectorXd& vel_des, Eigen::VectorXd* q, Eigen::VectorXd* dqdt )
{
    /* compute the pseudoinverse of the Jacobian */
    Eigen::MatrixXd pinv_J = J.completeOrthogonalDecomposition().pseudoInverse();
    /* utilize CLIK */
    Eigen::VectorXd pose_err = PoseError( pose_des, pose_FK );
    //~ pose_err.tail(3) = Eigen::Vector3d::Zero(); //temp
    Eigen::VectorXd v = vel_des + clik_gain_ * pose_err;
    /* compute the joint velocity */
    *dqdt = pinv_J * v;
    /* integrate the joint velocity to get the joint positon */
    *q = *q + *dqdt * dt_;
}

Eigen::VectorXd InverseKinematics::PoseError( const Eigen::VectorXd& pose_des, const Eigen::VectorXd& pose_FK )
{
    Eigen::VectorXd pose_err(6);
    pose_err.head(3) = pose_des.head(3) - pose_FK.head(3);
    Eigen::Vector4d ori_des = pose_des.tail(4);
    Eigen::Vector4d ori_FK = pose_FK.tail(4);
    pose_err.tail(3) = -ori_des(3) * ori_FK.head(3) + ori_FK(3) * ori_des.head(3) - Eigen::Vector3d(ori_des.head(3)).cross(Eigen::Vector3d(ori_FK.head(3)));
    
    return pose_err;
}

} // namespace rrlib

#endif // INV_KIN_H
