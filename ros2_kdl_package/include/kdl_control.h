#ifndef KDL_CONTROL_H
#define KDL_CONTROL_H

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <Eigen/Dense>
#include <vector>

#include "kdl_robot.h"

using namespace Eigen;

// Dichiarazione della pseudoinversa (definita in kdl_control.cpp)
Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd &J);

class KDLController
{
public:
    KDLController(KDLRobot &_robot);
    
    // Joint Space Inverse Dynamics Controller (pos, vel, acc)
    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp, double _Kd);

    // Cartesian Space Inverse Dynamics Controller (pos, vel, acc)
    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp, double _Kpo,
                           double _Kdp, double _Kdo);

    // Velocity Control (e, J, Kp)
    Eigen::VectorXd velocity_ctrl(const Eigen::VectorXd &error,
                                  const Eigen::MatrixXd &J,
                                  double Kp);

    // Velocity Control with Null Space (e, J, q_dot_null, q_curr, q_des_null, Kp, Kp_null)
    // Ho rimosso l'implementazione obsoleta in favore di una pi\u00f9 semplice (come usata nel file principale)
    Eigen::VectorXd velocity_ctrl_null(const Eigen::VectorXd &e,
                                       const Eigen::MatrixXd &J,
                                       const Eigen::VectorXd &q_dot_null,
                                       const Eigen::VectorXd &q_curr,
                                       const Eigen::VectorXd &q_des_null,
                                       double Kp, double Kp_null);
    
    // Vision-Based Look-at-Point Control (Task Primario + JLA Spazio Nullo)
    // Firma aggiornata per consentire l'uso di Kp_null e limiti di giunto per il calcolo del gradiente.
    Eigen::VectorXd vision_ctrl(
        const Eigen::Vector3d &Pc_curr, // Posizione dell'oggetto nel frame telecamera
        const Eigen::MatrixXd &J_curr,  // Jacobiana del robot (6xN)
        const KDL::Rotation& Rc_curr,   // Rotazione telecamera/base
        double Kp,                      // Guadagno task primario (visione)
        double Kp_null,                 // Guadagno/Lambda per Joint Limit Avoidance (JLA)
        const Eigen::VectorXd &q_curr,  // Posizioni attuali dei giunti
        const Eigen::VectorXd &q_min,   // Limiti minimi dei giunti
        const Eigen::VectorXd &q_max);  // Limiti massimi dei giunti


    // Trajectory generation
    std::vector<Eigen::Vector3d> generateLinearTrajectory(const Eigen::Vector3d &start,
                                                         const Eigen::Vector3d &end,
                                                         int steps);

private:
    KDLRobot *robot_;
    Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v);
};

#endif // KDL_CONTROL_H
