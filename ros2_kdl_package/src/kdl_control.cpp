#include "kdl_control.h"
#include <iostream>
#include <cmath> 
#include <kdl/frames.hpp>
#include <Eigen/Dense>

// ================== Costanti Globali per Vision Control ==================
// Guadagno per il controllo del Rollio nello spazio nullo. 
const double ROLL_GAIN = 15.0; 
// Tolleranza per l'errore di Rollio (~1 grado = 0.017 rad)
const double ROLL_TOLERANCE = 0.01; 
// Tolleranza per l'ARRESTO del Centramento (5 mm)
const double TOLERANCE_LAP = 0.005; 
// Tolleranza per la RIATTIVAZIONE del Centramento (12 mm)
const double TOLERANCE_LAP_RESUME = 0.005; 

// ================== Funzioni di Utilità Pubbliche ==================

// ===== Implementazione della funzione pseudoinverse (Damped Least Squares - DLS) =====
Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd &J)
{
    const double lambda = 1e-2;
    const double lambda_sq = lambda * lambda; 
    
    Eigen::MatrixXd J_T = J.transpose();
    Eigen::MatrixXd B = J * J_T;
    
    // Aggiungi la matrice identit\u00e0 smorzata
    B += lambda_sq * Eigen::MatrixXd::Identity(B.rows(), B.cols());
    
    return J_T * B.inverse();
}

// ================== Funzioni di Utilit\u00e0 Private ==================

Eigen::Matrix3d KDLController::skew_symmetric(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d S;
    S << 0, -v(2), v(1),
         v(2), 0, -v(0),
        -v(1), v(0), 0;
    return S;
}

// ================== Funzioni KDLController Implementate ==================

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd * de + _Kp * e)
           + robot_->getCoriolis() + robot_->getGravity();
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &/*_desPos*/,
                                      KDL::Twist &/*_desVel*/,
                                      KDL::Twist &/*_desAcc*/,
                                      double /*_Kpp*/, double /*_Kpo*/,
                                      double /*_Kdp*/, double /*_Kdo*/)
{
    Eigen::VectorXd tau;
    return tau;
}

Eigen::VectorXd KDLController::velocity_ctrl(const Eigen::VectorXd &error,
                                             const Eigen::MatrixXd &J,
                                             double Kp)
{
    Eigen::MatrixXd J_pinv = pseudoinverse(J);
    Eigen::VectorXd q_dot = J_pinv * (Kp * error);
    return q_dot;
}

Eigen::VectorXd KDLController::velocity_ctrl_null(const Eigen::VectorXd &e,
                                                 const Eigen::MatrixXd &J,
                                                 const Eigen::VectorXd &q_dot_null,
                                                 const Eigen::VectorXd &q_curr,
                                                 const Eigen::VectorXd &q_des_null,
                                                 double Kp, double Kp_null)
{
    Eigen::MatrixXd J_pinv = pseudoinverse(J);
    
    Eigen::VectorXd q_dot_primary = J_pinv * (Kp * e);
    Eigen::VectorXd q_dot_null_task = Kp_null * (q_des_null - q_curr) + q_dot_null;

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(J.cols(), J.cols());
    Eigen::VectorXd q_dot_secondary = (I - J_pinv * J) * q_dot_null_task;
    
    return q_dot_primary + q_dot_secondary;
}


// ===== Controllo in velocit\u00e0 basato su Visione (LAP + Rollio + JLA) - STABILIZZATO E FINALIZZATO =====
Eigen::VectorXd KDLController::vision_ctrl(
    const Eigen::Vector3d &Pc_curr, 
    const Eigen::MatrixXd &J_curr,  
    const KDL::Rotation& Rc_curr,   
    double Kp,                      
    double Kp_null,                 
    const Eigen::VectorXd &q_curr,
    const Eigen::VectorXd &q_min,
    const Eigen::VectorXd &q_max)
{
    int N_jnts = J_curr.cols();

    // 1. Task Primario: Centramento Look-at-Point (LAP)
    
    double rho = Pc_curr.norm(); 
    Eigen::Vector3d e_vis = Eigen::Vector3d::Zero();
    bool lap_finished = false;

    // Variabili dichiarate nello scope superiore
    Eigen::Vector3d s;
    Eigen::Vector3d sd;
    sd << 0.0, 0.0, 1.0; 

    if (rho >= 0.01) { 
        s = Pc_curr / rho; 
        e_vis = sd - s; 
        double e_lap_norm = e_vis.norm();

        // Logica di ISTERESI per stabilizzare il centramento:
        if (e_lap_norm < TOLERANCE_LAP) 
        {
            // 1) Sotto 5mm: Task Completato, Errore zero
            e_vis = Eigen::Vector3d::Zero();
            lap_finished = true;
        } else if (e_lap_norm > TOLERANCE_LAP_RESUME) {
            // 2) Sopra 12mm: Task Attivo, Errore mantenuto
            lap_finished = false;
        } else {
             // 3) Tra 5mm e 12mm (Zona Morta): Task Completato, Errore zero
             e_vis = Eigen::Vector3d::Zero();
             lap_finished = true; // Correzione per la zona morta
        }

    } else {
        // Se siamo troppo vicini/su rho=0, consideriamo il task completato
        s = sd; 
        lap_finished = true; 
    }
    
    // Matrice di Interazione L_lap (3x6)
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d S_s = skew_symmetric(s); 
    Eigen::Matrix3d Projection = I - s * s.transpose(); 

    Eigen::Matrix<double, 3, 6> L_lap_temp;
    L_lap_temp.block<3, 3>(0, 0) = S_s; 
    L_lap_temp.block<3, 3>(0, 3) = I;   
    Eigen::Matrix<double, 3, 6> L_lap = (-1.0 / (rho == 0.0 ? 1.0 : rho)) * Projection * L_lap_temp; 
    
    Eigen::Matrix3d Rc_eigen;
    for (int i = 0; i < 3; ++i) 
        for (int j = 0; j < 3; ++j) 
            Rc_eigen(i, j) = Rc_curr(i, j);
    
    Eigen::MatrixXd T_v = Eigen::MatrixXd::Zero(6, 6);
    T_v.block<3, 3>(0, 0) = Rc_eigen.transpose();
    T_v.block<3, 3>(3, 3) = Rc_eigen.transpose();

    // Jacobiana Totale A_prim (3x7) = L_lap * T_v * J_curr
    Eigen::MatrixXd A_prim = L_lap * T_v * J_curr; 
    Eigen::MatrixXd A_dagger = pseudoinverse(A_prim); 

    Eigen::VectorXd q_dot_prim = A_dagger * (-Kp * e_vis); 

    // Matrice di Proiezione del Null Space Primario
    Eigen::MatrixXd I_jnts = Eigen::MatrixXd::Identity(N_jnts, N_jnts);
    Eigen::MatrixXd N_prim = I_jnts - A_dagger * A_prim;

    // 2. Task Secondario 1 : Controllo del Rollio

    // A. Errore di Rollio
    double roll, pitch, yaw;
    Rc_curr.GetRPY(roll, pitch, yaw); 
    
    double roll_des = 0.0; 
    double e_roll = std::fmod(yaw - roll_des + M_PI, 2.0 * M_PI) - M_PI; 
    
    bool roll_finished = (std::fabs(e_roll) < ROLL_TOLERANCE);

    // B. Jacobiana del Rollio
    Eigen::MatrixXd J_roll(1, N_jnts);
    J_roll.row(0) = J_curr.row(5); 

    // C. Velocità desiderata per il Rollio
    Eigen::VectorXd v_roll_des(1);
    
    if (roll_finished) {
        v_roll_des(0) = 0.0;
    } else {
        v_roll_des(0) = -ROLL_GAIN * e_roll; 
    }

    // D. Proiezione del Rollio nello Spazio Nullo Primario (N_prim)
    Eigen::MatrixXd J_roll_N = J_roll * N_prim; 
    Eigen::MatrixXd J_roll_N_dagger = pseudoinverse(J_roll_N);

    // E. Velocità dei giunti per correggere il Rollio
    Eigen::VectorXd q_dot_roll = J_roll_N_dagger * v_roll_des;

    // F. Matrice di Proiezione del Null Space Residuo (dopo LAP e Rollio)
    Eigen::MatrixXd N_roll = N_prim - J_roll_N_dagger * J_roll_N; 


    // 3. Task Secondario 2 : Joint Limit Avoidance (JLA)


    Eigen::VectorXd q0_gradient = Eigen::VectorXd::Zero(N_jnts);
    for (int i = 0; i < N_jnts; ++i)
    {
        double qi = q_curr(i);
        double qi_plus = q_max(i);
        double qi_minus = q_min(i);
        double denom = (qi_plus - qi) * (qi - qi_minus);
        
        if (std::fabs(denom) < 1e-6) denom = 1e-6;
        
        q0_gradient(i) = std::pow(qi_plus - qi_minus, 2) * (qi_plus + qi_minus - 2.0 * qi) / (denom * denom);
    }
    
    Eigen::VectorXd v_jla = Kp_null * q0_gradient; 
    Eigen::VectorXd q_dot_jla = N_roll * v_jla;
    
    // 4. Risultato Finale
    
    // Stampa di debug per il monitoraggio
    if (!lap_finished || !roll_finished) {
        std::cout << "[DEBUG] E_LAP: " << e_vis.norm() * 1000 << " mm | E_Roll (Yaw): " << e_roll * 180.0 / M_PI << " deg" << std::endl;
    }

    // Se entrambi i task primari (LAP e Rollio) sono completati, ferma TUTTO.
    if (lap_finished && roll_finished) {
        std::cout << "[INFO] MARKER ACCENTRATO : Centramento e Rollio OK." << std::endl;
        std::cout << "[INFO] TASK COMPLETATO." << std::endl;
        
        return Eigen::VectorXd::Zero(N_jnts); 
    }
    
    return q_dot_prim + q_dot_roll + q_dot_jla;
}


// ===== Generazione traiettoria lineare nello spazio cartesiano =====
std::vector<Eigen::Vector3d> KDLController::generateLinearTrajectory(const Eigen::Vector3d &start,
                                                             const Eigen::Vector3d &end,
                                                             int steps)
{
    std::vector<Eigen::Vector3d> trajectory;
    trajectory.reserve(steps);
    for (int i = 0; i < steps; ++i)
    {
        double alpha = static_cast<double>(i) / (steps - 1);
        Eigen::Vector3d p = (1 - alpha) * start + alpha * end;
        trajectory.push_back(p);
    }
    return trajectory;
}
