#include <fstream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <algorithm> // Necessario per std::min

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "kdl_parser/kdl_parser.hpp"
#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

// Action include
#include "ros2_kdl_package/action/linear_traj.hpp"

using LinearTraj = ros2_kdl_package::action::LinearTraj;
using GoalHandleLinearTraj = rclcpp_action::ServerGoalHandle<LinearTraj>;

// Funzione di utilit\u00e0 esterna (assumendo che sia definita in kdl_control.h o altrove)
// Nota: pseudoinverse \u00e8 ora in kdl_control.cpp
extern Eigen::MatrixXd pseudoinverse(const Eigen::MatrixXd &J); 
extern std::vector<double> toStdVector(const Eigen::VectorXd &vec);


class Iiwa_pub_sub_action : public rclcpp::Node
{
public:
    Iiwa_pub_sub_action() : Node("ros2_kdl_node_action")
    {
        // ================== Parametri ROS2 ==================
        declare_parameter("cmd_interface", "velocity");
        declare_parameter("traj_type", "linear");
        declare_parameter("s_type", "trapezoidal");
        declare_parameter("traj_duration", 1.5);
        declare_parameter("acc_duration", 0.5);
        declare_parameter("total_time", 3.0);
        declare_parameter("trajectory_len", 75);
        declare_parameter("Kp", 0.5);
        declare_parameter("Kp_null", 0.001); 
        declare_parameter("end_position", std::vector<double>({0.5, -0.2, 0.6}));
        declare_parameter("lambda", 0.1); // Usato come Kp_null/lambda nel Vision Ctrl
        declare_parameter("ctrl_mode", "velocity_ctrl");
        declare_parameter("auto_start", true); 

        get_parameter("cmd_interface", cmd_interface_);
        get_parameter("traj_type", traj_type_);
        get_parameter("s_type", s_type_);
        get_parameter("traj_duration", traj_duration_);
        get_parameter("acc_duration", acc_duration_);
        get_parameter("total_time", total_time_);
        get_parameter("trajectory_len", trajectory_len_);
        get_parameter("Kp", Kp_);
        get_parameter("Kp_null", Kp_null_); 
        get_parameter("end_position", end_position_);
        get_parameter("lambda", lambda_);
        get_parameter("ctrl_mode", ctrl_mode_);
        get_parameter("auto_start", auto_start_);

        RCLCPP_INFO(this->get_logger(), "Controller mode: %s", ctrl_mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "Command interface: %s", cmd_interface_.c_str());
        RCLCPP_INFO(this->get_logger(), "Auto start: %s", auto_start_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Kp_null: %.3f", Kp_null_); 

        // ================== Robot description ==================
        std::string robot_description;
        try
        {
            auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "/robot_state_publisher");
            RCLCPP_INFO(this->get_logger(), "Waiting for /robot_state_publisher service...");
            if (!parameters_client->wait_for_service(10s))
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout: /robot_state_publisher non disponibile!");
                throw std::runtime_error("robot_state_publisher not available");
            }

            auto future = parameters_client->get_parameters({"robot_description"});
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s)
                != rclcpp::FutureReturnCode::SUCCESS)
            {
                throw std::runtime_error("Failed to get robot_description parameter");
            }

            auto results = future.get();
            if (!results.empty())
            {
                robot_description = results[0].value_to_string();
                if (robot_description.size() > 1 && robot_description.front() == '"' && robot_description.back() == '"')
                    robot_description = robot_description.substr(1, robot_description.size() - 2);
            }

            if (robot_description.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Parameter 'robot_description' is empty!");
                throw std::runtime_error("Missing robot_description parameter");
            }

            RCLCPP_INFO(this->get_logger(), "Robot description received (%zu chars)", robot_description.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Exception: %s", e.what());
            throw;
        }

        // ================== Costruzione modello KDL ==================
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(robot_description, robot_tree))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to parse robot_description into KDL Tree!");
            throw std::runtime_error("KDL parse error");
        }

        robot_ = std::make_shared<KDLRobot>(robot_tree);
        unsigned int nj = robot_->getNrJnts();
        RCLCPP_INFO(this->get_logger(), "KDL robot model created with %u joints", nj);

        // ================== Limiti giunto ==================
        KDL::JntArray q_min(nj), q_max(nj);
        if (nj == 7)
        {
            q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96;
            q_max.data <<  2.96,  2.09,  2.96,  2.09,  2.96,  2.09,  2.96;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unexpected joint count (%u). Using default \u00b13.14 limits.", nj);
            for (unsigned int i = 0; i < nj; ++i)
            {
                q_min(i) = -3.14;
                q_max(i) = 3.14;
            }
        }
        robot_->setJntLimits(q_min, q_max);
        q_min_ = q_min;
        q_max_ = q_max;

        // ================== Inizializzazione vettori ==================
        joint_positions_ = KDL::JntArray(nj);
        joint_velocities_ = KDL::JntArray(nj);
        joint_positions_cmd_ = KDL::JntArray(nj);
        joint_velocities_cmd_ = KDL::JntArray(nj);
        joint_efforts_cmd_ = KDL::JntArray(nj);
        joint_efforts_cmd_.data.setZero();
        joint_positions_prev_ = KDL::JntArray(nj);
        desired_commands_ = std::vector<double>(nj, 0.0);
        cPo_.setZero();
        Rc_kdl_ = KDL::Rotation::Identity();

        // ================== Subscriber /joint_states ==================
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&Iiwa_pub_sub_action::joint_state_subscriber, this, std::placeholders::_1));

        RCLCPP_WARN(this->get_logger(), "Waiting for first /joint_states...");

        // ================== Subscriber ArUco Pose ==================
        aruco_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_detect/pose", 10,
            std::bind(&Iiwa_pub_sub_action::aruco_pose_subscriber, this, std::placeholders::_1));

        RCLCPP_WARN(this->get_logger(), "Waiting for /aruco_single/pose...");

        // ================== Timer e variabili ==================
        t_ = 0.0;
        iteration_ = 0;

        // ================== CSV Logging ==================
        std::string csv_filename;
        if (ctrl_mode_ == "velocity_ctrl") csv_filename = "log_vel.csv";
        else if (ctrl_mode_ == "velocity_ctrl_null") csv_filename = "log_null.csv";
        else if (ctrl_mode_ == "vision_ctrl") csv_filename = "log_vision.csv"; 
        else csv_filename = "log_default.csv";
        
        csv_file_.open(csv_filename, std::ios::out);
        if (!csv_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open CSV file for logging!");
        }

        // ================== Action Server ==================
        action_server_ = rclcpp_action::create_server<LinearTraj>(
            this,
            "linear_traj",
            std::bind(&Iiwa_pub_sub_action::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&Iiwa_pub_sub_action::handle_cancel, this, std::placeholders::_1),
            std::bind(&Iiwa_pub_sub_action::handle_accepted, this, std::placeholders::_1)
        );
    }

private:
    // ================== Membri per velocit\u00e0 numerica ==================
    KDL::JntArray joint_positions_prev_;

    // ================== Membri per Vision Control ==================
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_;
    Eigen::Vector3d cPo_; // Posizione dell'oggetto nel frame telecamera
    KDL::Rotation Rc_kdl_; // Rotazione del frame telecamera (utilizzata in vision_ctrl)
    bool aruco_pose_available_ = false;

    // ================== Action server callbacks ==================
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const LinearTraj::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal to move to [%.3f, %.3f, %.3f]",
                    goal->target_position.x, goal->target_position.y, goal->target_position.z);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLinearTraj> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<GoalHandleLinearTraj> goal_handle)
    {
        std::thread{std::bind(&Iiwa_pub_sub_action::execute_goal, this, goal_handle)}.detach();
    }

    void execute_goal(const std::shared_ptr<GoalHandleLinearTraj> goal_handle)
    {
        auto feedback = std::make_shared<LinearTraj::Feedback>();
        auto result = std::make_shared<LinearTraj::Result>();

        robot_->setJntState(joint_positions_);
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        KDL::Frame cartpos = robot_->getEEFrame();
        Eigen::Vector3d current_EE(cartpos.p.data[0], cartpos.p.data[1], cartpos.p.data[2]);
        Eigen::Vector3d end_EE(goal_handle->get_goal()->target_position.x,
                                 goal_handle->get_goal()->target_position.y,
                                 goal_handle->get_goal()->target_position.z);

        KDLPlanner planner(traj_duration_, acc_duration_, current_EE, end_EE);
        trajectory_point p;
        p.pos = current_EE;

        if (!cmdPublisher_)
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
        }

        double t = 0.0;
        double dt = total_time_ / trajectory_len_;

        while (rclcpp::ok() && t <= total_time_)
        {
            if (goal_handle->is_canceling())
            {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            p = (traj_type_ == "linear") ? ((s_type_ == "trapezoidal") ? planner.linear_traj_trapezoidal(t) : planner.linear_traj_cubic(t))
                                         : ((s_type_ == "trapezoidal") ? planner.circular_traj_trapezoidal(t) : planner.circular_traj_cubic(t));

            robot_->setJntState(joint_positions_);
            KDL::Frame cartpos_current = robot_->getEEFrame();
            Eigen::Vector3d current_EE_pos(cartpos_current.p.data[0], cartpos_current.p.data[1], cartpos_current.p.data[2]);
            Eigen::Vector3d error = computeLinearError(p.pos, current_EE_pos);

            feedback->position_error.x = error[0];
            feedback->position_error.y = error[1];
            feedback->position_error.z = error[2];
            goal_handle->publish_feedback(feedback);

            Eigen::MatrixXd J = robot_->getEEJacobian().data;
            Eigen::MatrixXd J_pinv = pseudoinverse(J);
            Eigen::VectorXd q_dot;
            KDLController controller(*robot_);

            if (ctrl_mode_ == "velocity_ctrl")
            {
                Eigen::VectorXd cartvel(6);
                cartvel.setZero();
                cartvel.head<3>() = -Kp_ * error;
                double alpha = std::min(1.0, t / 1.0);
                cartvel *= alpha;
                q_dot = J_pinv * cartvel;
            }
            else if (ctrl_mode_ == "velocity_ctrl_null")
            {
                Eigen::VectorXd ep(6);
                ep.setZero();
                ep.head<3>() = -Kp_ * error;
                double alpha = std::min(1.0, t / 1.0);
                ep *= alpha;

                Eigen::VectorXd q0 = compute_q0(joint_positions_);
                Eigen::MatrixXd I = Eigen::MatrixXd::Identity(J.cols(), J.cols());

                q_dot = J_pinv * ep + (I - J_pinv * J) * q0;
            }

            else // VISION_CTRL aggiornato
            {
                if (!aruco_pose_available_)
                {
                    q_dot = Eigen::VectorXd::Zero(robot_->getNrJnts());
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for ArUco pose...");
                }
                else
                {
                    // --- Correzione della chiamata a vision_ctrl ---
                    q_dot = controller.vision_ctrl(
                        cPo_,  // Eigen::Vector3d cPo
                        J,     // Eigen::MatrixXd J (6xN)
                        Rc_kdl_, // KDL::Rotation Rc
                        Kp_,   // double Kp (guadagno primario)
                        lambda_, // double Kp_null (usato come lambda nel JLA)
                        joint_positions_.data,  // Eigen::VectorXd q_curr
                        q_min_.data, // Eigen::VectorXd q_min
                        q_max_.data  // Eigen::VectorXd q_max
                    );
                    // --- Fine Correzione ---
                }
            }

            Eigen::VectorXd q_current = joint_positions_.data;
            Eigen::VectorXd q_cmd = q_current + q_dot * dt;

            Eigen::VectorXd q_dot_num = (q_cmd - joint_positions_prev_.data) / dt;
            joint_positions_prev_.data = q_cmd;
            joint_positions_cmd_.data = q_cmd;
            joint_positions_.data = q_cmd;  

            FloatArray cmd_msg;
            cmd_msg.data.resize(joint_positions_cmd_.rows());
            for (int i = 0; i < joint_positions_cmd_.rows(); ++i)
                cmd_msg.data[i] = joint_positions_cmd_(i);
            cmdPublisher_->publish(cmd_msg);

            if (csv_file_.is_open())
            {
                for (int i = 0; i < joint_positions_cmd_.rows(); ++i)
                    csv_file_ << joint_positions_cmd_(i) << ",";
                for (int i = 0; i < joint_velocities_.rows(); ++i)
                    csv_file_ << q_dot_num(i) << (i != joint_velocities_.rows() - 1 ? "," : "");
                csv_file_ << std::endl;
            }

            t += dt;
            std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        }

        if (rclcpp::ok() && !goal_handle->is_canceling())
        {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Trajectory execution finished");
        }
    }

    // ================== Timer originale (per auto_start o vision_ctrl) ==================
    void cmd_publisher()
    {
        if (!joint_state_available_)
            return;

        double dt = total_time_ / trajectory_len_;
        t_ += dt;
        
        if ((ctrl_mode_ == "velocity_ctrl" || ctrl_mode_ == "velocity_ctrl_null") && t_ > total_time_)
        {
            RCLCPP_INFO(this->get_logger(), "Trajectory finished");
            timer_->cancel();
            if (csv_file_.is_open()) csv_file_.close();
            return;
        }

        static bool first_call = true;
        if (first_call)
        {
            robot_->setJntState(joint_positions_);
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            KDL::Frame cartpos = robot_->getEEFrame();
            Eigen::Vector3d current_EE(cartpos.p.data[0], cartpos.p.data[1], cartpos.p.data[2]);
            Eigen::Vector3d end_EE(end_position_[0], end_position_[1], end_position_[2]);
            planner_ = KDLPlanner(traj_duration_, acc_duration_, current_EE, end_EE);
            p_.pos = current_EE;
            joint_positions_prev_ = joint_positions_;
            first_call = false;
        }

        if (ctrl_mode_ == "velocity_ctrl" || ctrl_mode_ == "velocity_ctrl_null")
        {
             if (traj_type_ == "linear")
                 p_ = (s_type_ == "trapezoidal") ? planner_.linear_traj_trapezoidal(t_) : planner_.linear_traj_cubic(t_);
             else
                 p_ = (s_type_ == "trapezoidal") ? planner_.circular_traj_trapezoidal(t_) : planner_.circular_traj_cubic(t_);
        }
        
        robot_->setJntState(joint_positions_);
        KDL::Frame cartpos = robot_->getEEFrame();
        Eigen::Vector3d current_EE(cartpos.p.data[0], cartpos.p.data[1], cartpos.p.data[2]);
        Eigen::Vector3d error = computeLinearError(p_.pos, current_EE); 

        Eigen::MatrixXd J = robot_->getEEJacobian().data; 
        Eigen::MatrixXd J_pinv = pseudoinverse(J);
        Eigen::VectorXd q_dot;
        
        KDLController controller(*robot_); 

        if (ctrl_mode_ == "velocity_ctrl")
        {
            Eigen::VectorXd cartvel(6);
            cartvel.setZero();
            cartvel.head<3>() = -Kp_ * error;
            double alpha = std::min(1.0, t_ / 1.0);
            cartvel *= alpha;
            q_dot = J_pinv * cartvel;
        }
        
        else if (ctrl_mode_ == "velocity_ctrl_null")
        {
           Eigen::VectorXd ep(6);
           ep.setZero();
           ep.head<3>() = -Kp_ * error;
           double alpha = std::min(1.0, t_ / 1.0);
           ep *= alpha;

           Eigen::VectorXd q0 = compute_q0(joint_positions_);
           Eigen::MatrixXd I = Eigen::MatrixXd::Identity(J.cols(), J.cols());

           q_dot = J_pinv * ep + (I - J_pinv * J) * q0;
        }

        else if (ctrl_mode_ == "vision_ctrl")
        {
            if (!aruco_pose_available_)
            {
                q_dot = Eigen::VectorXd::Zero(robot_->getNrJnts());
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for ArUco pose...");
            }
            else
            {
                // --- Correzione della chiamata a vision_ctrl ---
                q_dot = controller.vision_ctrl(
                    cPo_,  // Eigen::Vector3d cPo
                    J,     // Eigen::MatrixXd J (6xN)
                    Rc_kdl_, // KDL::Rotation Rc
                    Kp_,   // double Kp (guadagno primario)
                    lambda_, // double Kp_null (usato come lambda nel JLA)
                    joint_positions_.data,  // Eigen::VectorXd q_curr
                    q_min_.data, // Eigen::VectorXd q_min
                    q_max_.data  // Eigen::VectorXd q_max
                );
                // --- Fine Correzione ---
            }
        }
        else
        {
            q_dot = Eigen::VectorXd::Zero(robot_->getNrJnts());
            RCLCPP_ERROR(this->get_logger(), "Controller mode non valido!");
        }

        Eigen::VectorXd q_current = joint_positions_.data;
        Eigen::VectorXd q_cmd = q_current + q_dot * dt;
        Eigen::VectorXd q_dot_num = (q_cmd - joint_positions_prev_.data) / dt;
        joint_positions_prev_.data = q_cmd;
        joint_positions_cmd_.data = q_cmd;
        joint_positions_.data = q_cmd;  

        if (!cmdPublisher_)
        {
            cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
        }

        FloatArray cmd_msg;
        cmd_msg.data.resize(joint_positions_cmd_.rows());
        for (int i = 0; i < joint_positions_cmd_.rows(); ++i)
            cmd_msg.data[i] = joint_positions_cmd_(i);
        cmdPublisher_->publish(cmd_msg);

        if (csv_file_.is_open())
        {
            for (int i = 0; i < joint_positions_cmd_.rows(); ++i)
                csv_file_ << joint_positions_cmd_(i) << ",";
            for (int i = 0; i < joint_velocities_.rows(); ++i)
                csv_file_ << q_dot_num(i) << (i != joint_velocities_.rows() - 1 ? "," : "");
            csv_file_ << std::endl;
        }
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState &msg)
    {
        if (msg.position.empty())
            return;

        int n = std::min((int)joint_positions_.rows(), (int)msg.position.size());
        for (int i = 0; i < n; ++i)
        {
            joint_positions_(i) = msg.position[i];
            if (i < (int)msg.velocity.size())
                joint_velocities_(i) = msg.velocity[i];
        }

        if (!joint_state_available_)
        {
            joint_state_available_ = true;
            RCLCPP_INFO(this->get_logger(), "First joint_states received!");

            if (auto_start_ && (ctrl_mode_ == "velocity_ctrl" || ctrl_mode_ == "velocity_ctrl_null" || ctrl_mode_ == "vision_ctrl"))
            {
                RCLCPP_INFO(this->get_logger(), "Starting autonomous control timer...");
                double dt_seconds = total_time_ / trajectory_len_;
                auto timer_period = std::chrono::duration<double>(dt_seconds);
                timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(timer_period), 
                                                std::bind(&Iiwa_pub_sub_action::cmd_publisher, this));
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Node in Action Server mode: waiting for client goal...");
            }
        }
    }

    void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped &msg)
    {
        cPo_ << msg.pose.position.x, 
              msg.pose.position.y, 
              msg.pose.position.z;

        Rc_kdl_ = KDL::Rotation::Quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        );
        
        if (!aruco_pose_available_)
        {
            aruco_pose_available_ = true;
            RCLCPP_INFO(this->get_logger(), "First ArUco pose received!");
        }
    }

    // Funzione compute_q0 mantenuta qui se non vuoi spostarla (anche se \u00e8 usata solo in velocity_ctrl_null)
    Eigen::VectorXd compute_q0(const KDL::JntArray &q)
    {
        int nj = q.rows();
        Eigen::VectorXd q0 = Eigen::VectorXd::Zero(nj);
        double lambda = lambda_; // Usa lambda_ come Kp_null/lambda nella formula
        for (int i = 0; i < nj; ++i)
        {
            double qi = q(i);
            double qi_plus = q_max_(i);
            double qi_minus = q_min_(i);
            double denom = (qi_plus - qi) * (qi - qi_minus);
            if (std::fabs(denom) < 1e-6) denom = 1e-6; 
            q0(i) = std::pow(qi_plus - qi_minus, 2) * (qi_plus + qi_minus - 2.0 * qi) / (lambda * denom * denom);
        }
        return q0;
    }

    Eigen::Vector3d computeLinearError(const Eigen::Vector3d &target, const Eigen::Vector3d &current)
    {
        return target - current;
    }

    // ================== Membri ==================
    rclcpp_action::Server<LinearTraj>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;
    trajectory_point p_;

    KDL::JntArray joint_positions_, joint_velocities_;
    KDL::JntArray joint_positions_cmd_, joint_velocities_cmd_, joint_efforts_cmd_;
    KDL::JntArray q_min_, q_max_;
    std::vector<double> desired_commands_;

    bool joint_state_available_ = false;
    int iteration_ = 0;
    double t_ = 0.0;

    std::string cmd_interface_, traj_type_, s_type_, ctrl_mode_;
    double traj_duration_, acc_duration_, total_time_, Kp_, Kp_null_, lambda_; 
    int trajectory_len_;
    std::vector<double> end_position_;
    bool auto_start_;

    std::ofstream csv_file_; // CSV output
};

// ================== Main ==================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Iiwa_pub_sub_action>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
