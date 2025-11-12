#include <iostream>
#include <memory>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"

// Importa l'interfaccia dell'Action
#include "ros2_kdl_package/action/linear_traj.hpp"

using LinearTraj = ros2_kdl_package::action::LinearTraj;
using ClientGoalHandleLinearTraj = rclcpp_action::ClientGoalHandle<LinearTraj>;
using namespace std::chrono_literals;

class KDLActionClient : public rclcpp::Node
{
public:
    KDLActionClient() : Node("kdl_action_client")
    {
        this->action_client_ = rclcpp_action::create_client<LinearTraj>(
            this,
            "linear_traj"
        );
        RCLCPP_INFO(this->get_logger(), "KDL Action Client inizializzato.");
    }

    void send_goal(double x, double y, double z, double duration)
    {
        if (!this->action_client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(this->get_logger(), "Action Server 'linear_traj' non disponibile dopo 10 secondi.");
            return;
        }

        auto goal_msg = LinearTraj::Goal();
        goal_msg.target_position.x = x;
        goal_msg.target_position.y = y;
        goal_msg.target_position.z = z;
        goal_msg.duration = duration;

        RCLCPP_INFO(this->get_logger(), "Invio goal a target: (%.3f, %.3f, %.3f) in %.1fs", x, y, z, duration);

        auto send_goal_options = rclcpp_action::Client<LinearTraj>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&KDLActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&KDLActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&KDLActionClient::get_result_callback, this, std::placeholders::_1);

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<LinearTraj>::SharedPtr action_client_;

    void goal_response_callback(const ClientGoalHandleLinearTraj::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal rifiutato dal server!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accettato. Esecuzione in corso...");
        }
    }

    void feedback_callback(
        ClientGoalHandleLinearTraj::SharedPtr,
        const std::shared_ptr<const LinearTraj::Feedback> feedback)
    {
        double error_x = feedback->position_error.x;
        RCLCPP_INFO(this->get_logger(), "Ricevuto Feedback: Errore X = %.6f", error_x);
    }

    void get_result_callback(const ClientGoalHandleLinearTraj::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Azione completata con SUCCESSO!");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Azione CANCELLATA!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Azione ABORTITA!");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Stato risultato sconosciuto!");
            break;
        }

        // Stampa solo il campo realmente presente nel server
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Risultato: Successo = %s", result.result->success ? "TRUE" : "FALSE");
        }

        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<KDLActionClient>();

    // Obiettivo di prova
    double target_x = 0.5;
    double target_y = -0.2;
    double target_z = 0.6;
    double duration = 12.0;

    action_client->send_goal(target_x, target_y, target_z, duration);

    rclcpp::spin(action_client);

    return 0;
}
