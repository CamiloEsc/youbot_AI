/* *****************************************************************
 *
 * luh_youbot_controller
 *
 * Copyright (c) 2015,
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 * Author: Nikolaj Ponomarjow
 * Modified by: Simon Aden (info@luhbots.de)
 ******************************************************************/

#include "luh_youbot_controller/torque_controller/torque_controller.hpp"

using namespace luh_youbot_kinematics;

nAxesControllerTorque::nAxesControllerTorque(ros::NodeHandle &node):
    node_(&node)
{
    this->ta = 0.002;
    this->d_ta = 500;

    this->m_max.resize(5);
    this->m_max[0] = 9.5; // original Angabe 9.5 (9,5 Nm -> 1,818 A sind 32 % von Imax = 5,768 A)
    this->m_max[1] = 9.5; // original Angabe 9.5 (9,5 Nm -> 1,818 A sind 32 % von Imax = 5,768 A)
    this->m_max[2] = 6.0; // original Angabe 6.0 (6,0 Nm -> 1,791 A sind 46 % von Imax = 3,899 A) 4,5 Nm -> 34%
    this->m_max[3] = 5.0; // original Angabe 2.0 (5,0 Nm -> 1,381 A sind 50 % von Imax = 2,746 A)
    this->m_max[4] = 2.0; // original Angabe 1.0 (2,0 Nm -> 0,575 A sind 33 % von Imax = 1,75 A)

    this->k_p.resize(5);
    this->k_d.resize(5);
    this->k_i.resize(5);
    this->e_sum.resize(5);
    this->e_max.resize(5);
    this->e_max[0] = 0.7;
    this->e_max[1] = 0.6;
    this->e_max[2] = 0.5;
    this->e_max[3] = 0.2;
    this->e_max[4] = 0.1;

    this->k_feed = 0.0;
    this->use_computed_torque_mpc_ = false;
    this->mpc_horizon_steps_ = 15;

    this->k_f_n_1.resize(5);
    this->k_f_n_2.resize(5);
    this->k_f_n_3.resize(5);
    this->k_fr_n_1.resize(5);
    this->k_fr_n_2.resize(5);
    this->k_fr_n_3.resize(5);
    this->mpc_q_pos_.assign(5, 40.0);
    this->mpc_q_vel_.assign(5, 3.0);
    this->mpc_r_torque_.assign(5, 0.2);
    this->mpc_terminal_q_pos_.assign(5, 80.0);
    this->mpc_terminal_q_vel_.assign(5, 6.0);
    this->observer_gain_.assign(5, 0.06);
    this->disturbance_estimate_.assign(5, 0.0);
    this->previous_torque_command_.assign(5, 0.0);

    if(!loadParameters())
        ROS_ERROR("Failed to load torque controller parameters.");
}
nAxesControllerTorque::~nAxesControllerTorque() {

}

bool nAxesControllerTorque::loadParameters()
{
    if(!node_->getParam("torque_controller/k_p", k_p))
        return false;

    if(!node_->getParam("torque_controller/k_i", k_i))
        return false;

    if(!node_->getParam("torque_controller/k_d", k_d))
        return false;

    if(!node_->getParam("torque_controller/k_feed", k_feed))
        return false;

    if(!node_->getParam("torque_controller/k_f_n_1", k_f_n_1))
        return false;

    if(!node_->getParam("torque_controller/k_f_n_2", k_f_n_2))
        return false;

    if(!node_->getParam("torque_controller/k_f_n_3", k_f_n_3))
        return false;

    if(!node_->getParam("torque_controller/k_fr_n_1", k_fr_n_1))
        return false;

    if(!node_->getParam("torque_controller/k_fr_n_2", k_fr_n_2))
        return false;

    if(!node_->getParam("torque_controller/k_fr_n_3", k_fr_n_3))
        return false;

    node_->param("torque_controller/use_computed_torque_mpc", use_computed_torque_mpc_, false);
    node_->param("torque_controller/mpc_horizon_steps", mpc_horizon_steps_, 15);

    std::vector<double> tmp_vector;
    if(node_->getParam("torque_controller/mpc_q_pos", tmp_vector) && tmp_vector.size() == 5)
        mpc_q_pos_ = tmp_vector;

    if(node_->getParam("torque_controller/mpc_q_vel", tmp_vector) && tmp_vector.size() == 5)
        mpc_q_vel_ = tmp_vector;

    if(node_->getParam("torque_controller/mpc_r_torque", tmp_vector) && tmp_vector.size() == 5)
        mpc_r_torque_ = tmp_vector;

    if(node_->getParam("torque_controller/mpc_terminal_q_pos", tmp_vector) && tmp_vector.size() == 5)
        mpc_terminal_q_pos_ = tmp_vector;

    if(node_->getParam("torque_controller/mpc_terminal_q_vel", tmp_vector) && tmp_vector.size() == 5)
        mpc_terminal_q_vel_ = tmp_vector;

    if(node_->getParam("torque_controller/observer_gain", tmp_vector) && tmp_vector.size() == 5)
        observer_gain_ = tmp_vector;

    return true;
}

void nAxesControllerTorque::reset()
{
    for(unsigned int i = 0; i < 5; i++)
    {
        e_sum[i] = 0.0;
        disturbance_estimate_[i] = 0.0;
        previous_torque_command_[i] = 0.0;
    }
}

double nAxesControllerTorque::computeMpcAcceleration(double position_error,
                                                     double velocity_error,
                                                     unsigned int joint) const
{
    // discrete double integrator: e_{k+1} = A e_k + B u_k
    double x1 = position_error;
    double x2 = velocity_error;

    // one-step Riccati recursion for scalar-input decoupled joints
    // gives a finite horizon linear-quadratic MPC feedback u = -Kx.
    double p11 = mpc_terminal_q_pos_[joint];
    double p12 = 0.0;
    double p22 = mpc_terminal_q_vel_[joint];

    const double dt = ta;
    const double q1 = mpc_q_pos_[joint];
    const double q2 = mpc_q_vel_[joint];
    const double r = mpc_r_torque_[joint];

    double k1 = 0.0;
    double k2 = 0.0;

    for(int step = 0; step < mpc_horizon_steps_; ++step)
    {
        const double b1 = 0.5 * dt * dt;
        const double b2 = dt;

        const double bt_p_b = b1 * (p11 * b1 + p12 * b2) + b2 * (p12 * b1 + p22 * b2);
        const double denom = r + bt_p_b;

        const double bt_p_a11 = b1 * p11 + b2 * p12;
        const double bt_p_a12 = b1 * (p11 * dt + p12) + b2 * (p12 * dt + p22);

        k1 = bt_p_a11 / denom;
        k2 = bt_p_a12 / denom;

        const double a11 = 1.0;
        const double a12 = dt;
        const double a21 = 0.0;
        const double a22 = 1.0;

        const double m11 = a11 - b1 * k1;
        const double m12 = a12 - b1 * k2;
        const double m21 = a21 - b2 * k1;
        const double m22 = a22 - b2 * k2;

        const double n11 = p11 * m11 + p12 * m21;
        const double n12 = p11 * m12 + p12 * m22;
        const double n21 = p12 * m11 + p22 * m21;
        const double n22 = p12 * m12 + p22 * m22;

        p11 = q1 + m11 * n11 + m21 * n21;
        p12 =      m11 * n12 + m21 * n22;
        p22 = q2 + m12 * n12 + m22 * n22;
    }

    return -(k1 * x1 + k2 * x2);
}

JointVector nAxesControllerTorque::getTorques(const JointPosition &position_command,
                                              const JointVelocity &velocity_command,
                                              const JointPosition &current_position,
                                              const JointVelocity &current_velocity,
                                              const JointVector   &joint_efforts)
{
    JointVector moment;

    for(unsigned int i = 0; i < 5; i++)
    {

        this->e_pos[i] = position_command[i] - current_position[i];
        this->e_vel[i] = velocity_command[i] - current_velocity[i];

        // I-Term
        e_sum[i] = e_pos[i] + e_sum[i];
        // Clipping to e_max
        if(fabs(e_sum[i]) > e_max[i])
        {
            if (e_sum[i] < 0)
                e_sum[i] = -1 * e_max[i];
            else
                e_sum[i] = 1 * e_max[i];
        }

        /* Torque Control */
        //        if(e_pos[i] > 0.2 || e_vel[i] > 0.3){
        if(e_pos[i] > 5.0 || e_vel[i] > 5.0)
        {
            ROS_WARN("error exceeded joint %d  p = %.2f  v = %.2f", i, e_pos[i], e_vel[i]);
            break; // todo: was dann?
        }
        else
        {
            const double mpc_acceleration = computeMpcAcceleration(e_pos[i], e_vel[i], i);
            const double pid_torque = k_p[i] * e_pos[i] + k_i[i] * e_sum[i] * ta + k_d[i] * e_vel[i];

            if(use_computed_torque_mpc_)
                m_joint_torques[i] = pid_torque + mpc_acceleration;
            else
                m_joint_torques[i] = pid_torque;


            /* Reibung */
            //            friction[i] = k_f_n_1[i] * tanh(k_f_n_2[i] * velocity_command[i]) + k_f_n_3[i] * velocity_command[i];

            /* Richtungsabhängige Reibung */
            if(velocity_command[i] > 0.0)
            {
                friction[i] = k_f_n_1[i] * tanh(k_f_n_2[i] * velocity_command[i]) + k_f_n_3[i] * velocity_command[i];
            }
            else
            {
                friction[i] = k_fr_n_1[i] * tanh(k_fr_n_2[i] * velocity_command[i]) + k_fr_n_3[i] * velocity_command[i];
            }
            /* Richtungsabhängige Reibung */

            reibvor[i] = k_feed * (joint_efforts[i] + friction[i]);

            const double nominal_torque = m_joint_torques[i] + reibvor[i];
            const double residual = previous_torque_command_[i] - nominal_torque;
            disturbance_estimate_[i] += observer_gain_[i] * (residual - disturbance_estimate_[i]);

            moment[i] = nominal_torque + disturbance_estimate_[i];

            //            moment[i] = m_joint_torques[i] + k_feed * m_joint_efforts[i] + friction[i];

            if(fabs(moment[i]) > m_max[i])
            {
                if (moment[i] < 0)
                    moment[i] = -1 * m_max[i];
                else
                    moment[i] = 1 * m_max[i];
            }

            previous_torque_command_[i] = moment[i];
        }

    }

    return moment;
}
