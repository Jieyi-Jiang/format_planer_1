#include <cmath>
#include <iostream>
#include "car_model.hpp"
#include "common_function.hpp"

void CarModel::inputControl(double v, double theta)
{
    if (v > this->v_max)
    {
        v = this->v_max;
    }
    else if (v < -this->v_max)
    {
        v = -this->v_max;
    }
    if (theta > this->theta_max)
    {
        theta = this->theta_max;
    }
    else if (theta < -this->theta_max)
    {
        theta = -this->theta_max;
    }
    this->v = v;
    this->theta = theta;
}

void CarModel::inputPose(double x, double y, double phi)
{
    this->x = x;
    this->y = y;
    this->phi = phi;
}

void CarModel::update(double dt)
{
    double temp = -std::tan(this->theta);
    if (temp == 0)
    {
        temp = 1e-20;
    }
    double R = this->L / temp;
    double x_dif = this->v * std::cos(this->phi);
    double y_dif = this->v * std::sin(this->phi);
    double phi_dif = this->v / R * dt;
    this->x += x_dif * dt;
    this->y += y_dif * dt;
    this->phi += phi_dif;
    this->phi = std::fmod(this->phi, 2 * M_PI);
    if (this->phi > M_PI)
    {
        this->phi -= 2 * M_PI;
    }
}

void CarModel::getState(double &x, double &y, double &phi)
{
    x = this->x;
    y = this->y;
    phi = this->phi;
}

double CarModel::get_velocity()
{
    return this->v;
}

void CarModel::reset()
{
    this->x = 0.0;
    this->y = 0.0;
    this->phi = 0.0;
    this->v = 0.0;
    this->theta = 0.0;
    this->a = 0.0;
}

// #include "car_model.hpp"
// #include "common_function.hpp"

// void CarModel::inputControl(double velocity, double theta)
// {
//     double v = threshold(v, v_max, -v_max);
//     double th = threshold(theta, theta_max, -theta_max);
//     this->car_state.v = v;
//     this->car_state.theta = th;
// }

// void CarModel::inputPose(double x, double y, double phi)
// {
//     this->car_state.x = x;
//     this->car_state.y = y;
//     this->car_state.phi = phi;
// }

// void CarModel::updateState(double dt)
// {
//     double tan_theta = 0.0;
//     if (this->car_state.theta != 0.0)
//     {
//         tan_theta = tan(this->car_state.theta);
//     }
//     else
//     {
//         tan_theta = 1e-20;
//     }
//     double R = this->L / tan_theta;
//     double x_dif = this->car_state.v * cos(this->car_state.theta) * dt;
//     double y_dif = this->car_state.v * sin(this->car_state.theta) * dt;
//     double phi_dif = this->car_state.v / R * dt;
//     this->car_state.x += x_dif;
//     this->car_state.y += y_dif;
//     this->car_state.phi += phi_dif;
//     this->car_state.phi = fmod(this->car_state.phi, 2 * PI);
//     if (this->car_state.phi > PI)
//     {
//         this->car_state.phi -= 2 * PI;
//     }
// }

// vector<CarPos> CarModel::car_simulation(double sim_t, double K, double velocity, double theta, double x, double y, double phi)
// {
//     CarPos car_pos_temp;
//     vector<CarPos> car_pos_vec;
//     this->inputControl(velocity, theta);
//     this->inputPose(x, y, phi);
//     double sim_interval = sim_t / K;
//     for (double i = 0; i < K; i += 1)
//     {
//         this->updateState(sim_interval);
//         car_pos_temp.x = this->car_state.x;
//         car_pos_temp.y = this->car_state.y;
//         car_pos_temp.phi = this->car_state.phi;
//         car_pos_vec.push_back(car_pos_temp);
//     }
//     this->sim_trajecotry = move(car_pos_vec);
//     return this->sim_trajecotry;
// }
