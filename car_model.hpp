#ifndef CAR_MODEL_HPP_
#define CAR_MODEL_HPP_

#include <cmath>
#include <iostream>
#include "common_function.hpp"

class CarModel {
public:
    const double L = 0.1;
    const double d = 0.04;
    const double theta_max = 0.7;
    const double v_max = 0.5;
    const double a_max = 0.5;

    // input 
    double a = 0.0;
    double theta = 0.0;
    double v = 0.0;

    // state
    double x = 0.0;
    double y = 0.0;
    double phi = 0.0;

    CarModel() {}
    void inputControl(double v, double theta);
    void inputPose(double x, double y, double phi);
    void update(double dt);
    void getState(double& x, double& y, double& phi);
    double get_velocity();
    void reset();
};


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







// #include <string>
// #include <iostream>
// #include <cmath>
// #include <vector>

// using namespace std;

// struct CarState
// {
//     double x;
//     double y;
//     double phi;
//     double v;
//     double theta;
// };

// struct CarPos
// {
//     double x;
//     double y;
//     double phi;
// };

// class CarModel
// {
// public:
//     CarModel(double L, double d, double theta_max, double v_max, double a_max) 
//         : L(L), d(d), theta_max(theta_max), v_max(v_max), a_max(a_max) {};
//     CarModel() {};
//     ~CarModel() {};
//     void inputControl(double velocity, double theta);
//     void inputPose(double x, double y, double phi);
//     void updateState(double dt);
//     /**
//      * @param sim_t: total simulation time
//      * @param K: simulation step
//      * @param velocity: velocity
//      * @param theta: steering angle
//      * @param x: car x position
//      * @param y: car y position
//      * @param phi: car phi angle
//      * @return car: position
//      */
//     vector<CarPos> car_simulation(double sim_t, double K, double velocity, double theta, double x, double y, double phi);
//     CarState getState() { return car_state; };
//     vector<CarPos> getTrajectory() {return sim_trajecotry; };
//     vector<CarPos> sim_trajecotry;
// private:
//     // car model parameters
//     double L           = 0.1;
//     double d           = 0.04;
//     double theta_max   = 0.7;
//     double v_max       = 0.5;
//     double a_max       = 0.5;

//     // control inputs
//     double a           = 0.0;  // unuse for now
//     double v           = 0.0;  // velocity
//     double theta       = 0.0;  // steering angle
//     // car states
//     CarState car_state; 
// };

#endif // CAR_MODEL_HPP_