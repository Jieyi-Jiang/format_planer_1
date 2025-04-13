#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "common_function.hpp"
#include "fuzzy_logic.hpp"
#include "car_model.hpp"

using namespace Eigen;

class BoundedDWA {
public:
    BoundedDWA() {
        L = 0.1;
        d = 0.04;
        car = new CarModel();
        sim_t = 0.1;
        v_max = 0.2;
        v_min = -0.3;
        a_max = 1.5;
        a_min = -1.5;
        theta_max = 0.7;
        theta_min = -0.7;
        a_theta_max = 2.0;
        a_theta_min = -2.0;
        sample_axis = 9;
        sim_step_num = 5;
        fiction_factor = 0.3;
        gravity_acc = 9.8;
        v_now = 0.0;
        theta_now = 0.0;
        target_pos = {0, 0, 0};
        pos_car = {0, 0, 0};
        car_xy = {0, 0};
        car_phi = 0;
        pos_goal = {0, 0, 0};
        goal_xy = {0, 0};
        goal_phi = 0;

        sample_space = \
            std::vector<std::vector<std::vector<double>>> \
            ( sample_axis, std::vector<std::vector<double>>( sample_axis, std::vector<double>(3, 0) ) );
        w_heading   = 1.0;
        w_dis_obs   = 1.0;
        w_velocity  = 1.0;
        w_direction = 1.0;
        w_dis_vert  = 1.0;
        w_dis_goal  = 1.0;
        set_weight_gain(1.0 , 1.0, 1.0, 1.0, 1.0, 1.0);
    }

    ~BoundedDWA() {
        delete car;
    }

    void set_sim_t(double sim_t, double run_time) {
        this->sim_t = sim_t;
        this->run_time = run_time;
    }

    void set_sample_axis(int sample_axis) {
        this->sample_axis = sample_axis;
        sample_space = std::vector<std::vector<std::vector<double>>>(sample_axis, std::vector<std::vector<double>>(sample_axis, std::vector<double>(3, 0)));
    }

    void set_sim_step_num(int sim_step_num) {
        this->sim_step_num = sim_step_num;
    }
    // int get_max_index(std::vector<double> list) {
        
    // }
    std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, double, double>> get_all_trajectory() {
        return trajectory_list;
    }

    std::vector<std::vector<double>> get_all_cost() {
        return {cost_list, cost_heading_list, cost_dis_obs_list, cost_velo_list, cost_direction_list, cost_dis_vert_list};
    }

    void update_weight(double w_heading, double w_dis_obs, double w_velocity, double w_direction, double w_dis_vert) {
        this->w_heading = w_heading;
        this->w_dis_obs = w_dis_obs;
        this->w_velocity = w_velocity;
        this->w_direction = w_direction;
        this->w_dis_vert = w_dis_vert;
    }

    void cal_weight() {
        // printf("cal_weight\n");
        FuzzySystem fs;
        double d_goal = std::sqrt(std::pow(goal_xy[0] - car_xy[0], 2) + std::pow(goal_xy[1] - car_xy[1], 2));
        // double d_phi = std::abs(car_phi - goal_phi);
        vector<double> vect_car_phi = {std::cos(car_phi), std::sin(car_phi)};
        vector<double> vect_goal_phi = {std::cos(goal_phi), std::sin(goal_phi)};
        double d_phi = cal_angle_of_vect_rad(vect_car_phi, vect_goal_phi);
        double d_vert = cal_distance_vertical(pos_car);
        double obstacle = 100;
        // auto [w1, w2, w3, w4, w5] = fs.inference(d_goal, d_phi);
        auto [w1, w2, w3, w4, w5] = fs.inference(d_goal, d_phi, d_vert);
        cout << "d_vert: " << d_vert << endl;
        double w_heading    = w1   * wg_heading;
        double w_dis_obs    = 1.0  * wg_dis_obs;
        double w_velocity   = w2   * wg_velocity;
        double w_direction  = w3   * wg_direction;
        double w_dis_vert   = w4   * wg_dis_vert;
        double w_dis_goal   = w5   * wg_dis_goal;
        // std::cout << "fuzzy_output: " << fuzzy_output << std::endl;
        // double w_heading    = 1.0 * wg_heading;
        // double w_dis_obs    = 1.0 * wg_dis_obs;
        // double w_velocity   = 1.0 * wg_velocity;
        // double w_direction  = 1.0 * wg_direction;
        // // double w_dis_vert   = w_direction * fuzzy_output * wg_dis_vert;
        // double w_dis_vert   = w_direction * fuzzy_output * fuzzy_output * wg_dis_vert;
        // double w_dis_goal   = 1.0 * wg_dis_goal;
        // std::cout << "fuzzy_output: " << fuzzy_output << std::endl;
        cout << "fuzzy out | " << w1 << " " << w2 << " " << w3 << " " << w4 << " " << w5 << endl;
        std::cout << "w_heading: " << w_heading << ", w_dis_obs: " << w_dis_obs << ", w_velocity: " << w_velocity << ", w_direction: " << w_direction << ", w_dis_vert: " << w_dis_vert << ", w_dis_goal: " << w_dis_goal << std::endl;
        // update_weight(w_heading, w_dis_obs, w_velocity, w_direction, w_dis_vert);
        printf("cal_weight set weight\n");
        this->w_heading = w_heading;
        this->w_dis_obs = w_dis_obs;
        this->w_velocity = w_velocity;
        this->w_direction = w_direction;
        this->w_dis_vert = w_dis_vert;
        this->w_dis_goal = w_dis_goal;
    }

    std::vector<double> cost_heading() {
        std::vector<double> temp_list;
        // cout << "1 " << endl;
        for (int i = 0; i < trajectory_list.size(); ++i) {
            auto [x_list, y_list, phi_list, v_samp, theta_samp] = trajectory_list[i];
            double x = x_list.back();
            double y = y_list.back();
            double phi = phi_list.back();
            if (v_samp == 0) {
                v_samp = 1e-20;
            }
            double v_direction = v_samp / std::abs(v_samp);
            std::vector<double> vec_car2goal = {goal_xy[0] - x, goal_xy[1] - y};
            std::vector<double> vec_car = {std::cos(phi) * v_direction, std::sin(phi) * v_direction};
            double silm = cal_similarity_of_vect(vec_car, vec_car2goal);
            // double temp = std::abs(180.0 - angle) / 180.0;
            double temp = silm;
            temp_list.push_back(temp);
        }
        // cout << "2 " << endl;
        // double max_value, index;
        auto [max_value, index] = find_max_and_index(temp_list);
        // cout << "max_value: " << max_value << endl; 
        // cout << "index: " << index << endl;
        // cout << "3 " << endl;
        std::vector<double> np_list(temp_list.size());
        // cout << "4 " << endl;
        for (int i = 0; i < temp_list.size(); ++i) {
            np_list[i] = temp_list[i] / max_value;
        }
        cost_heading_list = np_list;
        // cout << "5 " << endl;
        return cost_heading_list;
    }

    std::vector<double> cost_dis_obs() {
        std::vector<double> temp_list(trajectory_list.size(), 0.0);
        cost_dis_obs_list = temp_list;
        return cost_dis_obs_list;
    }

    std::vector<double> cost_velocity() {
        std::vector<double> temp_list;
        for (int i = 0; i < trajectory_list.size(); ++i) {
            auto [x_list, y_list, phi_list, v_samp, theta_samp] = trajectory_list[i];
            double temp = (temp >= 0) ? std::abs(v_samp) : std::abs(v_samp) / 3.0;
            temp_list.push_back(temp);
        }
        double max_value, index;
        std::tie(max_value, index) = find_max_and_index(temp_list);
        std::vector<double> np_list(temp_list.size());
        for (int i = 0; i < temp_list.size(); ++i) {
            np_list[i] = temp_list[i] / max_value;
        }
        cost_velo_list = np_list;
        return cost_velo_list;
    }

    std::vector<double> cost_direction() {
        std::vector<double> temp_list;
        double temp;
        for (int i = 0; i < trajectory_list.size(); ++i) {
            auto [x_list, y_list, phi_list, v_samp, theta_samp] = trajectory_list[i];
            double phi_predicted = phi_list.back();
            double phi_goal = goal_phi;
            vector<double> phi_predicted_vec = {std::cos(phi_predicted), std::sin(phi_predicted)};
            vector<double> phi_goal_vec = {std::cos(phi_goal), std::sin(phi_goal)};
            // double temp = 1 - std::abs(phi_predicted - phi_goal) / M_PI;
            temp = cal_similarity_of_vect(phi_predicted_vec, phi_goal_vec);
            temp_list.push_back(temp);
        }
        double max_value, index;
        std::tie(max_value, index) = find_max_and_index(temp_list);
        std::vector<double> np_list(temp_list.size());
        for (int i = 0; i < temp_list.size(); ++i) {
            np_list[i] = temp_list[i] / max_value;
        }
        cost_direction_list = np_list;
        return cost_direction_list;
    }

    std::tuple<double, double, double> cal_vect2line(double x, double y, double phi) {
        double x1 = x, y1 = y;
        double x2 = x + std::cos(phi), y2 = y + std::sin(phi);
        double k = (x2 - x1 == 0.0) ? 100000000000000000.0 : (y2 - y1) / (x2 - x1);
        double b = y1 - k * x1;
        double A = k, B = -1, C = b;
        return std::make_tuple(A, B, C);
    }

    double cal_distance_vertical(std::vector<double> pos_predicted) {
        double pre_x = pos_predicted[0];
        double pre_y = pos_predicted[1];
        double pre_phi = pos_predicted[2];
        double A, B, C;
        std::tie(A, B, C) = cal_vect2line(goal_xy[0], goal_xy[1], goal_phi);
        double temp = std::sqrt(A * A + B * B);
        if (temp == 0.0) {
            temp = 1e-20;
        }
        double dis = std::abs(A * pre_x + B * pre_y + C) / temp;
        return dis;
    }

    std::vector<double> cal_all_distance_vertical() {
        std::vector<double> dis_vert_list;
        for (int i = 0; i < trajectory_list.size(); ++i) {
            auto [x_list, y_list, phi_list, v_samp, theta_samp] = trajectory_list[i];
            std::vector<double> pos_predicted = {x_list.back(), y_list.back(), phi_list.back()};
            double dis_vert = cal_distance_vertical(pos_predicted);
            dis_vert_list.push_back(dis_vert);
        }
        return dis_vert_list;
    }

    std::vector<double> cost_dis_vert() {
        std::vector<double> dis_vert_list = cal_all_distance_vertical();
        double max_dis_vert, index;
        std::tie(max_dis_vert, index) = find_max_and_index(dis_vert_list);
        std::vector<double> temp_list(dis_vert_list.size());
        for (int i = 0; i < dis_vert_list.size(); ++i) {
            temp_list[i] = 1 - dis_vert_list[i] / max_dis_vert;
        }
        double max_value, index2;
        std::tie(max_value, index2) = find_max_and_index(temp_list);
        std::vector<double> np_list(temp_list.size());
        for (int i = 0; i < temp_list.size(); ++i) {
            np_list[i] = temp_list[i] / max_value;
        }
        cost_dis_vert_list = np_list;
        return cost_dis_vert_list;
    }

    std::vector<double> cost_dis_goal() {
        std::vector<double> temp_list;
        for (int i = 0; i < trajectory_list.size(); ++i) {
            auto [x_list, y_list, phi_list, v_samp, theta_samp] = trajectory_list[i];
            std::vector<double> pos_predicted = {x_list.back(), y_list.back(), phi_list.back()};
            Vector2d goal(goal_xy[0], goal_xy[1]);
            Vector2d pos(pos_predicted[0], pos_predicted[1]);
            double dis_goal = (pos - goal).norm();
            temp_list.push_back(dis_goal);

        }
        double max_value, index, min_value, index2;
        std::tie(max_value, index) = find_max_and_index(temp_list);
        // std::tie(min_value, index2) = find_min_and_index(temp_list);
        std::vector<double> np_list(temp_list.size());
        for (int i = 0; i < temp_list.size(); ++i) {
            
            np_list[i] = (max_value - temp_list[i]) / max_value;
        }
        cost_dis_goal_list = np_list;
        return cost_dis_goal_list;
    }

    std::tuple<std::vector<double>, double, int> cost_overall() {
        if (trajectory_list.empty()) {
            // cout << "trajectory_list empty" << endl;
            // return temp_list;
        }
        else
        {
            // cout << "trajectory_list size: " << trajectory_list.size() << endl;
        }
        // cout << "cost_heading" << endl;
        cost_heading();
        // cout << "cost_dis_obs" << endl;
        cost_dis_obs();
        // cout << "cost_velocity" << endl;
        cost_velocity();
        // cout << "cost_direction" << endl;
        cost_direction();
        // cout << "cost_dis_vert" << endl;
        cost_dis_vert();
        cost_dis_goal();
        std::vector<double> temp_list;
        // cout << "cost: ";
        for (int i = 0; i < trajectory_list.size(); ++i) {
            double temp = cost_heading_list[i]      * w_heading     +
                          cost_dis_obs_list[i]      * w_dis_obs     +
                          cost_velo_list[i]         * w_velocity    +
                          cost_direction_list[i]    * w_direction   +
                          cost_dis_vert_list[i]     * w_dis_vert    +
                          cost_dis_goal_list[i]     * w_dis_goal;
            // cout << temp << " ";
            temp_list.push_back(temp);
        }
        // cout << endl;
        cost_list = temp_list;
        double max_cost, index;
        std::tie(max_cost, index) = find_max_and_index(temp_list);
        best_trajectory_index = index;
        cout << "max_index: " << index << endl;
        cout << "max_cost: "    << max_cost                                     \
        << ", heading: "        << cost_heading_list[index]     * w_heading     \
        << ", dis_obs: "        << cost_dis_obs_list[index]     * w_dis_obs     \
         << ", velocity: "      << cost_velo_list[index]        * w_velocity    \
         << ", direction: "     << cost_direction_list[index]   * w_direction   \
         << ", dis_vert: "      << cost_dis_vert_list[index]    * w_dis_vert    \
         << ", dis_goal: "      << cost_dis_goal_list[index]    * w_dis_goal    << endl;
        return std::make_tuple(cost_list, max_cost, index);
    }

    void cal_all_trajectory() {
        trajectory_list.clear();
        for (int i = 0; i < sample_axis; ++i) {
            for (int j = 0; j < sample_axis; ++j) {
                if (sample_space[i][j][2] == 0) {
                    // cout << "sample_space[i][j][2] == 0" << endl;
                    continue;
                } else {
                    double v_samp = sample_space[i][j][0];
                    double theta_samp = sample_space[i][j][1];
                    auto [x_list, y_list, phi_list] = cal_trajectory(v_samp, theta_samp);
                    // 打印数组长度
                    // cout << "x_list size: " << x_list.size() << endl;
                    // cout << "y_list size: " << y_list.size() << endl;
                    // cout << "phi_list size: " << phi_list.size() << endl;
                    trajectory_list.push_back(std::make_tuple(x_list, y_list, phi_list, v_samp, theta_samp));
                }
            }
        }
    }

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> cal_trajectory(double velocity, double theta) {
        std::vector<double> x_list, y_list, phi_list;
        double sim_interval = sim_t / sim_step_num;
        car->inputControl(velocity, theta);
        car->inputPose(pos_car[0], pos_car[1], pos_car[2]);
        for (int k = 0; k < sim_step_num; ++k) {
            car->update(sim_interval);
            double car_x, car_y, car_phi;
            car->getState(car_x, car_y, car_phi);
            x_list.push_back(car_x);
            y_list.push_back(car_y);
            phi_list.push_back(car_phi);
        }
        return std::make_tuple(x_list, y_list, phi_list);
    }

    std::vector<std::vector<std::vector<double>>> get_sample_space() {
        return sample_space;
    }

    void cal_sample_space(double v_now, double theta_now) {
        this->v_now = v_now;
        this->theta_now = theta_now;
        double v_window_max = v_now + a_max * this->run_time;
        double v_window_min = v_now + a_min * this->run_time;
        cout << "v_window_min: " << v_window_min << ", v_window_max: " << v_window_max << endl;
        // double theta_window_min = theta_max;
        // double theta_window_max = theta_min;
        double theta_window_min = theta_now + a_theta_max * this->run_time;
        double theta_window_max = theta_now + a_theta_min * this->run_time;
        cout << "theta_window_min: " << theta_window_min << ", theta_window_max: " << theta_window_max << endl;
        double d_v = (v_window_max - v_window_min) / (sample_axis - 1);
        double d_theta = (theta_window_max - theta_window_min) / (sample_axis - 1);
        std::vector<double> v_sample_space(sample_axis), theta_sample_space(sample_axis);
        for (int i = 0; i < sample_axis; ++i) {
            v_sample_space[i] = v_window_min + i * d_v;
            theta_sample_space[i] = theta_window_min + i * d_theta;
        }
        for (int i = 0; i < sample_axis; ++i) {
            for (int j = 0; j < sample_axis; ++j) {
                sample_space[i][j] = {v_sample_space[i], theta_sample_space[j], 1};
                // cout << "(" << v_sample_space[i] << "," << theta_sample_space[j] << ") ";
            }
            // cout << endl;
        }
    }

    void sample_space_constraint() {
        for (int i = 0; i < sample_axis; ++i) {
            for (int j = 0; j < sample_axis; ++j) {
                double v = sample_space[i][j][0];
                double theta = sample_space[i][j][1];
                if (theta == 0.0) {
                    theta = 1e-20;
                }
                if (v > v_max) sample_space[i][j][0] = v_max;
                else if (v < v_min) sample_space[i][j][0] = v_min;
                if (theta > theta_max) sample_space[i][j][1] = theta_max;
                else if (theta < theta_min) sample_space[i][j][1] = theta_min;
                if (std::pow(v, 2) > std::abs(fiction_factor * gravity_acc * L / std::tan(theta))) {
                    sample_space[i][j][0] = 0;
                    sample_space[i][j][1] = 0;
                    sample_space[i][j][2] = 0;
                    // cout << "invalid sample: " << "(" << v << "," << theta << ") ";
                }
            }
            // cout << endl;
        }
    }
    void update_control_value()
    {
        control_v = std::get<3>(trajectory_list[best_trajectory_index]);
        control_theta = std::get<4>(trajectory_list[best_trajectory_index]);
        
    }
    std::tuple<double, double> get_control()
    {
        return std::make_tuple(control_v, control_theta);
    }
    std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, double, double>> 
    plan(double vol_now, double theta_now, std::vector<double> pos_car, std::vector<double> pos_goal) {
        this->pos_car = pos_car;
        this->pos_goal = pos_goal;
        car_xy = {pos_car[0], pos_car[1]};
        car_phi = pos_car[2];
        goal_xy = {pos_goal[0], pos_goal[1]};
        goal_phi = pos_goal[2];
        cal_sample_space(vol_now, theta_now);
        sample_space_constraint();
        cal_all_trajectory();
        trajectory_num = trajectory_list.size();
        cal_weight();
        cost_overall();
        update_control_value();
        return trajectory_list;
    }

    void set_weight_gain(double wg_heading, double wg_dis_obs, double wg_velocity, double wg_direction, double wg_dis_vert, double wg_dis_goal) {
        this->wg_heading    = wg_heading;
        this->wg_dis_obs    = wg_dis_obs;
        this->wg_velocity   = wg_velocity;
        this->wg_direction  = wg_direction;
        this->wg_dis_vert   = wg_dis_vert;
        this->wg_dis_goal   = wg_dis_goal;
    }

private:
    double wg_heading, wg_dis_obs, wg_velocity, wg_direction, wg_dis_vert, wg_dis_goal;
    double L, d, theta_max, v_max, a_max, v_min, a_min, theta_min;
    double sim_t, run_time;
    double a_theta_max, a_theta_min;
    double best_trajectory_index;
    double control_v, control_theta;
    CarModel* car;
    int sample_axis, sim_step_num;
    double fiction_factor, gravity_acc;
    double v_now, theta_now;
    std::vector<double> target_pos, pos_car, car_xy;
    double car_phi;
    std::vector<double> pos_goal, goal_xy;
    double goal_phi;
    std::vector<std::vector<std::vector<double>>> sample_space;
    std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, double, double>> trajectory_list;
    int trajectory_num;
    double w_heading, w_dis_obs, w_velocity, w_direction, w_dis_vert, w_dis_goal;
    std::vector<double> cost_list, cost_heading_list, cost_dis_obs_list, cost_velo_list, cost_direction_list, cost_dis_vert_list, cost_dis_goal_list;
};

