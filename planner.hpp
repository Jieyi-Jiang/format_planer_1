#pragma once

#include <cmath>
#include <vector>
#include <string>
#include <array>
#include <tuple>
#include <iostream>
#include <fstream>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "json.hpp"
#include <memory>


#include "mavlink_udp.hpp"
#include "car_planner.hpp"

using namespace std;
using namespace Eigen;

struct RobotPos{
    double x;
    double y;
    double phi;
};

struct RobotControl{
    double velocity;
    double theta;
};

struct RobotVelocity{
    double v_x;
    double v_y;
    double v_z;
    double v_norm;
};

class Planner
{

public:
    Planner()
    {
        config_param();
        config_network("232.10.11.12", "3333", "3333");
        v_robot_ctrl.resize(robot_num);
        for (int i = 0; i < robot_num; i++) {
            v_robot_ctrl[i] = {0, 0};
        }
        v_robot_pos.resize(robot_num);
        for (int i = 0; i < robot_num; i++) {
            v_robot_pos[i] = {0, 0, 0};
        }
        v_robot_velocity.resize(robot_num);
        for (int i = 0; i < robot_num; i++) {
            v_robot_velocity[i] = {0, 0, 0, 0};
        }
        dwa_planners.resize(robot_num);
        v_robot_ctrl_ready.resize(robot_num);
        for (int i = 0; i < robot_num; i++) {
            v_robot_ctrl_ready[i] = false;
        }
        v_robot_ctrl_last.resize(robot_num);
        for (int i = 0; i < robot_num; i++) {
            v_robot_ctrl_last[i] = {0, 0};
        }
        car_planner_init(0);
        
    }
    ~Planner(){
        delete mav_udp;
    }
    // void plan();
    
    void config_param();
    void receive_data();
    void handle_receive(int system_id, const mavlink_message_t* message);
    void send_data();
    void config_network(const string multicast_ip, const string multicast_port, const string bind_port);
    void send_heartbeat();
    void plan(int robot_index);
    void car_planner_init(int robot_index);
    void send_control(int robot_index);
    void set_target_pos(int robot_index, double x, double y, double phi);
    void fomat_plan();
protected:
    MavlinkUDP *mav_udp;
    // int robot_num;
    // int leader_id;
    Matrix3d relation{
        {0, 0, 0},
        {1, 0, 0},
        {1, 0, 0}
    };
    vector<vector<double>> relation_pos ={
        {-1.0, -1.0, 0.0},
        {1.0, -1.0, 0.0},
    };

    Vector3<bool> show_robor;
    double sim_time;
    double run_time;
    vector<double> wieght_gain;
    double tolerance_err_dis = 0.05;
    double tolerance_err_phi = 0.1;
    int robot_num = 0;
    int leader_id = 0;
    bool show_heartbeat_flag = false;
    bool show_gps_flag = false;
    bool show_compass_flag = false;
    bool show_imu_flag = false;
    bool send_heartbeat_flag = false;
    bool send_flag = false;
    vector<RobotPos> v_robot_pos;
    vector<RobotPos> v_target_pos;
    vector<RobotVelocity> v_robot_velocity;
    vector<RobotControl> v_robot_ctrl;
    vector<RobotControl> v_robot_ctrl_last;
    vector<unique_ptr<BoundedDWA>> dwa_planners;
    vector<bool> v_robot_ctrl_ready;
};

