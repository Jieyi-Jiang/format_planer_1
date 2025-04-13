#include <fstream>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Core>
#include "json.hpp"

#include "planner.hpp"
#include "common/mavlink.h"
#include "common_function.hpp"

// #define M_PI ((float)asin(1)*2)

// using json = nlohmann::json;
using namespace nlohmann;
using namespace std;
using namespace Eigen;
void test_json()
{
    // // ifstream f("config.json");
    // ifstream test; 
    // test.open("config.json");
    // if (!test.is_open())
    // {
    //     cout << "can not open file" << endl;
    // }
    // char str[500]; 
    // // test >> str;
    // // cout << str << endl;
    // json data = json::parse(test);
    // double value = data["pi"];
    // cout << "data.size(): " << data.size() << endl;
    // cout << value << endl;
}

void Planner::config_param() // 定义Planner类的成员函数config_param，用于配置参数
{
    string path = "D:/Project/Robot/communication/planner/config.json";
    ifstream file(path); // 创建一个输入文件流对象file，并尝试打开名为"config.json"的文件
    if (!file.is_open()) // 检查文件是否成功打开
    {
        cout << "can not open the file" << endl; 
        throw "can not open the file"; 
    }
    json data = json::parse(file); 
    robot_num = data["robot_num"];
    leader_id = data["leader_id"];
    show_robor[0] = data["show_robot"][0]; 
    show_robor[1] = data["show_robot"][1]; 
    show_robor[2] = data["show_robot"][2]; 
    show_heartbeat_flag = data["show_heartbeat"]; 
    show_compass_flag = data["show_compass"]; 
    show_gps_flag = data["show_gps"]; 
    show_imu_flag = data["show_imu"];
    send_heartbeat_flag = data["send_heartbeat"];
    sim_time = data["sim_time"];
    run_time = data["run_time"];
    wieght_gain.resize(6);
    wieght_gain[0] = data["weight_gain"][0];
    wieght_gain[1] = data["weight_gain"][1];
    wieght_gain[2] = data["weight_gain"][2];
    wieght_gain[3] = data["weight_gain"][3];
    wieght_gain[4] = data["weight_gain"][4];
    wieght_gain[5] = data["weight_gain"][5];
    v_target_pos.resize(robot_num);
    for (int i = 0; i < robot_num; i++) {
        v_target_pos[i] = {0, 0, 0};
    }
    set_target_pos(0, data["goal_pos"][0], data["goal_pos"][1], data["goal_pos"][2]);
    printf("show_robor: %d %d %d\n", show_robor[0], show_robor[1], show_robor[2]); // 打印show_robor数组的三个元素
    file.close(); // 关闭文件
}

void normalize_vector(double &x, double &y)
{
    double length = sqrt(x*x + y*y);
    x /= length;
    y /= length;
}

// void update_angle()
double angle_of_vector_2d(double x1, double y1, double x2, double y2)
{
    normalize_vector(x1, y1);
    normalize_vector(x2, y2);
    double cosine = x1*x2 + y1*y2;
    double sine = y2*x1-y1*x2;
    double angle = atan2(sine, cosine)/PI*180;
    return angle;
}
/**
 * force_direction : robot -----> goal
 * calculate : goal - robot
 * 指向被减向量
 */
double gravitation(double pos_robot, double pos_goal, double interest_radius, double k)
{   
    double distance = pos_robot - pos_goal;
    double abs_distance = abs(distance);
    double sign = distance / abs_distance;
    if (abs_distance <= interest_radius)
    {
        return sign * (1/2 * k * pow(abs_distance, 2));
    }
    else
    {
        return sign * (interest_radius * k * abs_distance - 1/2 * pow(interest_radius, 2) * k);
    }
}

/**
 * force_direction : robot <----- barrier
 * calculate : robot - barrier
 */
double repulsion(double pos_robot, double pos_barrier, double interest_radius, double k)
{
    double distance = pos_robot - pos_barrier;
    double abd_distance = abs(distance);
    double sign = distance / abd_distance;
    if (abd_distance <= interest_radius)
    {
        return sign * (1/2 * k * pow((1/distance - 1/interest_radius), 2));
    }
    else
    {
        return 0.0;
    }
}

void Planner::config_network(const string multicast_ip, const string multicast_port, const string bind_port)
{
    mav_udp = new MavlinkUDP(multicast_ip, multicast_port, bind_port);
}


void Planner::handle_receive(int system_id, const mavlink_message_t* message)
{
    int robot_index = system_id - 1;
    Vector3d velocity;
    double phi;
    // cout << system_id << " | ";
    switch(message->msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
            if (show_robor[robot_index] && show_heartbeat_flag)
            {
                cout << system_id << " | get a heartbeat packet" << endl;
            }
            break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            mavlink_local_position_ned_t data1;
            mavlink_msg_local_position_ned_decode(message, &data1);
            v_robot_pos[robot_index].x = data1.x;
            v_robot_pos[robot_index].y = data1.y;
            velocity << data1.vx, data1.vy, data1.vz;
            v_robot_velocity[robot_index].v_x = velocity[0];
            v_robot_velocity[robot_index].v_y = velocity[1];
            v_robot_velocity[robot_index].v_z = velocity[2];
            v_robot_velocity[robot_index].v_norm = velocity.norm();
            // cout << "velocity: x(" << velocity[0] << "), y(" << velocity[1] << "), z(" << velocity[2] << "), norm(" << velocity.norm() << ")" << endl;
            if (show_robor[robot_index] && show_gps_flag)
            {
                printf("%d | 1: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", system_id, data1.x, data1.y, data1.z, data1.vx, data1.vy, data1.vz);
            }
            break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET:
            mavlink_local_position_ned_system_global_offset_t data2;
            mavlink_msg_local_position_ned_system_global_offset_decode(message, &data2);
            phi = data2.yaw;
            v_robot_pos[robot_index].phi = phi;
            cout << "car phi: " << phi << endl;
            if (show_robor[robot_index] && show_compass_flag)
            {
                printf("%d | 2: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", system_id, data2.x, data2.y, data2.z, data2.roll, data2.pitch, data2.yaw);
            }
            break;
        case MAVLINK_MSG_ID_HIGHRES_IMU:
            mavlink_highres_imu_t data3;
            mavlink_msg_highres_imu_decode(message, &data3);
            v_robot_ctrl_last[robot_index].velocity = data3.abs_pressure;
            v_robot_ctrl_last[robot_index].theta = data3.diff_pressure;
            if (show_robor[robot_index] && show_imu_flag)
            {
                printf("%d | 3: acc[%.2f, %.2f, %.2f] gyro[%.2f, %.2f, %.2f] meag[%.2f, %.2f, %.2f]\n", system_id, data3.xacc, data3.yacc, data3.zacc, data3.xgyro, data3.ygyro, data3.zgyro, data3.xmag, data3.ymag, data3.zmag);
            }
            break;
    }
}

void Planner::receive_data()
{
    mavlink_message_t message;
    mavlink_status_t status;
    mav_udp->mavudp_recive_message(MAVLINK_COMM_0, &message, &status);
    switch(message.sysid)
    {
        case 1:
            handle_receive(1, &message);
            break;
        case 2:
            handle_receive(2, &message);
            break;
        case 3:
            handle_receive(3, &message);
            break;
    }
}

void Planner::send_control(int robot_index)
{
    while(!v_robot_ctrl_ready[robot_index]){
        Sleep(10); // 每隔10毫秒检查一次
    };
    cout << "send control" << endl;
    mavlink_local_position_ned_t ctrl;
    mavlink_message_t msg;
    ctrl.x = v_robot_ctrl[robot_index].velocity;
    ctrl.y = v_robot_ctrl[robot_index].theta;
    cout << "ctrl.x: " << ctrl.x << endl;
    cout << "ctrl.y: " << ctrl.y << endl;
    uint8_t sysid = 0;
    uint8_t compid = MAV_COMP_ID_SYSTEM_CONTROL;
    mavlink_msg_local_position_ned_encode(sysid, compid, &msg, &ctrl);
    mav_udp->mavudp_send_message(&msg);
    v_robot_ctrl_ready[robot_index] = false;
}

void Planner::send_data()
{
    send_control(0);
}

void Planner::send_heartbeat()
{
    if (!send_heartbeat_flag) return;
    // for (int i = 1; i < 4; i++)
    // {
        cout << "send heartbeat" << endl;
        uint8_t sysid = 0;
        mavlink_message_t msg;
        mavlink_heartbeat_t heartbeat;
        uint8_t compid = MAV_COMP_ID_SYSTEM_CONTROL;
        heartbeat.autopilot = MAV_AUTOPILOT_GENERIC; 
        heartbeat.type = MAV_TYPE_GCS;  // ground control station
        heartbeat.base_mode = 0;
        heartbeat.custom_mode = 0; 
        heartbeat.system_status = MAV_STATE_ACTIVE; 
        mavlink_msg_heartbeat_encode(sysid, compid, &msg, &heartbeat);
        mav_udp->mavudp_send_message(&msg);
    // }
}

void Planner::car_planner_init(int robot_index)
{
    dwa_planners[0] = std::make_unique<BoundedDWA>();
    dwa_planners[0]->set_sim_t(0.1, 0.05);
    dwa_planners[0]->set_sim_step_num(21);
    dwa_planners[0]->set_sample_axis(35);
}


// 仅测试一辆车
void Planner::plan()
{
    auto time_start = std::chrono::high_resolution_clock::now();
    dwa_planners[0]->set_sim_t(this->sim_time, this->run_time);
    int car_index = 0;
    // 在 handle_receive 中已经更新了 v_robot_pos 和 v_robot_velocity
    // double car_v = v_robot_velocity[car_index].v_norm;
    double car_v = v_robot_ctrl_last[car_index].velocity;
    double car_theta = v_robot_ctrl_last[car_index].theta;
    // double car_v = v_robot_ctrl_last[car_index].velocity;
    // double car_theta = 0;

    double car_x = v_robot_pos[car_index].x;
    double car_y = v_robot_pos[car_index].y;
    double car_phi = v_robot_pos[car_index].phi;

    double target_x = v_target_pos[car_index].x;
    double target_y = v_target_pos[car_index].y;
    double target_phi = v_target_pos[car_index].phi;
    double err_x = target_x - car_x;
    double err_y = target_y - car_y;
    double err_phi = target_phi - car_phi;
    Vector2d err_xy = {err_x, err_y};
    double err_norm = err_xy.norm();
    if (err_norm < tolerance_err_dis && abs(err_phi) < tolerance_err_phi)
    {
        v_robot_ctrl[car_index].velocity = 0;
        v_robot_ctrl[car_index].theta = 0;
        v_robot_ctrl_ready[0] = true;
        cout << "arrive target" << endl;
        return;
    }
    double car_v_dir = car_v / abs(car_v);
    std::vector<double> vec_car2goal = {target_x - car_x, target_y - car_y};
    cout << "vec_car2goal: " << vec_car2goal[0] << ", " << vec_car2goal[1] << endl;
    std::vector<double> vec_car = {std::cos(car_phi) * car_v_dir, std::sin(car_phi) * car_v_dir};
    cout << "vec_car: " << vec_car[0] << ", " << vec_car[1] << endl;
    double angle = cal_angle_of_vect(vec_car, vec_car2goal);
    cout << "diff angle: " << angle << endl;
    // cout << "car_v: " << car_v << endl;
    // cout << "car_x: " << car_x << endl;
    // cout << "car_y: " << car_y << endl;
    // cout << "car_phi: " << car_phi << endl;
    // cout << "car_theta: " << car_theta << endl;
    // cout << "target_x: " << target_x << endl;
    // cout << "target_y: " << target_y << endl;
    // cout << "target_phi: " << target_phi << endl;
    // 规划
    vector<double> car_x_y_phi = {car_x, car_y, car_phi};
    vector<double> target_x_y_phi = {target_x, target_y, target_phi};
    dwa_planners[car_index]->set_weight_gain(wieght_gain[0], wieght_gain[1], wieght_gain[2], wieght_gain[3], wieght_gain[4], wieght_gain[5]);
    dwa_planners[car_index]->plan(car_v, car_theta, car_x_y_phi, target_x_y_phi);
    auto [ctrl_v, ctrl_theta] = dwa_planners[car_index]->get_control();
    // 设置控制量
    v_robot_ctrl[car_index].velocity = ctrl_v;
    v_robot_ctrl[car_index].theta = ctrl_theta;
    v_robot_ctrl_ready[0] = true;
    // Sleep(time_ms);
    auto time_end = std::chrono::high_resolution_clock::now();
    // 输出花费的时间
    auto time_cost = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();
    cout << "Time cost: " << time_cost*0.001 << " ms" << endl;
    cout << "========================================================================================" << endl;
}

void Planner::set_target_pos(int robot_index, double x, double y, double phi)
{
    v_target_pos[robot_index].x = x;
    v_target_pos[robot_index].y = y;
    v_target_pos[robot_index].phi = phi;
}