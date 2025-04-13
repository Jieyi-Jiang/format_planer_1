#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include "planner.hpp"
#include "common_function.hpp"
#include "fuzzy_logic.hpp"
#include "car_model.hpp"
#include "car_planner.hpp"
#include <atomic>
using Eigen::MatrixXd;
using namespace std;
void th_recive(Planner *planner)
{
    while (true)
    {
        planner->receive_data();
    }
}

void th_send(Planner *planner)
{
    while (true)
    {

        planner->send_data();
        Sleep(1);
    }
}

void th_send_heartbeat(Planner *planner)
{
    while (true)
    {

        planner->send_heartbeat();
        Sleep(1000);
    }
}

void th_plan(Planner *planner)
{
    while (true)
    {
        planner->fomat_plan();
        planner->plan(0);
        planner->plan(1);
        planner->plan(2);
        Sleep(5);
    }
}

int main_fuzzy();

int main_car_model();

int main_car_planer();

/// @brief  测试BoundedDWA类
/// @return 
/// @note   测试BoundedDWA类的plan函数，计算从当前点到目标点的最优路径
/// 测试通过 ！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
int main_car_planer_2();


int main()
{
    Planner planner;
    // planner.config_param();
    // planner.config_network("232.10.11.12", "3333", "3333");
    thread th1(th_recive, &planner);
    thread th2(th_send, &planner);
    thread th3(th_send_heartbeat, &planner);
    thread th4(th_plan, &planner);
    // main_fuzzy();
    // main_car_model();
    // main_car_planer();
    // main_car_planer_2();
    while (true)
    {
        Sleep(1000);
    }
    th1.join();
    th2.join();
    th3.join();
    // th4.join();
    return 0;
}


int main_fuzzy() 
{
    FuzzySystem fs;
    double goal = 10;
    double d_phi = M_PI / 4;
    double obstacle = 10;
    // double output = fs.inference(goal, d_phi, obstacle);
    // std::cout << "Output: " << output << std::endl;
    auto [w1, w2, w3, w4, w5] = fs.inference(goal, d_phi);
    cout << "fuzzy out | " << w1 << " " << w2 << " " << w3 << " " << w4 << " " << w5 << endl;
    return 0;
}

int main_car_model()
{
    CarModel car;
    car.inputControl(0.5, 0.3);
    car.update(0.1);
    double x, y, phi;
    car.getState(x, y, phi);
    std::cout << "x: " << x << ", y: " << y << ", phi: " << phi << std::endl;
    return 0;
}

int main_car_planer() {
    BoundedDWA BoundedDWA_test;
    BoundedDWA_test.set_sim_t(0.1, 0.1);
    BoundedDWA_test.set_sim_step_num(15);
    BoundedDWA_test.set_sample_axis(9);
    double car_v = 0.2;
    double car_theta = 0.3;
    std::tuple<double, double, double> car_pos = std::make_tuple(0.0, 0.0, 1.57);
    std::tuple<double, double, double> target_pos = std::make_tuple(0.0, 0.5, 0.0);
    BoundedDWA_test.cal_sample_space(car_v, car_theta);
    std::vector<std::vector<std::vector<double>>> test_space = BoundedDWA_test.get_sample_space();
    for (int i = 0; i < test_space.size(); i++) {
        for (int j = 0; j < test_space[i].size(); j++) {
            for (int k = 0; k < test_space[i][j].size(); k++) {
                std::cout << test_space[i][j][k] << " ";
            }
        }
    }
    return 0;
}

/// @brief  测试BoundedDWA类
/// @return 
/// @note   测试BoundedDWA类的plan函数，计算从当前点到目标点的最优路径
/// 测试通过 ！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
int main_car_planer_2() {
    auto time_start = std::chrono::high_resolution_clock::now();
    BoundedDWA BoundedDWA_test;
    BoundedDWA_test.set_sim_t(1.0, 1.0);
    BoundedDWA_test.set_sim_step_num(15);
    BoundedDWA_test.set_sample_axis(15);
    double car_v = 0.0;
    double car_theta = 0.3;
    vector<double> car_pos = {0.0, 0.1, 0};
    vector<double> target_pos = {0.4, 0.4, 0};
    auto plan_test = BoundedDWA_test.plan(car_v, car_theta, car_pos, target_pos);
    // BoundedDWA_test.update_weight(1.0, 1.0, 1.0, 1.0, 1.0);
    auto [cost_list, max_cost, max_idx] = BoundedDWA_test.cost_overall();
    // auto max_element_iter = std::max_element(cost_list.begin(), cost_list.end());
    // double max_val = *max_element_iter;
    // int max_idx = std::distance(cost_list.begin(), max_element_iter);
    cout << max_idx << endl;
    auto time_end = std::chrono::high_resolution_clock::now();
    // 输出花费的时间
    auto time_cost = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();
    cout << "Time cost: " << time_cost*0.001 << " ms" << endl;
    return 0;
}