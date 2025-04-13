#include <iostream>
#include <cmath>
#include <vector>
#include <tuple>
#include "fuzzy_logic.hpp"
#include "common_function.hpp"

using namespace std;

FuzzySystem::FuzzySystem()
{
    // goal
    range_distance = {0, 1.5};
    k_distance = 0.75;
    ms_distance = {0.0, 0.0}; // near, far
    // d_phi
    range_d_phi = {0, M_PI};
    k_d_phi = M_PI / 2; //2
    ms_d_phi = {0, 0}; // near, far
    // obstacle
    range_obstacle = {0, 1};
    k_obstacle = 0.6;
    ms_obstacle = {0, 0}; // near, far
    weight_output.resize(5);
}

std::pair<double, double> 
FuzzySystem::function_sigmod(double x, double k, double x_min, double x_max, double slope = 30)
{
    double x_range = std::abs(x_max - x_min);
    double u_x = (x - x_min) / x_range;
    double u_k = (k - x_min) / x_range;
    double a = slope;
    double value = 1 / (1 + std::exp(-a * (u_x - u_k)));
    return {1 - value, value}; // near, far
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
FuzzySystem::fuzzification(double goal, double d_phi, double obstacle)
{
    // distance
    auto [distance_near, distance_far] = function_sigmod(goal, k_distance, range_distance[0], range_distance[1], 20);
    ms_distance = {distance_near, distance_far};
    std::cout << "goal:   N: " << ms_distance[0] << ", F: " << ms_distance[1] << std::endl;
    // d_phi
    auto [d_phi_near, d_phi_far] = function_sigmod(d_phi, k_d_phi, range_d_phi[0], range_d_phi[1], 20);
    ms_d_phi = {d_phi_near, d_phi_far};
    std::cout << "d_phi:  N: " << ms_d_phi[0] << ", F: " << ms_d_phi[1] << std::endl;
    // obstacle
    auto [obstacle_near, obstacle_far] = function_sigmod(obstacle, k_obstacle, range_obstacle[0], range_obstacle[1]);
    ms_obstacle = {obstacle_near, obstacle_far};
    std::cout << "obs:    N: " << ms_obstacle[0] << ", F: " << ms_obstacle[1] << std::endl;
    return {ms_distance, ms_d_phi, ms_obstacle};
} 

std::vector<double>
FuzzySystem::rule_base(
    const std::vector<double> &ms_goal,
    const std::vector<double> &ms_d_phi,
    const std::vector<double> &ms_obstacle)
{
    std::vector<std::vector<int>> menbership = {
        {F, F},
        {F, N},
        {N, F},
        {N, N},
    };

    std::vector<double> rules(4);
    rules[0] = std::min(ms_goal[menbership[0][0]], ms_d_phi[menbership[0][1]]);
    rules[1] = std::min(ms_goal[menbership[1][0]], ms_d_phi[menbership[1][1]]);
    rules[2] = std::min(ms_goal[menbership[2][0]], ms_d_phi[menbership[2][1]]);
    rules[3] = std::min(ms_goal[menbership[3][0]], ms_d_phi[menbership[3][1]]);
    std::cout << "R1: " << rules[0] << ", R2: " << rules[1] << ", R3: " << rules[2] << ", R4: " << rules[3] << std::endl;
    return rules;
}

void FuzzySystem::defuzzification(const std::vector<double> &rules)
{
    // vector<vector<double>> defuzzy_rules = {
    //     {1,0, 1.0, 0.2, 0.6},   // heading
    //     {0.8, 1.0, 0.3, 0.5},   // velocity
    //     {0.3, 0.1, 1.0, 0.8},   // direction
    //     {0.3, 0.1, 0.8, 1.0},   //dis_vect
    //     {0.1, 0.1, 0.3, 1.0}    // dis_goal

    // };
    for (int i = 0; i < 5; i++) {
        cout << i+1 << " | defuzzy_rules[" << i+1 << "]: " << defuzzy_rules[i][0] << ", " << defuzzy_rules[i][1] << ", " << defuzzy_rules[i][2] << ", " << defuzzy_rules[i][3] << endl;
        double u_r1 = rules[0] * this->defuzzy_rules[i][0];
        double u_r2 = rules[1] * this->defuzzy_rules[i][1];
        double u_r3 = rules[2] * this->defuzzy_rules[i][2];
        double u_r4 = rules[3] * this->defuzzy_rules[i][3];
        cout << i+1 << " | u_r1: " << u_r1 << ", u_r2: " << u_r2 << ", u_r3: " << u_r3 << ", u_r4: " << u_r4 << endl;
        // cout << "u_r1: " << u_r1 << ", u_r2: " << u_r2 << ", u_r3: " << u_r3 << ", u_r4: " << u_r4 << endl;
        double u_sum = u_r1 + u_r2 + u_r3 + u_r4;
        double r_sum = rules[0] + rules[1] + rules[2] + rules[3];
        weight_output[i] = u_sum / r_sum;
    }
}

tuple<double, double, double, double, double> 
FuzzySystem::inference(double goal, double d_phi, double obstacle)
{
    auto [ms_goal, ms_d_phi, ms_obstacle] = fuzzification(goal, d_phi, obstacle);
    std::vector<double> rules = rule_base(ms_goal, ms_d_phi, ms_obstacle);
    // std::cout << "R1: " << rules[0] << ", R2: " << rules[1] << ", R3: " << rules[2] << ", R4: " << rules[3] << ", R5: " << rules[4] << ", R6: " << rules[5] << ", R7: " << rules[6] << ", R8: " << rules[7] << std::endl;
    defuzzification(rules);
    return make_tuple(weight_output[0], weight_output[1], weight_output[2], weight_output[3], weight_output[4]);
}
