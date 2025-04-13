// #include <iostream>
// #include <cmath>
// #include <vector>
// #include <tuple>
// #include "fuzzy_logic.hpp"
// #include "common_function.hpp"

// using namespace std;

// FuzzySystem::FuzzySystem()
// {
//     // goal
//     range_goal = {0, 1.5};
//     k_goal = 0.6;
//     ms_goal = {0.0, 0.0}; // near, far
//     // d_phi
//     range_d_phi = {0, M_PI};
//     k_d_phi = M_PI / 4;
//     ms_d_phi = {0, 0}; // near, far
//     // obstacle
//     range_obstacle = {0, 1};
//     k_obstacle = 0.6;
//     ms_obstacle = {0, 0}; // near, far
// }

// std::pair<double, double> 
// FuzzySystem::function_sigmod(double x, double k, double x_min, double x_max, double slope = 20)
// {
//     double x_range = std::abs(x_max - x_min);
//     double u_x = (x - x_min) / x_range;
//     double u_k = (k - x_min) / x_range;
//     double a = slope;
//     double value = 1 / (1 + std::exp(-a * (u_x - u_k)));
//     return {1 - value, value}; // near, far
// }

// std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
// FuzzySystem::fuzzification(double goal, double d_phi, double obstacle)
// {
//     // goal
//     auto [goal_near, goal_far] = function_sigmod(goal, k_goal, range_goal[0], range_goal[1]);
//     ms_goal = {goal_near, goal_far};
//     // std::cout << "goal:   N: " << ms_goal[0] << ", F: " << ms_goal[1] << std::endl;
//     // d_phi
//     auto [d_phi_near, d_phi_far] = function_sigmod(d_phi, k_d_phi, range_d_phi[0], range_d_phi[1]);
//     ms_d_phi = {d_phi_near, d_phi_far};
//     // std::cout << "d_phi:  N: " << ms_d_phi[0] << ", F: " << ms_d_phi[1] << std::endl;
//     // obstacle
//     auto [obstacle_near, obstacle_far] = function_sigmod(obstacle, k_obstacle, range_obstacle[0], range_obstacle[1]);
//     ms_obstacle = {obstacle_near, obstacle_far};
//     // std::cout << "obs:    N: " << ms_obstacle[0] << ", F: " << ms_obstacle[1] << std::endl;
//     return {ms_goal, ms_d_phi, ms_obstacle};
// }

// std::vector<double>
// FuzzySystem::rule_base(
//     const std::vector<double> &ms_goal,
//     const std::vector<double> &ms_d_phi,
//     const std::vector<double> &ms_obstacle)
// {
//     std::vector<std::vector<int>> menbership = {
//         {0, 1, 0},
//         {1, 1, 0},
//         {0, 1, 1},
//         {1, 1, 1},
//         {0, 0, 0},
//         {1, 0, 0},
//         {0, 0, 1},
//         {1, 0, 1},
//     };

//     std::vector<double> rules(8);
//     rules[0] = std::min(ms_goal[menbership[0][0]], std::min(ms_d_phi[menbership[0][1]], ms_obstacle[menbership[0][2]]));
//     rules[1] = std::min(ms_goal[menbership[1][0]], std::min(ms_d_phi[menbership[1][1]], ms_obstacle[menbership[1][2]]));
//     rules[2] = std::min(ms_goal[menbership[2][0]], std::min(ms_d_phi[menbership[2][1]], ms_obstacle[menbership[2][2]]));
//     rules[3] = std::min(ms_goal[menbership[3][0]], std::min(ms_d_phi[menbership[3][1]], ms_obstacle[menbership[3][2]]));
//     rules[4] = std::min(ms_goal[menbership[4][0]], std::min(ms_d_phi[menbership[4][1]], ms_obstacle[menbership[4][2]]));
//     rules[5] = std::min(ms_goal[menbership[5][0]], std::min(ms_d_phi[menbership[5][1]], ms_obstacle[menbership[5][2]]));
//     rules[6] = std::min(ms_goal[menbership[6][0]], std::min(ms_d_phi[menbership[6][1]], ms_obstacle[menbership[6][2]]));
//     rules[7] = std::min(ms_goal[menbership[7][0]], std::min(ms_d_phi[menbership[7][1]], ms_obstacle[menbership[7][2]]));
//     return rules;
// }

// double FuzzySystem::defuzzification(const std::vector<double> &rules)
// {
//     std::vector<double> U = {0.1, 0.2, 0.4, 0.8, 1.0 - 0.8, 1.0 - 0.4, 1.0 - 0.1, 1.0 - 0.1};
//     double U_output = 0.0;
//     for (size_t i = 0; i < rules.size(); ++i)
//     {
//         U_output += U[i] * rules[i];
//     }
//     return U_output;
// }

// double FuzzySystem::inference(double goal, double d_phi, double obstacle)
// {
//     auto [ms_goal, ms_d_phi, ms_obstacle] = fuzzification(goal, d_phi, obstacle);
//     std::vector<double> rules = rule_base(ms_goal, ms_d_phi, ms_obstacle);
//     // std::cout << "R1: " << rules[0] << ", R2: " << rules[1] << ", R3: " << rules[2] << ", R4: " << rules[3] << ", R5: " << rules[4] << ", R6: " << rules[5] << ", R7: " << rules[6] << ", R8: " << rules[7] << std::endl;
//     double output = defuzzification(rules);
//     return output;
// }
