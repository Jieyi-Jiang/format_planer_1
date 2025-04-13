// #ifndef FUZZY_LOGIC_HPP
// #define FUZZY_LOGIC_HPP
// #include <iostream>
// #include <cmath>
// #include <vector>
// #include "common_function.hpp"

// class FuzzySystem
// {
// public:
//     FuzzySystem();

//     std::pair<double, double> function_sigmod(double x, double k, double x_min, double x_max, double slope);

//     std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
//     fuzzification(double goal, double d_phi, double obstacle);

//     std::vector<double>
//     rule_base(
//         const std::vector<double> &ms_goal,
//         const std::vector<double> &ms_d_phi,
//         const std::vector<double> &ms_obstacle);

//     double defuzzification(const std::vector<double> &rules);

//     double inference(double goal, double d_phi, double obstacle);

// private:
//     std::vector<double> range_goal;
//     double k_goal;
//     std::vector<double> ms_goal;

//     std::vector<double> range_d_phi;
//     double k_d_phi;
//     std::vector<double> ms_d_phi;

//     std::vector<double> range_obstacle;
//     double k_obstacle;
//     std::vector<double> ms_obstacle;
// };

// // int main() {
// //     FuzzySystem fs;
// //     double goal = 10;
// //     double d_phi = M_PI / 4;
// //     double obstacle = 10;
// //     double output = fs.inference(goal, d_phi, obstacle);
// //     std::cout << "Output: " << output << std::endl;
// //     return 0;
// // }

// #endif // FUZZY_LOGIC_HPP