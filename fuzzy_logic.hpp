#ifndef FUZZY_LOGIC_HPP
#define FUZZY_LOGIC_HPP
#include <iostream>
#include <cmath>
#include <vector>
#include <tuple>
#include "common_function.hpp"

using namespace std;

class FuzzySystem
{
public:
    FuzzySystem();

    std::pair<double, double> function_sigmod(double x, double k, double x_min, double x_max, double slope);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
    fuzzification(double goal, double d_phi, double obstacle);

    std::vector<double>
    rule_base(
        const std::vector<double> &ms_goal,
        const std::vector<double> &ms_d_phi,
        const std::vector<double> &ms_obstacle);

    void defuzzification(const std::vector<double> &rules);

    tuple<double, double, double, double, double> 
        inference(double goal, double d_phi, double obstacle);
    enum collection { N = 0, F = 1 };
    enum w_name { Heading = 0, Velocity = 1, Direction = 2, Dis_vect = 3, Dis_goal = 4};
private:
    vector<double> rule_value;
    vector<double> weight_output; // {Heading, Velocity, Direction, Dis_vect, Dis_goal};
    vector<double> range_distance;
    double k_distance;
    vector<double> ms_distance;

    vector<double> range_d_phi;
    double k_d_phi;
    vector<double> ms_d_phi;

    vector<double> range_obstacle;
    double k_obstacle;
    vector<double> ms_obstacle;

    vector<vector<double>> defuzzy_rules = {
        {0.4, 0.7, 0.3, 0.6},   // heading
        {0.6, 0.6, 0.2, 0.2},   // velocity
        {0.2, 0.1, 0.9, 0.8},   // direction
        {0.6, 0.8, 0.2, 0.4},   //dis_vect
        {0.1, 0.1, 0.2, 0.2}    // dis_goal
        // {1.0, 1.0, 0.2, 0.6},   // heading
        // {0.8, 1.0, 0.3, 0.5},   // velocity
        // {0.3, 0.1, 1.0, 0.8},   // direction
        // {0.3, 0.1, 0.8, 1.0},   //dis_vect
        // {0.1, 0.1, 0.3, 1.0}    // dis_goal

    };
};

// int main() {
//     FuzzySystem fs;
//     double goal = 10;
//     double d_phi = M_PI / 4;
//     double obstacle = 10;
//     double output = fs.inference(goal, d_phi, obstacle);
//     std::cout << "Output: " << output << std::endl;
//     return 0;
// }

#endif // FUZZY_LOGIC_HPP