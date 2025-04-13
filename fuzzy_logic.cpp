#include <iostream>
#include <cmath>
#include <vector>
#include <tuple>
#include "fuzzy_logic.hpp"
#include "common_function.hpp"

using namespace std;


std::pair<double, double> 
function_sigmod(double x, double k, double x_min, double x_max, double slope = 30)
{
    double x_range = std::abs(x_max - x_min);
    double u_x = (x - x_min) / x_range;
    double u_k = (k - x_min) / x_range;
    double a = slope;
    double value = 1 / (1 + std::exp(-a * (u_x - u_k)));
    return {1 - value, value}; // near, far
}

