#ifndef COMMON_FUNCTION_HPP
#define COMMON_FUNCTION_HPP
#include <vector>


#define PI (3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679)
#define M_PI (3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679)

double threshold(double x, double min, double max);

double cal_angle_of_vect(std::vector<double> a, std::vector<double> b);

std::pair<double, int> find_max_and_index(const std::vector<double>& arr);

std::pair<double, int> find_min_and_index(const std::vector<double>& arr);

double cal_similarity_of_vect(std::vector<double> a, std::vector<double> b);

#endif // COMMON_FUNCTION_HPP

