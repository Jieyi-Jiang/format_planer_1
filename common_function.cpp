#include <vector>
#include <cmath>
#include <iostream>

#include "common_function.hpp"
using namespace std;
double threshold(double x, double min, double max) {
    if (x < min) {
        return min;
    } else if (x > max) {
        return max;
    } else {
        return x;
    }
}

double cal_angle_of_vect(std::vector<double> a, std::vector<double> b) {
    double dot_product = a[0] * b[0] + a[1] * b[1];
    double magnitude_a = std::sqrt(a[0] * a[0] + a[1] * a[1]);
    double magnitude_b = std::sqrt(b[0] * b[0] + b[1] * b[1]);
    if (magnitude_a == 0) magnitude_a = 1e-20;
    if (magnitude_b == 0) magnitude_b = 1e-20;
    double cosine_of_angle = dot_product / (magnitude_a * magnitude_b);
    double angle_rad = std::acos(cosine_of_angle);
    double angle_deg = angle_rad * (180.0 / M_PI);
    return angle_deg;
}
double cal_angle_of_vect_rad(std::vector<double> a, std::vector<double> b) {
    double dot_product = a[0] * b[0] + a[1] * b[1];
    double magnitude_a = std::sqrt(a[0] * a[0] + a[1] * a[1]);
    double magnitude_b = std::sqrt(b[0] * b[0] + b[1] * b[1]);
    if (magnitude_a == 0) magnitude_a = 1e-20;
    if (magnitude_b == 0) magnitude_b = 1e-20;
    double cosine_of_angle = dot_product / (magnitude_a * magnitude_b);
    double angle_rad = std::acos(cosine_of_angle);
    return angle_rad;
}
double cal_similarity_of_vect(std::vector<double> a, std::vector<double> b) {
    double dot_product = a[0] * b[0] + a[1] * b[1];
    double magnitude_a = std::sqrt(a[0] * a[0] + a[1] * a[1]);
    double magnitude_b = std::sqrt(b[0] * b[0] + b[1] * b[1]);
    if (magnitude_a == 0) magnitude_a = 1e-20;
    if (magnitude_b == 0) magnitude_b = 1e-20;
    double cosine_of_angle = dot_product / (magnitude_a * magnitude_b);
    // arccos = math.acos(cosine_of_angle)
    // return (math.pi - arccos) / math.pi
    double arcos = acos(cosine_of_angle);
    // return cosine_of_angle + 1.0;
    return (M_PI - arcos) / M_PI;
}

std::pair<double, int> find_max_and_index(const std::vector<double>& arr) {
    try {
        // Check if the array is empty
        if (arr.empty()) {
            throw std::invalid_argument("Array is empty");
        }
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return std::make_pair(0.0, -1);
    }
    double max_value = arr[0];
    int max_index = 0;
    for (int i = 0; i < arr.size(); ++i) {
        if (arr[i] > max_value) {
            max_value = arr[i];
            max_index = i;
        }
    }
    // cout << "6 " << endl;
    return std::make_pair(max_value, max_index);
}

std::pair<double, int> find_min_and_index(const std::vector<double>& arr) {
    double min_value = arr[0];
    int min_index = 0;
    for (int i = 0; i < arr.size(); ++i) {
        if (arr[i] < min_value) {
            min_value = arr[i];
            min_index = i;
        }
    }
    return std::make_pair(min_value, min_index);
}
