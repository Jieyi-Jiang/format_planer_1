#ifndef FUZZY_LOGIC_HPP
#define FUZZY_LOGIC_HPP
#include <iostream>
#include <cmath>
#include <vector>
#include <tuple>
#include "common_function.hpp"
#include "common_function.hpp"

using namespace std;

pair<double, double> function_sigmod(double x, double k, double x_min, double x_max, double slope);

class FuzzySystem
{
public:
    FuzzySystem()
    {
        // goal
        range_distance = {0, 2.5};
        k_distance = 1.2;
        ms_distance = {0.0, 0.0}; // near, far
        // d_phi
        range_d_phi = {0, M_PI};
        k_d_phi = M_PI / 2; // 2
        ms_d_phi = {0, 0};  // near, far
        // obstacle
        range_obstacle = {0, 1};
        k_obstacle = 0.6;
        ms_obstacle = {0, 0}; // near, far
        weight_output.resize(5);
        // dis_vert
        range_dis_vert = {0, 0.8};
        k_dis_vert = 0.3;
        ms_dis_vert = {0, 0}; // near, far
    }

    tuple<vector<double>, vector<double>>
    fuzzification(const double goal, const double d_phi)
    {
        // distance
        auto [distance_near, distance_far] = function_sigmod(goal, k_distance, range_distance[0], range_distance[1], 20);
        ms_distance = {distance_near, distance_far};
        cout << "goal:   N: " << ms_distance[0] << ", F: " << ms_distance[1] << endl;
        // d_phi
        auto [d_phi_near, d_phi_far] = function_sigmod(d_phi, k_d_phi, range_d_phi[0], range_d_phi[1], 20);
        ms_d_phi = {d_phi_near, d_phi_far};
        cout << "d_phi:  N: " << ms_d_phi[0] << ", F: " << ms_d_phi[1] << endl;
        return {ms_distance, ms_d_phi};
    }
    tuple<vector<double>, vector<double>, vector<double>>
    fuzzification(const double goal, const double d_phi, const double dis_vert)
    {
        // distance
        auto [distance_near, distance_far] = function_sigmod(goal, k_distance, range_distance[0], range_distance[1], 20);
        ms_distance = {distance_near, distance_far};
        cout << "goal:   N: " << ms_distance[0] << ", F: " << ms_distance[1] << endl;
        // d_phi
        auto [d_phi_near, d_phi_far] = function_sigmod(d_phi, k_d_phi, range_d_phi[0], range_d_phi[1], 20);
        ms_d_phi = {d_phi_near, d_phi_far};
        cout << "d_phi:  N: " << ms_d_phi[0] << ", F: " << ms_d_phi[1] << endl;
        // dis_vert
        auto [dis_vert_near, dis_vert_far] = function_sigmod(dis_vert, k_dis_vert, range_dis_vert[0], range_dis_vert[1], 30);
        ms_dis_vert = {dis_vert_near, dis_vert_far};
        cout << "dis_vert:  N: " << ms_dis_vert[0] << ", F: " << ms_dis_vert[1] << endl;
        return {ms_distance, ms_d_phi, ms_dis_vert};
    }

    vector<double> rule_base( const vector<double> &ms_goal, const vector<double> &ms_d_phi)
    {

        vector<double> rules(4);
        rules[0] = min(ms_goal[menbership_1[0][0]], ms_d_phi[menbership_1[0][1]]);
        rules[1] = min(ms_goal[menbership_1[1][0]], ms_d_phi[menbership_1[1][1]]);
        rules[2] = min(ms_goal[menbership_1[2][0]], ms_d_phi[menbership_1[2][1]]);
        rules[3] = min(ms_goal[menbership_1[3][0]], ms_d_phi[menbership_1[3][1]]);
        cout << "R1: " << rules[0] << ", R2: " << rules[1] << ", R3: " << rules[2] << ", R4: " << rules[3] << endl;
        return rules;
    }
    vector<double> rule_base( const vector<double> &ms_goal, const vector<double> &ms_d_phi, const vector<double> &ms_dis_vert)
    {

        vector<double> rules(8);
        rules[0] = min(min(ms_goal[menbership_2[0][0]], ms_d_phi[menbership_2[0][1]]), ms_dis_vert[menbership_2[0][2]]);
        rules[1] = min(min(ms_goal[menbership_2[1][0]], ms_d_phi[menbership_2[1][1]]), ms_dis_vert[menbership_2[1][2]]);
        rules[2] = min(min(ms_goal[menbership_2[2][0]], ms_d_phi[menbership_2[2][1]]), ms_dis_vert[menbership_2[2][2]]);
        rules[3] = min(min(ms_goal[menbership_2[3][0]], ms_d_phi[menbership_2[3][1]]), ms_dis_vert[menbership_2[3][2]]);
        rules[4] = min(min(ms_goal[menbership_2[4][0]], ms_d_phi[menbership_2[4][1]]), ms_dis_vert[menbership_2[4][2]]);
        rules[5] = min(min(ms_goal[menbership_2[5][0]], ms_d_phi[menbership_2[5][1]]), ms_dis_vert[menbership_2[5][2]]);
        rules[6] = min(min(ms_goal[menbership_2[6][0]], ms_d_phi[menbership_2[6][1]]), ms_dis_vert[menbership_2[6][2]]);
        rules[7] = min(min(ms_goal[menbership_2[7][0]], ms_d_phi[menbership_2[7][1]]), ms_dis_vert[menbership_2[7][2]]);

        // cout << "R1: " << rules[0] << ", R2: " << rules[1] << ", R3: " << rules[2] << ", R4: " << rules[3];
        // cout << ", R5: " << rules[4] << ", R6: " << rules[5] << ", R7: " << rules[6] << ", R8: " << rules[7] << endl;
        return rules;
    }

    void defuzzification_1(const vector<double> &rules)
    {
        // vector<vector<double>> defuzzy_rules = {
        //     {1,0, 1.0, 0.2, 0.6},   // heading
        //     {0.8, 1.0, 0.3, 0.5},   // velocity
        //     {0.3, 0.1, 1.0, 0.8},   // direction
        //     {0.3, 0.1, 0.8, 1.0},   //dis_vect
        //     {0.1, 0.1, 0.3, 1.0}    // dis_goal

        // };
        for (int i = 0; i < 5; i++)
        {
            cout << i + 1 << " | defuzzy_rules[" << i + 1 << "]: " << defuzzy_rules_1[i][0] << ", " << defuzzy_rules_1[i][1] << ", " << defuzzy_rules_1[i][2] << ", " << defuzzy_rules_1[i][3] << endl;
            double u_r1 = rules[0] * this->defuzzy_rules_1[i][0];
            double u_r2 = rules[1] * this->defuzzy_rules_1[i][1];
            double u_r3 = rules[2] * this->defuzzy_rules_1[i][2];
            double u_r4 = rules[3] * this->defuzzy_rules_1[i][3];
            cout << i + 1 << " | u_r1: " << u_r1 << ", u_r2: " << u_r2 << ", u_r3: " << u_r3 << ", u_r4: " << u_r4 << endl;
            // cout << "u_r1: " << u_r1 << ", u_r2: " << u_r2 << ", u_r3: " << u_r3 << ", u_r4: " << u_r4 << endl;
            double u_sum = u_r1 + u_r2 + u_r3 + u_r4;
            double r_sum = rules[0] + rules[1] + rules[2] + rules[3];
            weight_output[i] = u_sum / r_sum;
        }
    }
    void defuzzification_2(const vector<double> &rules)
    {
        // vector<vector<double>> defuzzy_rules = {
        //     {1,0, 1.0, 0.2, 0.6},   // heading
        //     {0.8, 1.0, 0.3, 0.5},   // velocity
        //     {0.3, 0.1, 1.0, 0.8},   // direction
        //     {0.3, 0.1, 0.8, 1.0},   //dis_vect
        //     {0.1, 0.1, 0.3, 1.0}    // dis_goal

        // };
        for (int i = 0; i < 5; i++)
        {
            // cout << i + 1 << " | defuzzy_rules[" << i + 1 << "]: " << defuzzy_rules_2[i][0] << ", " << defuzzy_rules_2[i][1] << ", " << defuzzy_rules_2[i][2] << ", " << defuzzy_rules_2[i][3];
            // cout << ", " << defuzzy_rules_2[i][4] << ", " << defuzzy_rules_2[i][5] << ", " << defuzzy_rules_2[i][6] << ", " << defuzzy_rules_2[i][7] << endl;
            double u_r1 = rules[0] * this->defuzzy_rules_2[0][i];
            double u_r2 = rules[1] * this->defuzzy_rules_2[1][i];
            double u_r3 = rules[2] * this->defuzzy_rules_2[2][i];
            double u_r4 = rules[3] * this->defuzzy_rules_2[3][i];
            double u_r5 = rules[4] * this->defuzzy_rules_2[4][i];
            double u_r6 = rules[5] * this->defuzzy_rules_2[5][i];
            double u_r7 = rules[6] * this->defuzzy_rules_2[6][i];
            double u_r8 = rules[7] * this->defuzzy_rules_2[7][i];
            // cout << i + 1 << " | u_r1: " << u_r1 << ", u_r2: " << u_r2 << ", u_r3: " << u_r3 << ", u_r4: " << u_r4;
            // cout << ", u_r5: " << u_r5 << ", u_r6: " << u_r6 << ", u_r7: " << u_r7 << ", u_r8: " << u_r8 << endl;
            // cout << "u_r1: " << u_r1 << ", u_r2: " << u_r2 << ", u_r3: " << u_r3 << ", u_r4: " << u_r4 << endl;
            double u_sum = u_r1 + u_r2 + u_r3 + u_r4 + u_r5 + u_r6 + u_r7 + u_r8;
            double r_sum = rules[0] + rules[1] + rules[2] + rules[3] + rules[4] + rules[5] + rules[6] + rules[7];
            weight_output[i] = u_sum / r_sum;
        }
        weight_output[0] =  weight_output[0] * (1-weight_output[3]);
    }

    tuple<double, double, double, double, double>
    inference(double goal, double d_phi)
    {
        auto [ms_goal, ms_d_phi] = fuzzification(goal, d_phi);
        vector<double> rules = rule_base(ms_goal, ms_d_phi);
        // cout << "R1: " << rules[0] << ", R2: " << rules[1] << ", R3: " << rules[2] << ", R4: " << rules[3] << ", R5: " << rules[4] << ", R6: " << rules[5] << ", R7: " << rules[6] << ", R8: " << rules[7] << endl;
        defuzzification_1(rules);
        return make_tuple(weight_output[0], weight_output[1], weight_output[2], weight_output[3], weight_output[4]);
    }
    tuple<double, double, double, double, double>
    inference(const double goal, const double d_phi, const double dis_vert)
    {
        auto [ms_goal, ms_d_phi, ms_dis_vert] = fuzzification(goal, d_phi, dis_vert);
        vector<double> rules = rule_base(ms_goal, ms_d_phi, ms_dis_vert);
        // cout << "R1: " << rules[0] << ", R2: " << rules[1] << ", R3: " << rules[2] << ", R4: " << rules[3] << ", R5: " << rules[4] << ", R6: " << rules[5] << ", R7: " << rules[6] << ", R8: " << rules[7] << endl;
        defuzzification_2(rules);
        return make_tuple(weight_output[0], weight_output[1], weight_output[2], weight_output[3], weight_output[4]);
    }
    enum collection
    {
        N = 0,
        F = 1
    };
    enum w_name
    {
        Heading = 0,
        Velocity = 1,
        Direction = 2,
        Dis_vect = 3,
        Dis_goal = 4
    };

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

    vector<double> range_dis_vert;
    double k_dis_vert;
    vector<double> ms_dis_vert;

    vector<vector<double>> defuzzy_rules_1 = {
        {0.4, 0.7, 0.3, 0.6}, // heading
        {0.6, 0.6, 0.2, 0.2}, // velocity
        {0.2, 0.1, 0.9, 0.8}, // direction
        {0.6, 0.8, 0.2, 0.4}, // dis_vect
        {0.1, 0.1, 0.2, 0.2}  // dis_goal
        // {1.0, 1.0, 0.2, 0.6},   // heading
        // {0.8, 1.0, 0.3, 0.5},   // velocity
        // {0.3, 0.1, 1.0, 0.8},   // direction
        // {0.3, 0.1, 0.8, 1.0},   //dis_vect
        // {0.1, 0.1, 0.3, 1.0}    // dis_goal

    };
    vector<vector<int>> menbership_1 = {
        {F, F},
        {F, N},
        {N, F},
        {N, N},
    };

    // distance	d_phi	d_dis_vert	heading	velocity	directrion	dis_vert	dis_goal
    //      F     F         F          0.4      0.6         0.2         0.6         0.1
    //      F     N         F          0.7      0.6         0.1         0.8         0.1
    //      N     F         F          0.3      0.2         0.9         0.2         0.2
    //      N     N         F          0.6      0.2         0.8         0.4         0.2
    //      F     F         N          0.4      0.6         0.2         0.12        0.1
    //      F     N         N          0.7      0.6         0.1         0.16        0.1
    //      N     F         N          0.3      0.2         0.9         0.04        0.2
    //      N     N         N          0.6      0.2         0.8         0.08        0.2
    vector<vector<int>> menbership_2{
        {F, F, F},
        {F, N, F},
        {N, F, F},
        {N, N, F},
        {F, F, N},
        {F, N, N},
        {N, F, N},
        {N, N, N}};
    vector<vector<double>> defuzzy_rules_2 = {
        {0.6, 0.6,  0.2, 0.7,  0.1},
        {0.7, 0.6,  0.1, 0.9,  0.1},
        {0.3, 0.2,  0.9, 0.4,  0.2},
        {0.4, 0.2,  0.8, 0.6,  0.2},
        {0.72, 0.6, 0.2, 0.12, 0.1},
        {0.84, 0.6, 0.1, 0.16, 0.1},
        {0.36, 0.2, 0.9, 0.04, 0.2},
        {0.48, 0.2, 0.8, 0.08, 0.2},
    };
};

// int main() {
//     FuzzySystem fs;
//     double goal = 10;
//     double d_phi = M_PI / 4;
//     double obstacle = 10;
//     double output = fs.inference(goal, d_phi, obstacle);
//     cout << "Output: " << output << endl;
//     return 0;
// }

#endif // FUZZY_LOGIC_HPP