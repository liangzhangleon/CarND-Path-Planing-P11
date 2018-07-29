#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

float lane_speed(const Vehicle & vehicle, const vector<Vehicle>& sensor_fusion, int lane);

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const vector<Vehicle>& sensor_fusion);
float collision_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const vector<Vehicle>& sensor_fusion);

float calculate_cost(const Vehicle & vehicle, const vector<Vehicle>& sensor_fusion, const vector<Vehicle> & trajectory);


#endif // COST_H
