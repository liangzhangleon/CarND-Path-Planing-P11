#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
    //this class is adapted from Vehicle class in lesson 4.22 behavior planning
public:

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"PLCR", 1}, {"LCR", 1}};

  int lane;
  int s;
  float vel;
  float accel;
  string state;

  int preferred_buffer; // impacts "keep lane" behavior.
  float target_speed;
  float max_acceleration;

  Vehicle();
  Vehicle(int lane, float s, float v, float a, string state="KL");

  virtual ~Vehicle();

  //state transition function
  vector<Vehicle> choose_next_state(vector<Vehicle> sensor_fusion);
  vector<string> successor_states();

  //Generate simple trajectory for choose next state
  vector<Vehicle> generate_trajectory(string state, vector<Vehicle> sensor_fusion);
  vector<Vehicle> keep_lane_trajectory(vector<Vehicle> sensor_fusion);
  vector<Vehicle> prep_lane_change_trajectory(string state, vector<Vehicle> sensor_fusion);
  vector<Vehicle> lane_change_trajectory(string state, vector<Vehicle> sensor_fusion);

  //helper functions for generate trajectory
  vector<float> get_kinematics(vector<Vehicle> sensor_fusion, int lane);
  float compute_position_at(int t);
  bool get_vehicle_behind(vector<Vehicle> sensor_fusion, int lane, Vehicle & rVehicle);
  bool get_vehicle_ahead(vector<Vehicle> sensor_fusion, int lane, Vehicle & rVehicle);

};

#endif
