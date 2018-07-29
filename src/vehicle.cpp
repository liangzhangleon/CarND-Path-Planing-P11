#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {

    this->lane = lane;
    this->s = s;
    this->vel = v;
    this->accel = a;
    this->state = state;

    preferred_buffer = 6;
    max_acceleration = 5;
    target_speed = 50.0/2.24;

}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(vector<Vehicle> sensor_fusion) {
    /*
    INPUT:  Sensor fusion result, current vehicle state
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.
    */
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, sensor_fusion);
        if (trajectory.size() != 0) {
            cost = calculate_cost(*this, sensor_fusion, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    return final_trajectories[best_idx];
}


vector<string> Vehicle::successor_states() {
    /*
     * Provides the possible next states given the current state for the FSM
     * discussed in the course, with the exception that lane changes happen
     * instantaneously, so LCL and LCR can only transition back to KL.
     */
    vector<string> states;
    states.push_back("KL");
    if (state.compare("KL") == 0){
        if (lane == 1) {
            states.push_back("PLCL");
            states.push_back("PLCR");
        }
        else if (lane == 0)
            states.push_back("PLCR");
        else if (lane == 2)
            states.push_back("PLCL");
    }
    else if (state.compare("PLCL") == 0) {
            if (lane != 0) {
                states.push_back("PLCL");
                states.push_back("LCL");
            }
    }
    else if (state.compare("PLCR") == 0) {
            if (lane != 2) {
                states.push_back("PLCR");
                states.push_back("LCR");
            }
    }
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, vector<Vehicle> sensor_fusion) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(sensor_fusion);
    }
    else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, sensor_fusion);
    }
    else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, sensor_fusion);
    }
    return trajectory;
}

vector<float> Vehicle::get_kinematics(vector<Vehicle> sensor_fusion, int lane) {
    /*
     * Gets next timestep kinematics (position, velocity, acceleration)
     * for a given lane. Tries to choose the maximum velocity and acceleration,
     * given other vehicle positions and accel/velocity constraints.
     */
    float max_velocity_accel_limit = this->max_acceleration + this->vel;
    float new_position;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(sensor_fusion, lane, vehicle_ahead)) {

        if (get_vehicle_behind(sensor_fusion, lane, vehicle_behind)) {
            new_velocity = vehicle_ahead.vel; //must travel at the speed of traffic, regardless of preferred buffer
        }
        else {
            float max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.vel;
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        }
    } else {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }

    new_accel = new_velocity - this->vel; //Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity + new_accel / 2.0;
    return{new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::keep_lane_trajectory(vector<Vehicle> sensor_fusion) {

    vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->vel, this->accel, state)};
    vector<float> kinematics = get_kinematics(sensor_fusion, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;
}


//Generate a trajectory preparing for a lane change.
vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, vector<Vehicle> sensor_fusion) {

    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->vel, this->accel, this->state)};
    vector<float> curr_lane_new_kinematics = get_kinematics(sensor_fusion, this->lane);

    if (get_vehicle_behind(sensor_fusion, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];

    } else {
        vector<float> best_kinematics;
        vector<float> next_lane_new_kinematics = get_kinematics(sensor_fusion, new_lane);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    return trajectory;
}


vector<Vehicle> Vehicle::lane_change_trajectory(string state, vector<Vehicle> sensor_fusion) {

    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Collison will be considered later on in the cost function
    trajectory.push_back(Vehicle(this->lane, this->s, this->vel, this->accel, this->state));
    vector<float> kinematics = get_kinematics(sensor_fusion, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    return trajectory;
}



float Vehicle::compute_position_at(int t) {
    return this->s + this->vel*t + this->accel*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(vector<Vehicle> sensor_fusion, int lane, Vehicle & rVehicle) {
    //Returns a true if a vehicle is found behind the current vehicle, false otherwise.
    //The passed reference rVehicle is updated if a vehicle is found.
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (vector<Vehicle>::iterator it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
        temp_vehicle = *it;
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(vector<Vehicle> sensor_fusion, int lane, Vehicle & rVehicle) {
    //Returns a true if a vehicle is found ahead of the current vehicle, false otherwise.
    //The passed reference rVehicle is updated if a vehicle is found.
    int min_s = this->s + 40;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (vector<Vehicle>::iterator it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
        temp_vehicle = *it;
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}
