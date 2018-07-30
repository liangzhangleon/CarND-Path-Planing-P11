#include "cost.h"
#include <iterator>
#include <math.h>

float lane_speed(const Vehicle & vehicle, const vector<Vehicle>& sensor_fusion, int lane) {
    //found a vehicle ahead on intended lane
    int min_s = vehicle.s + 50.;
    bool found_vehicle = false;
    Vehicle ahead_vehicle;
    for (vector<Vehicle>::const_iterator it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
        if (it->lane == lane && it->s > vehicle.s && it->s < min_s) {
            min_s = it->s;
            ahead_vehicle = *it;
            found_vehicle = true;
        }
    }
    if(found_vehicle)
        return ahead_vehicle.vel;
    else
        return vehicle.target_speed;
}

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const vector<Vehicle>& sensor_fusion) {
    int lane = trajectory[1].lane;
    int intended_lane = trajectory[1].lane;
    if (trajectory[1].state.compare("PLCL") == 0) {
        intended_lane = lane - 1;
    }
    else if (trajectory[1].state.compare("PLCR") == 0) {
        intended_lane = lane + 1;
    }
    float intended_lane_speed = lane_speed(vehicle, sensor_fusion, intended_lane);
    float trajectory_lane_speed = lane_speed(vehicle, sensor_fusion, lane);
    float cost = (2.*vehicle.target_speed - trajectory_lane_speed - intended_lane_speed)/vehicle.target_speed;
    return cost;
}

float collision_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const vector<Vehicle>& sensor_fusion) {
//only for lane change states
    if(trajectory[1].state.compare("KL") == 0 || trajectory[1].state.compare("PLCL") == 0 || 
       trajectory[1].state.compare("PLCR") == 0)
        return 0.;
    int new_lane = trajectory[1].lane;
    bool collison = false;

 
    //Check if a lane change is possible
    for (vector<Vehicle>::const_iterator it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it) {
        if (it->lane == new_lane) {
           double distance = fabs(it->s - vehicle.s);
           if (distance <= 6.)
               return 1.;
           else if (it->s - vehicle.s > 6. && it->s - vehicle.s < 30)
               if( (it->s + 2.* it->vel/2.24 - vehicle.s - 2.*vehicle.vel/2.24) < 5.)
                   return 1.;
           else if (vehicle.s - it->s > 6. && vehicle.s - it->s < 35){
               //check if vehicle velocity is too low, below certain threshold, the collison probability could be high
               if(vehicle.vel < 0.55 * vehicle.target_speed)
                   return 1.;
               if(vehicle.s + 4.5 * vehicle.vel/2.24 - it->s - 4.5* it->vel/2.24 < 5.)
                   return 1.;
           }
        }
    }
    return 0.;
}

float calculate_cost(const Vehicle & vehicle, const vector<Vehicle>& sensor_fusion, const vector<Vehicle> & trajectory) {
    /*
    Sum collision cost and inefficient cost
    */
    float cost = 0.0;

    //Add additional cost functions here.
    const float COLLISION = pow(10, 4);
    const float EFFICIENCY = pow(10, 1);
    cost = EFFICIENCY * inefficiency_cost(vehicle, trajectory, sensor_fusion);
    cost += COLLISION * collision_cost(vehicle, trajectory, sensor_fusion);
    return cost;
}



