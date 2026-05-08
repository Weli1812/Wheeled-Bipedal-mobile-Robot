#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H
#include <stdint.h>
#include <stdbool.h>

#define MAP_WIDTH  50
#define MAP_HEIGHT 50
#define THETA_BINS 8     
#define MAX_PATH_LENGTH 250 
#define MAX_NODES 15000  

typedef struct { float x; float y; float theta; } Pose;
typedef struct { float x; float y; float theta; float t; } TrajectoryPoint;

// Initialization function for bipedal kinematic constraints
void InitKinematics(float min_turning_radius_m, float speed_m_s);

void InflateMap(const uint8_t* original_map, uint8_t* inflated_map, int inflation_cells);
int PlanKinematicPath(const uint8_t* map, Pose start, Pose goal, Pose* path_out);
void SmoothPath(Pose* path, int length, float data_weight, float smooth_weight, float tolerance, const uint8_t* map);
int GenerateConstantVelocityTrajectory(Pose* smooth_path, int path_length, float grid_velocity, float dt, TrajectoryPoint* traj_out, int max_traj_length);

// Added dt to the signature for anti-snap clamping
void FilterTrajectory(TrajectoryPoint* traj, int length, int max_window, float dt);

#endif