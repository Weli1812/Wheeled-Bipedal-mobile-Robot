#include "path_planner.h"
#include <math.h>

#define GRID_RES_M 0.2f 

typedef struct { Pose pose; float g_cost; float f_cost; int parent_idx; bool is_open; } Node;

static Node node_pool[MAX_NODES];
static bool closed_list[MAP_WIDTH][MAP_HEIGHT][THETA_BINS];

const float step_distance = 1.5f; 
const float primitive_costs[3] = {1.05f, 1.0f, 1.05f}; 

float steer_angles[3] = {0.0f, 0.0f, 0.0f}; 

static float robot_velocity_m_s = 0.2f; 
static float min_turn_rad_m = 0.5f;
static float max_yaw_rate_rad_s = 0.0f;

void InitKinematics(float min_turning_radius_m, float speed_m_s) {
    robot_velocity_m_s = speed_m_s;
    min_turn_rad_m = min_turning_radius_m;
    
    float step_distance_m = step_distance * GRID_RES_M; 
    float max_steer_angle = step_distance_m / min_turn_rad_m; 
    
    steer_angles[0] = -max_steer_angle;
    steer_angles[1] = 0.0f;
    steer_angles[2] = max_steer_angle;
    
    max_yaw_rate_rad_s = robot_velocity_m_s / min_turn_rad_m;
}

static inline float NormalizeAngle(float angle) { while (angle < 0) angle += 6.2831853f; while (angle >= 6.2831853f) angle -= 6.2831853f; return angle; }
static inline int GetThetaBin(float angle) { int bin = (int)(NormalizeAngle(angle) / 0.785398f); if (bin >= THETA_BINS) bin = 0; return bin; }
static inline bool IsValid(const uint8_t* map, float x, float y) {
    int ix = (int)roundf(x); int iy = (int)roundf(y);
    if (ix < 0 || ix >= MAP_WIDTH || iy < 0 || iy >= MAP_HEIGHT) return false;
    if (map[iy * MAP_WIDTH + ix] == 1) return false; 
    return true;
}

static inline float CalculateHeuristic(Pose current, Pose goal) { 
    float dx = goal.x - current.x; 
    float dy = goal.y - current.y; 
    float dist = sqrtf(dx * dx + dy * dy); 
    
    if (dist < 0.1f) return 0.0f;

    float angle_to_goal = atan2f(dy, dx);
    
    float diff_start = current.theta - angle_to_goal;
    while (diff_start > 3.14159f) diff_start -= 6.28318f;
    while (diff_start < -3.14159f) diff_start += 6.28318f;
    
    float diff_end = goal.theta - angle_to_goal;
    while (diff_end > 3.14159f) diff_end -= 6.28318f;
    while (diff_end < -3.14159f) diff_end += 6.28318f;
    
    float turning_radius_grids = min_turn_rad_m / GRID_RES_M;
    float turn_penalty = turning_radius_grids * (fabsf(diff_start) + fabsf(diff_end));
    
    return dist + turn_penalty; 
}

void InflateMap(const uint8_t* original_map, uint8_t* inflated_map, int inflation_cells) {
    for (int i = 0; i < MAP_WIDTH * MAP_HEIGHT; i++) inflated_map[i] = 0;
    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            if (original_map[y * MAP_WIDTH + x] == 1) {
                for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
                        if (dx*dx + dy*dy <= inflation_cells*inflation_cells) {
                            int nx = x + dx; int ny = y + dy;
                            if (nx >= 0 && nx < MAP_WIDTH && ny >= 0 && ny < MAP_HEIGHT) inflated_map[ny * MAP_WIDTH + nx] = 1;
                        }
                    }
                }
            }
        }
    }
}

int PlanKinematicPath(const uint8_t* map, Pose start, Pose goal, Pose* path_out) {
    for (int x = 0; x < MAP_WIDTH; x++) for (int y = 0; y < MAP_HEIGHT; y++) for (int t = 0; t < THETA_BINS; t++) closed_list[x][y][t] = false;
    int active_nodes = 0;
    node_pool[0] = (Node){start, 0, CalculateHeuristic(start, goal), -1, true};
    active_nodes++;
    int best_node_idx = -1; bool path_found = false;

    while (active_nodes < MAX_NODES - 5) {
        int current_idx = -1; float min_f = 999999.0f;
        for (int i = 0; i < active_nodes; i++) { if (node_pool[i].is_open && node_pool[i].f_cost < min_f) { min_f = node_pool[i].f_cost; current_idx = i; } }
        if (current_idx == -1) break; 
        Node* current = &node_pool[current_idx]; current->is_open = false;
        
        float dx = goal.x - current->pose.x;
        float dy = goal.y - current->pose.y;
        float dist = sqrtf(dx*dx + dy*dy);

        float diff_theta = current->pose.theta - goal.theta;
        while (diff_theta > 3.14159f) diff_theta -= 6.28318f;
        while (diff_theta < -3.14159f) diff_theta += 6.28318f;

        if (dist < 2.0f && fabsf(diff_theta) < 0.4f) { 
            best_node_idx = current_idx; path_found = true; break; 
        }

        int cx = (int)roundf(current->pose.x), cy = (int)roundf(current->pose.y), c_theta = GetThetaBin(current->pose.theta);
        closed_list[cx][cy][c_theta] = true;

        for (int i = 0; i < 3; i++) {
            Pose next_pose;
            next_pose.theta = NormalizeAngle(current->pose.theta + steer_angles[i]);
            next_pose.x = current->pose.x + step_distance * cosf(next_pose.theta);
            next_pose.y = current->pose.y + step_distance * sinf(next_pose.theta);
            if (!IsValid(map, next_pose.x, next_pose.y)) continue;
            
            int nx = (int)roundf(next_pose.x), ny = (int)roundf(next_pose.y), n_theta = GetThetaBin(next_pose.theta);
            if (closed_list[nx][ny][n_theta]) continue;
            
            float tentative_g = current->g_cost + primitive_costs[i];
            int new_idx = active_nodes;
            node_pool[new_idx] = (Node){next_pose, tentative_g, tentative_g + CalculateHeuristic(next_pose, goal), current_idx, true};
            active_nodes++;
        }
    }
    if (!path_found) return 0;
    
    int current_trace = best_node_idx, path_length = 0;
    while (current_trace != -1 && path_length < MAX_PATH_LENGTH - 2) { 
        path_out[path_length++] = node_pool[current_trace].pose; 
        current_trace = node_pool[current_trace].parent_idx; 
    }
    for (int i = 0; i < path_length / 2; i++) { Pose temp = path_out[i]; path_out[i] = path_out[path_length - 1 - i]; path_out[path_length - 1 - i] = temp; }
    
    return path_length;
}

void SmoothPath(Pose* path, int length, float data_weight, float smooth_weight, float tolerance, const uint8_t* map) {
    if (length <= 6) return; 
    Pose original_path[MAX_PATH_LENGTH];
    for(int i = 0; i < length; i++) original_path[i] = path[i];
    
    float change = tolerance; int iterations = 0;
    while(change >= tolerance && iterations < 1000) {
        change = 0.0f;
        // FIX: Protect the first 2 points (Launch Gate) AND last 5 points (Approach Gate)
        for(int i = 2; i < length - 5; i++) { 
            float aux_x = path[i].x, aux_y = path[i].y;
            float new_x = path[i].x + data_weight * (original_path[i].x - path[i].x) + smooth_weight * (path[i-1].x + path[i+1].x - 2.0f * path[i].x);
            float new_y = path[i].y + data_weight * (original_path[i].y - path[i].y) + smooth_weight * (path[i-1].y + path[i+1].y - 2.0f * path[i].y);
            
            if (IsValid(map, new_x, new_y)) {
                path[i].x = new_x; path[i].y = new_y;
                change += fabs(aux_x - path[i].x) + fabs(aux_y - path[i].y);
            }
        }
        iterations++;
    }
    
    for(int i = 0; i < length - 1; i++) {
        path[i].theta = atan2f(path[i+1].y - path[i].y, path[i+1].x - path[i].x);
    }
    path[length-1].theta = path[length-2].theta; 
}

int GenerateConstantVelocityTrajectory(Pose* smooth_path, int path_length, float grid_velocity, float dt, TrajectoryPoint* traj_out, int max_traj_length) {
    if (path_length < 2) return 0;
    float S[MAX_PATH_LENGTH] = {0};
    for (int i = 1; i < path_length; i++) S[i] = S[i-1] + sqrtf(pow(smooth_path[i].x - smooth_path[i-1].x, 2) + pow(smooth_path[i].y - smooth_path[i-1].y, 2));
    float total_time = S[path_length - 1] / grid_velocity;
    int total_samples = (int)(total_time / dt) + 1;
    if (total_samples > max_traj_length) total_samples = max_traj_length; 
    
    int current_segment = 0;
    for (int step = 0; step < total_samples; step++) {
        float current_t = step * dt, target_s = current_t * grid_velocity; 
        while (current_segment < path_length - 2 && S[current_segment + 1] < target_s) current_segment++;
        float segment_length = S[current_segment + 1] - S[current_segment];
        float alpha = (segment_length > 0.0001f) ? (target_s - S[current_segment]) / segment_length : 0.0f;
        traj_out[step].x = smooth_path[current_segment].x + alpha * (smooth_path[current_segment+1].x - smooth_path[current_segment].x);
        traj_out[step].y = smooth_path[current_segment].y + alpha * (smooth_path[current_segment+1].y - smooth_path[current_segment].y);
        
        float diff = smooth_path[current_segment+1].theta - smooth_path[current_segment].theta;
        while (diff > 3.14159f) diff -= 6.28318f; while (diff < -3.14159f) diff += 6.28318f;
        traj_out[step].theta = smooth_path[current_segment].theta + alpha * diff;
        traj_out[step].t = current_t;
    }
    return total_samples;
}

void FilterTrajectory(TrajectoryPoint* traj, int length, int max_window, float dt) {
    if (length <= 40) return;
    static TrajectoryPoint temp[2000]; 
    for(int i = 0; i < length; i++) temp[i] = traj[i];

    // FIX: Reduced to 2! This unlocks the filter, allowing it to instantly
    // sand down the jagged A* steps right at the moment of replanning.
    int protect_start = 3; 
    int protect_end = 3; 

    for(int i = protect_start; i < length - protect_end; i++) {
        int window = max_window;
        if (i - protect_start < window) window = i - protect_start;
        if ((length - protect_end) - 1 - i < window) window = (length - protect_end) - 1 - i;

        float sum_x = 0.0f, sum_y = 0.0f;
        for(int j = -window; j <= window; j++) {
            sum_x += temp[i+j].x;
            sum_y += temp[i+j].y;
        }
        float count = (float)(2 * window + 1);
        traj[i].x = sum_x / count;
        traj[i].y = sum_y / count;
    }

    for(int i = 0; i < length - 1; i++) {
        traj[i].theta = atan2f(traj[i+1].y - traj[i].y, traj[i+1].x - traj[i].x);
    }
    traj[length-1].theta = traj[length-2].theta;

    for(int i = 0; i < length; i++) temp[i].theta = traj[i].theta;

    for(int i = protect_start; i < length - protect_end; i++) {
        int window = max_window;
        if (i - protect_start < window) window = i - protect_start;
        if ((length - protect_end) - 1 - i < window) window = (length - protect_end) - 1 - i;

        float sum_sin = 0.0f, sum_cos = 0.0f;
        for(int j = -window; j <= window; j++) {
            sum_sin += sinf(temp[i+j].theta);
            sum_cos += cosf(temp[i+j].theta);
        }
        traj[i].theta = atan2f(sum_sin, sum_cos); 
    }
    
    // STRICT BIPEDAL KINEMATIC CLAMPING
    float max_dtheta = max_yaw_rate_rad_s * dt; 
    
    for(int i = 1; i < length; i++) {
        float dtheta = traj[i].theta - traj[i-1].theta;
        
        while (dtheta > 3.14159f) dtheta -= 6.28318f;
        while (dtheta < -3.14159f) dtheta += 6.28318f;
        
        if (dtheta > max_dtheta) {
            traj[i].theta = traj[i-1].theta + max_dtheta;
        } else if (dtheta < -max_dtheta) {
            traj[i].theta = traj[i-1].theta - max_dtheta;
        }
        
        while (traj[i].theta > 3.14159f) traj[i].theta -= 6.28318f;
        while (traj[i].theta < -3.14159f) traj[i].theta += 6.28318f;
    }
}