#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <windows.h>
#include <math.h>         
#include "path_planner.h"

uint8_t global_map[MAP_WIDTH * MAP_HEIGHT] = {0};
uint8_t inflated_map[MAP_WIDTH * MAP_HEIGHT] = {0};

typedef enum { STATE_MOVING, STATE_WAITING, STATE_REPLANNING } RobotState;

void ExportTrajectory(const char* filename, TrajectoryPoint* traj, int length) {
    FILE *fp = fopen(filename, "w");
    if (fp) { 
        fprintf(fp, "Time_s,X,Y,Theta_Rads\n"); 
        for (int i = 0; i < length; i++) {
            fprintf(fp, "%.3f,%.3f,%.3f,%.3f\n", traj[i].t, traj[i].x, traj[i].y, traj[i].theta); 
        }
        fclose(fp); 
    }
}

void DrawObstacle(int start_x, int end_x, int start_y, int end_y) {
    for (int y = start_y; y <= end_y; y++) {
        for (int x = start_x; x <= end_x; x++) {
            if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT) {
                global_map[y * MAP_WIDTH + x] = 1;
            }
        }
    }
}

void EraseObstacle(int start_x, int end_x, int start_y, int end_y) {
    for (int y = start_y; y <= end_y; y++) {
        for (int x = start_x; x <= end_x; x++) {
            if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT) {
                global_map[y * MAP_WIDTH + x] = 0; 
            }
        }
    }
}

int main() {
    printf("--- Bipedal PC Test: LIVE HARDWARE & SIMULATION ---\n");
    remove("path_replan.csv"); 

    DrawObstacle(10, 12, 15, 25); 
    DrawObstacle(40, 45, 30, 37); 
    DrawObstacle(35, 40, 10, 15); 
    DrawObstacle(15, 20, 37, 40); 
    DrawObstacle(22, 27, 22, 27);
    
    InitKinematics(0.5f, 0.2f); 
    
    Pose start = {0.0f, 0.0f, 0.0f}; 
    Pose goal = {45.0f, 0.0f, 0.0f}; 
    Pose temp_path[MAX_PATH_LENGTH];
    
    InflateMap(global_map, inflated_map, 3); 
    
    TrajectoryPoint traj_A[2000]; 
    TrajectoryPoint traj_B[2000]; 
    TrajectoryPoint* active_traj = traj_A;
    int total_path_points = 0;
    
    Pose approach_pose;
    approach_pose.x = goal.x - 5.0f * cosf(goal.theta);
    approach_pose.y = goal.y - 5.0f * sinf(goal.theta);
    approach_pose.theta = goal.theta;
    
    int waypoints = PlanKinematicPath(inflated_map, start, approach_pose, temp_path);
    if (waypoints > 0) {
        waypoints--; 
        
        // LAUNCH GATE (Initial Path)
        if (waypoints > 3) {
            for (int step = 1; step <= 3; step++) {
                temp_path[step].x = temp_path[0].x + (step * 0.5f * cosf(temp_path[0].theta));
                temp_path[step].y = temp_path[0].y + (step * 0.5f * sinf(temp_path[0].theta));
                temp_path[step].theta = temp_path[0].theta;
            }
        }
        
        // APPROACH GATE
        for(int i = 0; i <= 5; i++) {
            temp_path[waypoints].x = approach_pose.x + (i * 1.0f * cosf(goal.theta));
            temp_path[waypoints].y = approach_pose.y + (i * 1.0f * sinf(goal.theta));
            temp_path[waypoints].theta = goal.theta;
            waypoints++;
        }
        
        SmoothPath(temp_path, waypoints, 0.2f, 0.2f, 0.0001f, inflated_map);
        total_path_points = GenerateConstantVelocityTrajectory(temp_path, waypoints, 1.0f, 0.05f, active_traj, 2000);
        FilterTrajectory(active_traj, total_path_points, 30, 0.05f);
        
        ExportTrajectory("path_initial.csv", active_traj, total_path_points);
    } else {
        printf("FAILED to find initial path!\n");
        return 1;
    }

    bool live_hardware = true;
    const char *port_name = "\\\\.\\COM7"; 
    HANDLE hSerial = CreateFileA(port_name, GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    
    if (hSerial == INVALID_HANDLE_VALUE) { 
        printf("[WARNING] Could not find ESP32 on %s.\n", port_name); 
        printf("[SYSTEM] Switching to PC Simulation (Mock Mode)...\n\n");
        live_hardware = false; 
    } else {
        DCB dcbSerialParams = {0}; 
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams); 
        GetCommState(hSerial, &dcbSerialParams);
        dcbSerialParams.BaudRate = CBR_115200; 
        dcbSerialParams.ByteSize = 8; 
        SetCommState(hSerial, &dcbSerialParams);
        COMMTIMEOUTS timeouts = {0}; 
        timeouts.ReadTotalTimeoutConstant = 50; 
        SetCommTimeouts(hSerial, &timeouts);
        printf("[SYSTEM] Connected to ESP32! System Armed.\nWaiting for sensor data to begin drive...\n\n");
    }

    FILE *log_fp = fopen("telemetry.csv", "w");
    if (log_fp) fprintf(log_fp, "Time_s,X,Y,Theta,State,SensorDist\n");

    RobotState current_state = STATE_MOVING;
    float t_sim = 0.0f, wait_start_time = 0.0f, latest_r = 250.0f;
    int current_path_idx = 0;
    char rx_buffer[128]; int rx_index = 0; char rx_byte; DWORD bytes_read;
    bool packet_ready = false;

    static float log_zU = 0, log_zL = 0, log_xF = 0;
    static int log_th_start = 0, log_th_end = 0;
    static float last_print_time = 0.0f;

    while (current_path_idx < total_path_points) {
        
        if (live_hardware) {
            if (ReadFile(hSerial, &rx_byte, 1, &bytes_read, NULL) && bytes_read > 0) {
                if (rx_byte == '\n' || rx_byte == '\r') {
                    rx_buffer[rx_index] = '\0';
                    if (strstr(rx_buffer, "DATA ->") != NULL) {
                        float zU, zL, xF, r_dist; int th_start, th_end;
                        if (sscanf(rx_buffer, "DATA -> US:%f, L:%f, F:%f, r:%f, t_start:%d, t_end:%d", &zU, &zL, &xF, &r_dist, &th_start, &th_end) == 6) {
                            latest_r = r_dist / 20.0f;
                            log_zU = zU; log_zL = zL; log_xF = xF; 
                            log_th_start = th_start; log_th_end = th_end;
                            packet_ready = true;
                        }
                    }
                    rx_index = 0;
                } else if (rx_index < 127) { rx_buffer[rx_index++] = rx_byte; }
            }
        } else {
            if (t_sim >= 8.0f && t_sim < 9.0f) latest_r = 6.0f; 
            else latest_r = 250.0f; 
            packet_ready = true;
        }

        if (packet_ready) {
            t_sim += 0.05f; 
            packet_ready = false;

            if (t_sim - last_print_time >= 0.2f) {
                if (live_hardware) {
                    printf("[t=%5.2f] Grid(X:%4.1f, Y:%4.1f) | Fused Target Dist: %5.1f cm | (US: %5.1f, LiDAR: %5.1f)\n", 
                           t_sim, active_traj[current_path_idx].x, active_traj[current_path_idx].y, log_xF, log_zU, log_zL);
                } else {
                    printf("[t=%5.2f] Grid(X:%4.1f, Y:%4.1f) | Simulation Sensor Dist: %5.1f cm\n", 
                           t_sim, active_traj[current_path_idx].x, active_traj[current_path_idx].y, latest_r * 20.0f);
                }
                last_print_time = t_sim;
            }

            float stopping_distance_meters = 1.5f; 
            float stop_grids = stopping_distance_meters / 0.2f; 

            if (latest_r < stop_grids && current_state == STATE_MOVING) { 
                current_state = STATE_WAITING;
                wait_start_time = t_sim;
                
                printf("\n==================================================\n");
                printf("[!] OBSTACLE DETECTED! Braking at %.0fcm. Initiating 2-second patience wait...\n", latest_r * 20.0f);
                printf("==================================================\n\n");
            }

            if (log_fp) {
                fprintf(log_fp, "%.3f,%.3f,%.3f,%.3f,%d,%.2f\n", 
                        t_sim, active_traj[current_path_idx].x, active_traj[current_path_idx].y, 
                        active_traj[current_path_idx].theta, current_state, latest_r);
            }

            switch (current_state) {
                case STATE_MOVING: 
                    current_path_idx++; 
                    break;
                    
                case STATE_WAITING:
                    if (latest_r >= stop_grids) {
                        printf("\n[t=%.2f] OBSTACLE CLEARED! Resuming current path...\n\n", t_sim);
                        current_state = STATE_MOVING;
                    } 
                    else if ((t_sim - wait_start_time) >= 2.0f) { 
                        current_state = STATE_REPLANNING; 
                        printf("\n[t=%.2f] Obstacle persists. Commencing Dubins Replan...\n", t_sim); 
                    }
                    break;
                    
                case STATE_REPLANNING:
                    int ox = (int)active_traj[current_path_idx].x + (int)(latest_r * cos(active_traj[current_path_idx].theta));
                    int oy = (int)active_traj[current_path_idx].y + (int)(latest_r * sin(active_traj[current_path_idx].theta));
                    
                    DrawObstacle(ox-1, ox+1, oy-1, oy+1); 
                    InflateMap(global_map, inflated_map, 3);

                    Pose current_pose = {active_traj[current_path_idx].x, active_traj[current_path_idx].y, active_traj[current_path_idx].theta};
                    
                    int new_wp = PlanKinematicPath(inflated_map, current_pose, approach_pose, temp_path);
                    
                    EraseObstacle(ox-1, ox+1, oy-1, oy+1);
                    
                    if (new_wp > 0) {
                        new_wp--; 
                        
                        // FIX: THE LAUNCH GATE (Replan)
                        // Overwrite the first 3 A* points to project perfectly straight ahead. 
                        // This prevents A* from generating an instant 90-degree lateral step.
                        if (new_wp > 3) {
                            for (int step = 1; step <= 3; step++) {
                                temp_path[step].x = temp_path[0].x + (step * 0.5f * cosf(temp_path[0].theta));
                                temp_path[step].y = temp_path[0].y + (step * 0.5f * sinf(temp_path[0].theta));
                                temp_path[step].theta = temp_path[0].theta;
                            }
                        }

                        // APPROACH GATE
                        for(int i = 0; i <= 5; i++) {
                            temp_path[new_wp].x = approach_pose.x + (i * 1.0f * cosf(goal.theta));
                            temp_path[new_wp].y = approach_pose.y + (i * 1.0f * sinf(goal.theta));
                            temp_path[new_wp].theta = goal.theta;
                            new_wp++;
                        }
                        
                        SmoothPath(temp_path, new_wp, 0.2f, 0.2f, 0.0001f, inflated_map);
                        total_path_points = GenerateConstantVelocityTrajectory(temp_path, new_wp, 1.0f, 0.05f, traj_B, 2000);
                        FilterTrajectory(traj_B, total_path_points, 30, 0.05f);
                        
                        active_traj = traj_B; 
                        current_path_idx = 0; 
                        current_state = STATE_MOVING;
                        ExportTrajectory("path_replan.csv", traj_B, total_path_points);
                        printf("[t=%.2f] REPLAN SUCCESS! Drive sequence engaged.\n\n", t_sim);
                    } else {
                        printf("[t=%.2f] Replan Failed! Waiting...\n", t_sim);
                        current_state = STATE_WAITING; 
                        wait_start_time = t_sim;
                    }
                    latest_r = 250.0f; 
                    break;
            }
        }
    }

    if (log_fp) fclose(log_fp); 
    if (live_hardware) CloseHandle(hSerial);
    printf("\nGoal Reached! Simulation Complete.\n");
    return 0;
}