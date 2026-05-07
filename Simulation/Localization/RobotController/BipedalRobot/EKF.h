#ifndef EKF_H
#define EKF_H

// Call this ONCE in setup()
void init_biped_EKF();

// Call this in the loop() to get the perfect angles
// Added 'motor_penalty' (0.0 to 1.0) to dynamically scale magnetometer trust
void rpy_EKF(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz, double dt, double motor_penalty, 
                      double *roll, double *pitch, double *yaw);

#endif