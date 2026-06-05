#ifndef LQR_TRAJ2_ESP32_H
#define LQR_TRAJ2_ESP32_H

#include <stdint.h>
#include "../Traj2_grt_rtw/Traj2.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { LQR_TRAJ2_REF_LEN = 1012 };

typedef struct {
  double v_cmd;
  double w_cmd;
  boolean_T reached;
  boolean_T dirr;
  boolean_T dirl;
  double pwmr;
  double pwml;
} LqrTraj2Output;

void lqr_traj2_init(const double *refPathN3,
                    int numPoints,
                    const double *vdRef,
                    const double *wdRef,
                    const double initialPose[3]);

void lqr_traj2_step(double vrActual,
                    double vlActual,
                    const double currentPose[3],
                    uint32_t stepIndex0,
                    LqrTraj2Output *out);

void lqr_traj2_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
