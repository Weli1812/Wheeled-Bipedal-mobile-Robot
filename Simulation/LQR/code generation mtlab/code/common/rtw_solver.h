#ifndef RTW_SOLVER_H
#define RTW_SOLVER_H

#include <stddef.h>
#include "rtw_continuous.h"
#include "rtwtypes.h"

typedef struct {
  SimTimeStep *simTimeStepPtr;
  time_T **tPtr;
  time_T *stepSizePtr;
  real_T **dXPtr;
  real_T **contStatesPtr;
  int_T *numContStatesPtr;
  int_T *numPeriodicContStatesPtr;
  int_T **periodicContStateIndicesPtr;
  real_T **periodicContStateRangesPtr;
  boolean_T **contStateDisabledPtr;
  const char_T **errorStatusPtr;
  void *rtModelPtr;
  boolean_T isMinorTimeStepWithModeChange;
  boolean_T isContModeFrozen;
  time_T solverStopTime;
  void *solverData;
  const char_T *solverName;
} RTWSolverInfo;

static inline void rtsiSetSimTimeStepPtr(RTWSolverInfo *si, SimTimeStep *p)
{
  si->simTimeStepPtr = p;
}

static inline void rtsiSetSimTimeStep(RTWSolverInfo *si, SimTimeStep step)
{
  if (si->simTimeStepPtr != NULL) {
    *(si->simTimeStepPtr) = step;
  }
}

static inline void rtsiSetTPtr(RTWSolverInfo *si, time_T **p)
{
  si->tPtr = p;
}

static inline time_T rtsiGetT(const RTWSolverInfo *si)
{
  if ((si->tPtr == NULL) || (*(si->tPtr) == NULL)) {
    return 0.0;
  }

  return (*(si->tPtr))[0];
}

static inline void rtsiSetT(RTWSolverInfo *si, time_T t)
{
  if ((si->tPtr != NULL) && (*(si->tPtr) != NULL)) {
    (*(si->tPtr))[0] = t;
  }
}

static inline void rtsiSetStepSizePtr(RTWSolverInfo *si, time_T *p)
{
  si->stepSizePtr = p;
}

static inline time_T rtsiGetStepSize(const RTWSolverInfo *si)
{
  return (si->stepSizePtr != NULL) ? *(si->stepSizePtr) : 0.0;
}

static inline void rtsiSetdXPtr(RTWSolverInfo *si, real_T **p)
{
  si->dXPtr = p;
}

static inline void rtsiSetdX(RTWSolverInfo *si, real_T *dx)
{
  if (si->dXPtr != NULL) {
    *(si->dXPtr) = dx;
  }
}

static inline void rtsiSetContStatesPtr(RTWSolverInfo *si, real_T **p)
{
  si->contStatesPtr = p;
}

static inline real_T *rtsiGetContStates(const RTWSolverInfo *si)
{
  return (si->contStatesPtr != NULL) ? *(si->contStatesPtr) : (real_T *)0;
}

static inline void rtsiSetNumContStatesPtr(RTWSolverInfo *si, int_T *p)
{
  si->numContStatesPtr = p;
}

static inline void rtsiSetNumPeriodicContStatesPtr(RTWSolverInfo *si, int_T *p)
{
  si->numPeriodicContStatesPtr = p;
}

static inline void rtsiSetPeriodicContStateIndicesPtr(RTWSolverInfo *si,
  int_T **p)
{
  si->periodicContStateIndicesPtr = p;
}

static inline void rtsiSetPeriodicContStateRangesPtr(RTWSolverInfo *si,
  real_T **p)
{
  si->periodicContStateRangesPtr = p;
}

static inline void rtsiSetContStateDisabledPtr(RTWSolverInfo *si,
  boolean_T **p)
{
  si->contStateDisabledPtr = p;
}

static inline void rtsiSetErrorStatusPtr(RTWSolverInfo *si, const char_T **p)
{
  si->errorStatusPtr = p;
}

static inline void rtsiSetRTModelPtr(RTWSolverInfo *si, void *p)
{
  si->rtModelPtr = p;
}

static inline void rtsiSetIsMinorTimeStepWithModeChange(RTWSolverInfo *si,
  boolean_T val)
{
  si->isMinorTimeStepWithModeChange = val;
}

static inline void rtsiSetIsContModeFrozen(RTWSolverInfo *si, boolean_T val)
{
  si->isContModeFrozen = val;
}

static inline void rtsiSetSolverStopTime(RTWSolverInfo *si, time_T t)
{
  si->solverStopTime = t;
}

static inline time_T rtsiGetSolverStopTime(const RTWSolverInfo *si)
{
  return si->solverStopTime;
}

static inline boolean_T rtsiIsModeUpdateTimeStep(const RTWSolverInfo *si)
{
  return (si->simTimeStepPtr != NULL) && (*(si->simTimeStepPtr) == MAJOR_TIME_STEP);
}

static inline void rtsiSetSolverData(RTWSolverInfo *si, void *data)
{
  si->solverData = data;
}

static inline void *rtsiGetSolverData(const RTWSolverInfo *si)
{
  return si->solverData;
}

static inline void rtsiSetSolverName(RTWSolverInfo *si, const char_T *name)
{
  si->solverName = name;
}

#endif