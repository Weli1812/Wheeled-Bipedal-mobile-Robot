#ifndef RTW_SOLVER_H
#define RTW_SOLVER_H

#include "rtw_continuous.h"

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
  void *solverData;
  const char_T *solverName;
  time_T t;
  time_T solverStopTime;
  boolean_T isMinorTimeStepWithModeChange;
  boolean_T isContModeFrozen;
} RTWSolverInfo;

#define rtsiSetSimTimeStepPtr(si, ptr)      ((si)->simTimeStepPtr = (ptr))
#define rtsiSetTPtr(si, ptr)                ((si)->tPtr = (ptr))
#define rtsiSetStepSizePtr(si, ptr)         ((si)->stepSizePtr = (ptr))
#define rtsiSetdXPtr(si, ptr)               ((si)->dXPtr = (real_T **)(ptr))
#define rtsiSetContStatesPtr(si, ptr)       ((si)->contStatesPtr = (ptr))
#define rtsiSetNumContStatesPtr(si, ptr)    ((si)->numContStatesPtr = (ptr))
#define rtsiSetNumPeriodicContStatesPtr(si, ptr) ((si)->numPeriodicContStatesPtr = (ptr))
#define rtsiSetPeriodicContStateIndicesPtr(si, ptr) ((si)->periodicContStateIndicesPtr = (ptr))
#define rtsiSetPeriodicContStateRangesPtr(si, ptr)  ((si)->periodicContStateRangesPtr = (ptr))
#define rtsiSetContStateDisabledPtr(si, ptr) ((si)->contStateDisabledPtr = (ptr))
#define rtsiSetErrorStatusPtr(si, ptr)      ((si)->errorStatusPtr = (const char_T **)(ptr))
#define rtsiSetRTModelPtr(si, ptr)          ((si)->rtModelPtr = (ptr))
#define rtsiSetSolverData(si, ptr)          ((si)->solverData = (ptr))
#define rtsiSetSolverName(si, name)         ((si)->solverName = (name))
#define rtsiSetIsMinorTimeStepWithModeChange(si, val) ((si)->isMinorTimeStepWithModeChange = (val))
#define rtsiSetIsContModeFrozen(si, val)    ((si)->isContModeFrozen = (val))
#define rtsiSetSimTimeStep(si, val)         (*(si)->simTimeStepPtr = (val))
#define rtsiSetT(si, val)                   ((si)->t = (val))
#define rtsiSetdX(si, val)                  (*((si)->dXPtr) = (val))
#define rtsiSetSolverStopTime(si, val)      ((si)->solverStopTime = (val))

#define rtsiGetT(si)                        ((*(si)->tPtr)[0])
#define rtsiGetStepSize(si)                 (*(si)->stepSizePtr)
#define rtsiGetContStates(si)               (*(si)->contStatesPtr)
#define rtsiGetSolverData(si)               ((si)->solverData)
#define rtsiGetSolverStopTime(si)           ((si)->solverStopTime)

#endif
