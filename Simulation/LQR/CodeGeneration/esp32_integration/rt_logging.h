#ifndef RT_LOGGING_H
#define RT_LOGGING_H

#include "rtwtypes.h"

typedef struct {
  void *loggingInterval;
} RTWLogInfo;

typedef struct {
  int dummy;
} LogVar;

#define SS_DOUBLE 0
#define NO_LOGVALDIMS NULL

static inline void rtliSetLogXSignalInfo(RTWLogInfo *info, void *val)
{
  (void)info;
  (void)val;
}

static inline void rtliSetLogXSignalPtrs(RTWLogInfo *info, void *val)
{
  (void)info;
  (void)val;
}

static inline void rtliSetLogT(RTWLogInfo *info, const char *val)
{
  (void)info;
  (void)val;
}

static inline void rtliSetLogX(RTWLogInfo *info, const char *val)
{
  (void)info;
  (void)val;
}

static inline void rtliSetLogXFinal(RTWLogInfo *info, const char *val)
{
  (void)info;
  (void)val;
}

static inline void rtliSetLogVarNameModifier(RTWLogInfo *info, const char *val)
{
  (void)info;
  (void)val;
}

static inline void rtliSetLogFormat(RTWLogInfo *info, int_T val)
{
  (void)info;
  (void)val;
}

static inline void rtliSetLogMaxRows(RTWLogInfo *info, int_T val)
{
  (void)info;
  (void)val;
}

static inline void rtliSetLogDecimation(RTWLogInfo *info, int_T val)
{
  (void)info;
  (void)val;
}

static inline void rtliSetLogY(RTWLogInfo *info, const char *val)
{
  (void)info;
  (void)val;
}

static inline void rtliSetLogYSignalInfo(RTWLogInfo *info, void *val)
{
  (void)info;
  (void)val;
}

static inline void rtliSetLogYSignalPtrs(RTWLogInfo *info, void *val)
{
  (void)info;
  (void)val;
}

static inline int_T rt_StartDataLoggingWithStartTime(RTWLogInfo *info,
                                                      time_T start,
                                                      time_T final,
                                                      time_T step,
                                                      const char_T **err)
{
  (void)info;
  (void)start;
  (void)final;
  (void)step;
  (void)err;
  return 1;
}

static inline void *rt_CreateLogVar(RTWLogInfo *info,
                                    time_T start,
                                    time_T final,
                                    time_T step,
                                    const char_T **err,
                                    const char_T *name,
                                    int_T dataType,
                                    int_T complex,
                                    int_T frameData,
                                    int_T dimension,
                                    int_T row,
                                    int_T col,
                                    const int_T *dims,
                                    const void *valDims,
                                    const void *currSigDims,
                                    const void *currSigDimsSize,
                                    int_T maxRows,
                                    int_T decimation,
                                    real_T sampleTime,
                                    int_T hasName)
{
  static LogVar s_dummy;
  (void)info;
  (void)start;
  (void)final;
  (void)step;
  (void)err;
  (void)name;
  (void)dataType;
  (void)complex;
  (void)frameData;
  (void)dimension;
  (void)row;
  (void)col;
  (void)dims;
  (void)valDims;
  (void)currSigDims;
  (void)currSigDimsSize;
  (void)maxRows;
  (void)decimation;
  (void)sampleTime;
  (void)hasName;
  return &s_dummy;
}

static inline void rt_UpdateLogVar(LogVar *var, const void *data, int_T tid)
{
  (void)var;
  (void)data;
  (void)tid;
}

static inline void rt_UpdateTXYLogVars(RTWLogInfo *info, time_T *t)
{
  (void)info;
  (void)t;
}

#endif
