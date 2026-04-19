#ifndef RT_LOGGING_H
#define RT_LOGGING_H

#include "rtwtypes.h"

typedef struct {
  const void *loggingInterval;
} RTWLogInfo;

typedef void LogVar;

#ifndef NO_LOGVALDIMS
#define NO_LOGVALDIMS ((const int_T *)0)
#endif

static inline void rtliSetLogXSignalInfo(RTWLogInfo *li, const void *v)
{
  (void)li;
  (void)v;
}

static inline void rtliSetLogXSignalPtrs(RTWLogInfo *li, const void *v)
{
  (void)li;
  (void)v;
}

static inline void rtliSetLogT(RTWLogInfo *li, const char_T *v)
{
  (void)li;
  (void)v;
}

static inline void rtliSetLogX(RTWLogInfo *li, const char_T *v)
{
  (void)li;
  (void)v;
}

static inline void rtliSetLogXFinal(RTWLogInfo *li, const char_T *v)
{
  (void)li;
  (void)v;
}

static inline void rtliSetLogVarNameModifier(RTWLogInfo *li, const char_T *v)
{
  (void)li;
  (void)v;
}

static inline void rtliSetLogFormat(RTWLogInfo *li, int_T v)
{
  (void)li;
  (void)v;
}

static inline void rtliSetLogMaxRows(RTWLogInfo *li, int_T v)
{
  (void)li;
  (void)v;
}

static inline void rtliSetLogDecimation(RTWLogInfo *li, int_T v)
{
  (void)li;
  (void)v;
}

static inline void rtliSetLogY(RTWLogInfo *li, const char_T *v)
{
  (void)li;
  (void)v;
}

static inline void rtliSetLogYSignalInfo(RTWLogInfo *li, const void *v)
{
  (void)li;
  (void)v;
}

static inline void rtliSetLogYSignalPtrs(RTWLogInfo *li, const void *v)
{
  (void)li;
  (void)v;
}

static inline void rt_StartDataLoggingWithStartTime(
  RTWLogInfo *li,
  time_T startTime,
  time_T finalTime,
  time_T stepSize,
  const char_T **errStatus)
{
  (void)li;
  (void)startTime;
  (void)finalTime;
  (void)stepSize;
  (void)errStatus;
}

static inline void rt_UpdateTXYLogVars(RTWLogInfo *li, const time_T *t)
{
  (void)li;
  (void)t;
}

static inline void rt_UpdateLogVar(LogVar *v, const void *data, int_T rowIdx)
{
  (void)v;
  (void)data;
  (void)rowIdx;
}

static inline void *rt_CreateLogVar(
  RTWLogInfo *li,
  time_T startTime,
  time_T finalTime,
  time_T stepSize,
  const char_T **errStatus,
  const char_T *varName,
  int_T dataType,
  int_T isComplex,
  int_T frameData,
  int_T matrixFormat,
  int_T numDims,
  int_T dimsMode,
  const int_T *dims,
  const int_T *dimsValDims,
  void *currSigDims,
  void *currSigDimsSize,
  int_T maxRows,
  int_T decimation,
  time_T sampleTime,
  int_T append)
{
  static int_T dummy;

  (void)li;
  (void)startTime;
  (void)finalTime;
  (void)stepSize;
  (void)errStatus;
  (void)varName;
  (void)dataType;
  (void)isComplex;
  (void)frameData;
  (void)matrixFormat;
  (void)numDims;
  (void)dimsMode;
  (void)dims;
  (void)dimsValDims;
  (void)currSigDims;
  (void)currSigDimsSize;
  (void)maxRows;
  (void)decimation;
  (void)sampleTime;
  (void)append;

  return &dummy;
}

#endif