idf_component_register(
    SRCS
        "main.c"
        "../common/rt_nonfinite.c"
        "../common/rtGetInf.c"
        "../common/rtGetNaN.c"
        "../lqr_tracker_step_codegen/atan2.c"
        "../lqr_tracker_step_codegen/lqr_tracker_step_codegen.c"
        "../lqr_tracker_step_codegen/lqr_tracker_step_codegen_emxAPI.c"
        "../lqr_tracker_step_codegen/lqr_tracker_step_codegen_emxutil.c"
        "../lqr_tracker_step_codegen/lqr_tracker_step_codegen_initialize.c"
        "../lqr_tracker_step_codegen/lqr_tracker_step_codegen_terminate.c"
        "../Traj2_grt_rtw/Traj2.c"
        "../Traj2_grt_rtw/Traj2_data.c"
    INCLUDE_DIRS
        "."
        "../common"
        "../lqr_tracker_step_codegen"
        "../Traj2_grt_rtw"
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-function
    -Wno-unused-variable
)