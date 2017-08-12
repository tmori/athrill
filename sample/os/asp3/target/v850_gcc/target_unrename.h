
/* This file is included only when target_rename.h has been included. */
#ifdef TOPPERS_TARGET_RENAME_H
#undef TOPPERS_TARGET_RENAME_H

/*
 *  target_kernel_impl.c
 */
#undef target_initialize
#undef target_exit

/*
 *  tTraceLog.c
 */
#undef log_dsp_enter
#undef log_dsp_leave
#undef log_inh_enter
#undef log_inh_leave
#undef log_exc_enter
#undef log_exc_leave



#endif /* TOPPERS_TARGET_RENAME_H */
