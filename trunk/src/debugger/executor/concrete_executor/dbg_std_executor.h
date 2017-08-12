#ifndef _DBG_STD_EXECUTOR_H_
#define _DBG_STD_EXECUTOR_H_

extern void dbg_std_executor_parse_error(void *executor);
extern void dbg_std_executor_break(void *executor);
extern void dbg_std_executor_delete(void *executor);
extern void dbg_std_executor_cont(void *executor);
extern void dbg_std_executor_watch_data(void *executor);
extern void dbg_std_executor_intr(void *executor);

extern void dbg_std_executor_next(void *executor);
extern void dbg_std_executor_return(void *executor);
extern void dbg_std_executor_quit(void *executor);
extern void dbg_std_executor_exit(void *executor);

extern void dbg_std_executor_elaps(void *executor);
extern void dbg_std_executor_view(void *executor);
extern void dbg_std_executor_print(void *executor);
extern void dbg_std_executor_memset(void *executor);

extern void dbg_std_executor_serialin(void *executor);
extern void dbg_std_executor_info_cpu(void *executor);
extern void dbg_std_executor_func_trace(void *executor);
extern void dbg_std_executor_data_access_info(void *executor);
extern void dbg_std_executor_back_trace(void *executor);
extern void dbg_std_executor_profile(void *executor);
extern void dbg_std_executor_list(void *executor);
extern void dbg_std_executor_help(void *executor);


#endif /* _DBG_STD_EXECUTOR_H_ */
