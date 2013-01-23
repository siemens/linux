 
#ifndef _TRACE_RT_H
#define _TRACE_RT_H

#include <linux/tracepoint.h>

DECLARE_TRACE(rt_process_schedchange,
	TP_PROTO(unsigned int curr_pid, unsigned int req_pid, int rt_state),
	TP_ARGS(curr_pid,req_pid,rt_state));

DECLARE_TRACE(rt_fs_exec,
	TP_PROTO(char *filename),
	TP_ARGS(filename));

#endif
