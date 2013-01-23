/*
 * ltt/probes/rt-trace.c
 *
 * 2011-21-02:  Herbert.Bernecker@siemens.com
 * Copyright (c) 2007 Siemens AG
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 * 
 */
#include <linux/module.h>
#include <trace/rt.h>

void probe_rt_process_schedchange(unsigned int curr_pid, unsigned int req_pid, int rt_state)
{
	trace_mark_tp(rt, process_schedchange, rt_process_schedchange, probe_rt_process_schedchange,
		"curr_pid %u req_pid %u rt_state %i", curr_pid, req_pid, rt_state);
}

void probe_rt_fs_exec(char *filename)
{
	trace_mark_tp(rt, exec, rt_fs_exec, probe_rt_fs_exec, "filename %s",
		filename);
}

MODULE_LICENSE("GPL and additional rights");
MODULE_AUTHOR("Herbert Bernecker");
MODULE_DESCRIPTION("RT Tracepoint Probes");
