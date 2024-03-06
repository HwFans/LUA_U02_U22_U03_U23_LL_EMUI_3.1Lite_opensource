/**
 * ============================================================================
 * Name        : root_monitor_prof.h
 * Author      : Yanghui LI
 * Version     : v 1.1
 * Copyright   : Copyright (C) MediaTek Inc
 * Description : root monitor implements
 * ============================================================================
 * Update List:
 * Date	        Owner	    Comments
 * 2013-12-04   Yanghui Li  Create the document
 * 2013-12-10   Yanghui LI  V1.1 don't check if in irq, and save the result in raw data image rootm
 * 2013-12-13   Yanghui LI  V1.1.1 resolved some false alarm case.(aee_dumpstate,)
 */

#ifndef ROOT_MONITOR_PROF_H_
#define ROOT_MONITOR_PROF_H_

#include <linux/capability.h>
#include <linux/sched.h>

#define MAX_ACTION_LEN 32

#define SID_UNKNOWN     	-1
#define SH_PROCESS_NAME		"sh"
#define LOGWRAPPER_PROCESS_NAME "logwrapper"

/**
 * the root monitor unit
 * @warning if the parent_name is init, need check parent PID=1; 
 * @warning if parent_name is kthread, need check parent PID=2
 * @warning if sid = SID_UNKNOWN, it is not init sid 0, and is the parent PID.
 * @warning if use system(xxx) function, then P1->sh->P2, P2.pgid = P2.pid, P2.sid = sh.sid = sh.pid
 *             so, if parent is sh, then we set the parent as parent of sh. P1->sh->P2 ====> parent = P1;
 *             
 */
struct root_monitor_unit{
	char*    process_name;
	char*    parent_name;
	int      is_process_group_Leader;
	int      uid;
	int      euid;
	int      sid;
};

struct root_result_unit{
	struct timeval tv;
	pid_t          ppid;
	pid_t          pid;
	pid_t          tid;
	int 	       cap;
	int	       fork_by_sh; //system(xxxx), system/bin/logwrapper xxxx
	char           parent_name[TASK_COMM_LEN];
	char           process_name[TASK_COMM_LEN];
	char           thread_name[TASK_COMM_LEN];
        char           action[MAX_ACTION_LEN];
	struct work_struct save_work;
};

#define FORK_BY_SYSTEM		1
#define FORK_BY_LOGWRAPPER 	2


/**
 * check abnormal root capability.
 * if the process action is not abnormal, return 1, else return 0
 */
int check_abnormal_root_capability(int cap);

/**
 * report the abnormal root action
 * @action the action of process
 * @cap    the capability of abnormal, if no-cap, cap = -1
 */
void report_abnormal_root(const char* action, int cap);


#endif /* ROOT_MONITOR_PROF_H_ */
