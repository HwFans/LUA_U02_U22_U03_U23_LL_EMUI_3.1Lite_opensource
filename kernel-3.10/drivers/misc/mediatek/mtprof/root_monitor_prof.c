/**
 * ===========================================================================================================
 * Name        : root_monitor_prof.c
 * Author      : Yanghui LI
 * Version     : V1.2.6
 * Copyright   : Copyright (C) MediaTek Inc
 * Description : root monitor implements
 * ===========================================================================================================
 * Update List:
 * Date	        Owner	    Comments
 * 2013-12-04   Yanghui LI  V1.0   Create the document
 * 2013-12-10   Yanghui LI  V1.1   don't check if in irq, and save the result in raw data image rootm
 * 2013-12-13   Yanghui LI  V1.1.1 resolved some false alarm case.(aee_dumpstate,)
 *                                 if parent is sh, catch sh's parent, ignore sh.
 * 2013-12-25   Yanghui LI  V1.1.2P1 resolved false alarm for swap/zram
 * 2013-12-31   Yanghui LI  V1.1.2P2 resolved false alarm for shutdown/ipo
 *                                   if parent is logwrapper, catch logwrapper's parent, ignore logwrapper.
 * 2014-01-03   Yanghui LI  V1.2   resovle some false alarm case in MT6592(sdcard/wmt_loader)
 * 2014-01-09   Yanghui LI  V1.2.1 resolve if no Raw data image, cause NULL Pointer KE.
 * 2014-02-18   Yanghui LI  V1.2.2 resolve aee_core_forwarder error, don't care process which forked by kernel thread
 *                                 don't catch aee db if is not MTK internal
 * 2014-08-22   Yanghui LI  V1.2.3 resolve bug: After KK zygote fork app process, child(name as zygote) implements unshare action, 
 *                                 it uses CAP_SYS_ADMIN capabilities. 
 * 2015-03-12   Yanghui LI  V1.2.4 resolve bug: KK shutdown will use ptrace, kill root process permission.
 *                                 ipod is single process group forked by init.
 *                                 dumpstate can be called by adb shell dumpstate, and the init fork dumpstate to implements.
 * 2015-08-24   Yanghui LI  V1.2.6 update zygote comm for android L and add debuggerd64
 * 2015-10-12   Yanghui LI  V1.2.7 add umount for init for child and call fs_mgr function to mount/umount. 
 */


#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <asm/uaccess.h>
#include <linux/rtc.h>
#include <linux/hardirq.h>
#include <asm/uaccess.h>
#include <mach/mt_boot.h>

#ifdef CONFIG_MTK_AEE_FEATURE
#include <linux/aee.h>
#endif

#include <linux/root_monitor_prof.h>

static DEFINE_MUTEX(mt_rootprof_lock);

//Disabled in ENG BUILD version.
#ifdef CONFIG_MT_ENG_BUILD
static int mt_rootprof_enabled=0;
#else
static int mt_rootprof_enabled=1;
#endif

#define MAX_ROOT_RESULT 32
#define MAX_INPUT_SIZE  64

/**
 * save result to image? need partition ROOTM
 */
#define SAVE_RESULT_TO_IMAGE 1

static struct root_result_unit root_results[MAX_ROOT_RESULT];
static int root_count = 0;
static int save_count = 0;

#define SEQ_printf(m, x...)	    \
 do {			    \
    if (m)		    \
	seq_printf(m, x);	\
    else		    \
	printk(x);	    \
 } while (0)

static struct root_monitor_unit kill_process_table[]={
	{"main",         "init",      1, 0,   0,     0},  //kill it self when system server crash
	{"vold",           "init",      1, 0,   0,     0},  //ProcessKiller
	{"debuggerd",      "init",      1, 0,   0,     0},  //debuggerd
	{"debuggerd64",      "init",      1, 0,   0,     0},  //debuggerd64
	{"system_server",  "main",    0, 1000,1000,  0},  //stop app
	{"debuggerd",      "debuggerd", 0, 0,   0,     0},  //debuggerd
	{"debuggerd64",      "debuggerd64", 0, 0,   0,     0},  //debuggerd64
	{"shutdown",	   "init",	1, 0,   0,     0},  //mtk shutdown kill process
	{"ipod",	   "init",	1, 0,   0,     0},  //mtk ipo kill process
	{"netd",	   "init",	1, 0,   0,     0},  //netd maybe kill process
	{NULL, NULL, 0, 0 , 0,  0},
};

static struct root_monitor_unit sys_module_table[]={
	{"tc",		   "netd",      0, 0,   0,  0},  //netd fork tc to install module
	{"pvrsrvctl",	   "init",      1, 0,   0,  0},  //IMG powerVR GPU install module
	{"wlan_loader",    "init",      1, 0,   0,  0},  //wlan_loader install wifi module
	{"wmt_loader",     "init",      0, 0,   0,  0},  //92 new feature wmt_loader install wifi common module
	{NULL, NULL, 0, 0 , 0,  0},
};

static struct root_monitor_unit sys_admin_table[] ={
	{"init",	   "init",       0, 0,  0,  0}, //init fork service, and set some property(I.E ioprio)
	{"umount",     "init",       0, 0,  0,  0}, //umount for init for child and call fs_mgr function to mount/umount.
	{"vold",           "init",       1, 0,  0,  0}, //mount & unmount sdcard
	{"netd",           "init",       1, 0,  0,  0}, //sethostname(2),  and  setdomain©\name(2);
	{"main",         "main",     0, 0,  0,  0}, //zygote fork app process, app process need unshare action with CAP_SYS_ADMIN in KK 
	{"main",         "init",       1, 0,  0,  0}, //zygote set it's child process with NO_SUID
	{"debuggerd",      "debuggerd",  0, 0,  0,  0}, //debuggerd
	{"debuggerd64",      "debuggerd64",  0, 0,  0,  0}, //debuggerd
	{"shutdown",	   "init",	 1, 0,  0,  0},  //mtk shutdown maybe unmount image
	{"ipod",	   "init",	 1, 0,  0,  0},  //mtk ipo maybe unmount image
	{"sdcard",	   "init",	 1, 0,  0,  0},  //create virtual SD card at /mnt/sdcard
#ifdef CONFIG_ZRAM
	{"tiny_swapon",    "init",       0, 0,  0,  0}, //swap/zram after 82
        {"tiny_swapoff",   "init",       0, 0,  0,  0}, //swap/zram after 82
	{"tiny_mkswap",    "init",       0, 0,  0,  0}, //swap/zram after 82
        {"tiny_switch",    "init",       0, 0,  0,  0}, //swap/zram after 82
#endif
//#ifdef CONFIG_MT_ENG_BUILD
//	{"adbd",           "init",  1, 0 ,   0},
//#endif
	{NULL, NULL, 0, 0 , 0,  0},
};

static struct root_monitor_unit sys_ptrace_table[] ={
	{"debuggerd",      "init",          1, 0,  0,  0},  //debuggerd
	{"debuggerd",      "debuggerd",     0, 0,  0,  0},  //debuggerd
	{"debuggerd64",      "init",          1, 0,  0,  0},  //debuggerd
	{"debuggerd64",      "debuggerd64",     0, 0,  0,  0},  //debuggerd
	{"vold",           "init",          1, 0,  0,  0},  //void read proc fd information
	{"shutdown",	   "init",	    1, 0,  0,  0},  //mtk shutdown read process proc information
	{"ipod",	   "init",	    1, 0,  0,  0},  //mtk ipo read process proc information
    {"dumpstate",      "init",	    1, 0,  0,  0},  //dumpstate can read process information
//	{"aee_dumpstate",  "debuggerd",     0, 0,  0,  0},  //aee_dumpstate read proc fd information
//	{"ls",		   "aee_dumpstate", 0, 0,  0,  0},  //aee use ls to read proc information
#ifdef CONFIG_MT_ENG_BUILD
	{"procrank",       "aee_dumpstate",   0, 0,  0,   0},  //ignore aee use procrank
	{"librank",        "aee_dumpstate",   0, 0,  0,   0},  //ignore aee use librank
	{"proc_mem",       "aee_dumpstate",   0, 0,  0,   0},  //ignore aee use proc_mem
#endif
	{NULL, NULL, 0, 0 , 0,  0},
};

static struct root_monitor_unit setpcap_table[] ={
	{NULL, NULL, 0, 0 , 0,  0},
};

/**
 * this table is used to check from other uid to root uid.
 * @warning the uid is other uid, not root uid.
 * Now we only find a special case: zygote preloadClasses, 
 *     it will set euid as UNPRIVILEGED_UID and then set root again.
 */
static struct root_monitor_unit set_uid_table[] ={
	{"main", 	  "init",    1, 0,  9999,   0},  //zygote preloadclasses special case
	{NULL, NULL, 0, 0 ,0},
};

static struct root_monitor_unit ignore_process_table[] ={
	{"debuggerd",      "init",   1, 0,  0,  0},  //debuggerd
	{"debuggerd64",      "init",   1, 0,  0,  0},  //debuggerd64
	{NULL, NULL, 0, 0 ,0},
};

//save the result to image
#ifdef SAVE_RESULT_TO_IMAGE

#define MAX_TIME_LEN  32 

#define RAW_DATA_IMAGE_PATH "/dev/rootm"

#define ROOT_SAVE_MAGIC  0x00071029
#define ROOT_UNIT_MAGIC  0x00071029
#define ROOT_MAX_RECORD  1024


/**
 * =========== Image struct =============
 * 4: magic code: "0x00071029"
 * 4: int: the number of unit 
 * reserved[512-8]
 * struct root_save_unit: the list of unit
 * 
 */

/**
 * the size must alignment 512 for dumchar write
 * @warning: if you update this struct, must update reserved again.
 * @warning: need more compressed struct to use 
 */
struct root_save_unit{
	int 	       magic;
	pid_t          ppid;
	pid_t          pid;
	pid_t          tid;
	int 	       cap;
	int 	       fork_by_sh;   
	char           parent_name[TASK_COMM_LEN];
	char           process_name[TASK_COMM_LEN];
	char           thread_name[TASK_COMM_LEN];
        char           action[MAX_ACTION_LEN];
	char 	       time[MAX_TIME_LEN];
	char	       reserved[512-136];
};

/**
 * the size must alignment 512 for dumchar write
 * @warning: need more compressed struct to use 
 * @warning: if you update this struct, must update reserved again.
 */
struct root_save_head{
	int 		magic;
	int 		count;
	char		reserved[512-8];
};


static struct root_save_head  save_head_tmp;
static struct root_save_unit  save_unit_tmp;

static DEFINE_MUTEX(mt_rootprof_save_lock);

static struct file *open_file(char *path,int flag,int mode) 
{
	struct file *fp = NULL;
	fp=filp_open(path, flag, mode);
	if (!IS_ERR_OR_NULL(fp)) return fp;
	else return NULL;
}

static int read_file(struct file *fp,char *buf,int readlen)
{
	if (fp->f_op && fp->f_op->read)
		return fp->f_op->read(fp,buf,readlen, &fp->f_pos);
	else
		return -1;
}

static int write_file(struct file *fp,char *buf,int len)
{
	if (fp->f_op && fp->f_op->write)
		return fp->f_op->write(fp, buf, len, &fp->f_pos);
	else
		return -1;
}

static int close_file(struct file *fp)
{
	filp_close(fp,NULL);
	return 0;
}

/**
 * init the root_save_unit struct from root_result_unit struct
 */
static void init_root_save_unit(struct root_save_unit *save_unit, struct root_result_unit* root_unit){

	struct rtc_time tm_android;

	save_unit->magic = ROOT_UNIT_MAGIC;
	save_unit->ppid = root_unit->ppid;
	save_unit->pid  = root_unit->pid;
	save_unit->tid  = root_unit->tid;
	save_unit->cap  = root_unit->cap;

	save_unit->fork_by_sh = root_unit->fork_by_sh;

	strncpy(save_unit->parent_name, root_unit->parent_name, TASK_COMM_LEN);
	strncpy(save_unit->process_name, root_unit->process_name, TASK_COMM_LEN);
	strncpy(save_unit->thread_name, root_unit->thread_name, TASK_COMM_LEN);
	strncpy(save_unit->action, root_unit->action, MAX_ACTION_LEN);

	//time
	root_unit->tv.tv_sec -= sys_tz.tz_minuteswest*60;
	rtc_time_to_tm(root_unit->tv.tv_sec, &tm_android);
	snprintf(save_unit->time, MAX_TIME_LEN, "%d-%02d-%02d %02d:%02d:%02d",
				   tm_android.tm_year + 1900, tm_android.tm_mon + 1, tm_android.tm_mday,
				   tm_android.tm_hour, tm_android.tm_min, tm_android.tm_sec);
	root_unit->tv.tv_sec -= sys_tz.tz_minuteswest*60;
	
	//reserved 
	strncpy(save_unit->reserved, "unit end.", sizeof(save_unit->reserved));
}

/**
 * save the root monitor to result
 * return  0 : success to save result
 * return -1 : can not read the magic code or unit count
 * return -2 : can not write data.
 * return -3 : can not open /dev/rootm
 * return -4 : too many results, more than ROOT_MAX_RECORD
 */
static int save_result_to_image(struct work_struct *work){
	struct file * fp = NULL;

	int magic = 0;
	int count = 0;
	int result = 0;

	struct root_result_unit* root_unit = container_of(work, struct root_result_unit, save_work);

	mutex_lock(&mt_rootprof_save_lock);

	fp = open_file(RAW_DATA_IMAGE_PATH, O_RDWR, 0);

	if(fp != NULL){
		//read set kernel domain
		set_fs(KERNEL_DS);
		//read magic
		if(sizeof(int) != read_file(fp, &magic, sizeof(int))){
			//TODO need log
			printk("[root_monitor] read magic fail\n");
			result = -1;
			goto close_file;
		}
		//read count
		if(sizeof(int) != read_file(fp, &count, sizeof(int))){
			//TODO need log
			printk("[root_monitor] read count fail\n");
			result = -1;
			goto close_file;
		}
		init_root_save_unit(&save_unit_tmp, root_unit);
		if(magic != ROOT_SAVE_MAGIC || count > ROOT_MAX_RECORD || count <= 0){
			//we think there are no any unit, and this is first time
			generic_file_llseek(fp, 0, SEEK_SET);
			save_head_tmp.magic = ROOT_SAVE_MAGIC;
			save_head_tmp.count = 1;
			//save header
			if(sizeof(struct root_save_head) != write_file(fp, &save_head_tmp, sizeof(struct root_save_head))){
				result = -2;
				goto close_file;
			}
			//save unit
			if(sizeof(struct root_save_unit) != write_file(fp, &save_unit_tmp, sizeof(struct root_save_unit))){
				result = -2;
				goto close_file;
			}
			//update the count state
			save_count = count;
		}else if( count < ROOT_MAX_RECORD ){
			//leek 
			generic_file_llseek(fp, sizeof(struct root_save_head) + sizeof(struct root_save_unit)*count, SEEK_SET);			
			//save unit
			if(sizeof(struct root_save_unit) != write_file(fp, &save_unit_tmp, sizeof(struct root_save_unit))){
				result = -2;
				goto close_file;
			}
			//upate count
			save_head_tmp.magic = ROOT_SAVE_MAGIC;
			save_head_tmp.count = count + 1;
			generic_file_llseek(fp, 0, SEEK_SET);
			if(sizeof(struct root_save_head) != write_file(fp, &save_head_tmp, sizeof(struct root_save_head))){
				result = -2;
				goto close_file;
			}
			//update the count state
			save_count = count + 1;
		}else {
			//too many result saved in image, ignore this case
			result = -4;
			goto close_file;
		}

	}else{
		result = -3;
		goto out;
	}
close_file:
	//need set user domain again
	set_fs(USER_DS);
	//close file
	close_file(fp);
out:
	mutex_unlock(&mt_rootprof_save_lock);
	if(result < 0){
		printk("[root_monitor] save result to image fail, error=%d", result);
	}
	return result;
}

#endif


/**
 * if the task meets the table, return 0, else return 1.
 */
static int check_root_capability(const struct root_monitor_unit *table,
		 const struct task_struct *task) {
	int i = 0;
	int sid = 0;
	int flag;
	int result = 1;
	struct task_struct * parent = NULL;
	//need lock the parent, maybe released after parent exit.
	get_task_struct(task->real_parent);
	parent = task->real_parent;
	//if parent is sh, it is use system(XX), or use sh command
	//   don't care sh, catch sh's parent
	//if parent is logwrapper, it is use system/bin/logwrapper xxxx
	if(!strncmp(parent->group_leader->comm, SH_PROCESS_NAME, strlen(SH_PROCESS_NAME)+1)
		|| !strncmp(parent->group_leader->comm, LOGWRAPPER_PROCESS_NAME, strlen(LOGWRAPPER_PROCESS_NAME)+1) ){
		get_task_struct(parent->real_parent);
		parent = parent->real_parent;
		put_task_struct(task->real_parent);
	}
	for (i = 0; table[i].process_name; i++) {
		//meet process name
		if (!strncmp(task->group_leader->comm, table[i].process_name, TASK_COMM_LEN)) {
			//meet parent
			if (!strncmp(parent->group_leader->comm, table[i].parent_name, TASK_COMM_LEN)) {
				//double check parent pid must equals 1 or parent process group leader must equals 1: 
				//special for init fork process and call fs_mgr function and exec umount
				if (!strncmp(table[i].parent_name, "init", 5)){
					if (parent->group_leader->pid != 1 && task_pgrp_vnr(parent) != 0){
						result = 1;
						goto out;
					}
				}
				//meet process group leader
				flag = task_pgrp_vnr(task) == task->tgid? 1:0;
				if (flag == table[i].is_process_group_Leader){
					//meet uid & euid
					if(task_uid(task) == table[i].uid 
						&& task_euid(task) == table[i].euid){
						//meet sid, if SID_UNKNOWN, must equals parent pid
						sid = task_session_vnr(task);
						if(table[i].sid == SID_UNKNOWN 
							&& sid == task->real_parent->tgid && sid != 0){
							result = 0;
							goto out;
						}
						if(sid == table[i].sid){
							result = 0;
							goto out;
						}
					}
				}
			}
		}
	}

out:
	put_task_struct(parent);
	return result;
}

/**
 * ignore process tree
 */
static int ignore_process_tree(){
	struct task_struct * grandfather = current->real_parent;
	while(grandfather != NULL && grandfather->pid != 1 && grandfather->real_parent-> tgid != 1){
		grandfather = grandfather->real_parent;
	}
	if(grandfather != NULL && grandfather->pid != 1){
		return check_root_capability(ignore_process_table, grandfather)==0;
	}
	return 0;
}

int check_abnormal_root_capability(int cap) {
	int result = 0;
	//don't check if in irq
	if(in_interrupt()){
		return 0;
	}
	//feature enable ?
	if (!mt_rootprof_enabled){
		return 0;
	}
	//is kernel thread ? doesn't care kernel thread
	if (current->mm == NULL) {
		return 0;
	}
	//is forked by kernel thread ? doesn't care thread forked by kernel thread
	if (current->real_parent == NULL || current->real_parent->mm == NULL) {
		return 0;
	}
	//init, doesn't care.
	if (current->pid == 1){
		return 0;
	}
	//check root capability table
	if (cap == CAP_KILL) {
		result = check_root_capability(kill_process_table, current);
	}else if (cap == CAP_SYS_MODULE){
		result = check_root_capability(sys_module_table, current);
	}else if (cap == CAP_SYS_ADMIN){
		result = check_root_capability(sys_admin_table, current);
	}else if (cap == CAP_SYS_PTRACE){
		result = check_root_capability(sys_ptrace_table, current);
	}else if (cap == CAP_SETPCAP){
		result = check_root_capability(setpcap_table, current);
	}else if (cap == -1){ //for setuid from other uid to root
		result = check_root_capability(set_uid_table, current);
	}else {
		//doesn't care other capability.
		result = 0;
	}
	//ignore process tree
	if (result ==1 && ignore_process_tree()){
		return 0;
	}
	return result;
}

/**
 * add root result to list.
 * warning: we just care more earlier result.
 * return the index of new result, if not return -1
 */
static int add_root_result(const char* action, int cap){
	int i = 0;
	int result = -1;
	struct task_struct * parent = current->real_parent;

	mutex_lock(&mt_rootprof_lock);

	//check there is a same result? process name, thread name, and action, cap
	for(i=0 ; i< root_count; i++){
		if(!strncmp(root_results[i].thread_name, current->comm, TASK_COMM_LEN)
				&& !strncmp(root_results[i].process_name, current->group_leader->comm, TASK_COMM_LEN)
				&& !strncmp(root_results[i].action, action, MAX_ACTION_LEN)
				&& cap == root_results[i].cap ){
			//maybe so many logs, ignore same PID log
			if(current->tgid != root_results[i].pid){
				printk("[root_monitor] ignore duplicate root result:%s-%s:%s-%d\n", 
					root_results[i].process_name, root_results[i].thread_name, action,cap);
			}
			goto out;
		}
	}

	if(MAX_ROOT_RESULT > root_count){
		//if parent is sh, it is use system(XX), or use sh command
		//   don't care sh, catch sh's parent
		if(!strncmp(parent->group_leader->comm, SH_PROCESS_NAME, strlen(SH_PROCESS_NAME)+1)){
			parent = parent->real_parent;
			root_results[root_count].fork_by_sh = FORK_BY_SYSTEM;
		}else if (!strncmp(parent->group_leader->comm, LOGWRAPPER_PROCESS_NAME, strlen(LOGWRAPPER_PROCESS_NAME)+1)){
			parent = parent->real_parent;
			root_results[root_count].fork_by_sh = FORK_BY_LOGWRAPPER;
		}else{
			root_results[root_count].fork_by_sh = 0;
		}
		root_results[root_count].ppid = parent->tgid;
		root_results[root_count].pid = current->tgid;
		root_results[root_count].tid = current->pid;
		//TODO fix me, need lock to protect the parent changed.
		strncpy(root_results[root_count].parent_name, parent->comm, TASK_COMM_LEN);
		//here can't not use get_task_comm, it will lead deadlock.
		strncpy(root_results[root_count].thread_name, current->comm, TASK_COMM_LEN);
		//get_task_comm(root_results[root_count].thread_name, current);
		strncpy(root_results[root_count].process_name, current->group_leader->comm, TASK_COMM_LEN);
		//get_task_comm(root_results[root_count].process_name, current->group_leader);
		do_gettimeofday(&(root_results[root_count].tv));
                strncpy(root_results[root_count].action, action, MAX_ACTION_LEN);
		root_results[root_count].cap = cap;
		result = root_count;
		root_count ++;
	}else{
		//add log to here
		printk("[root_monitor] Have record abnormal root action times: %d, ignore it.", root_count);
	}
	 //TODO check the result type

out:
	mutex_unlock(&mt_rootprof_lock);

	return result;
}

static const char* cap_to_string(int cap){
	if (cap == CAP_KILL) {
		return "CAP_KILL";
	}else if (cap == CAP_SYS_MODULE){
		return "CAP_SYS_MODULE";
	}else if (cap == CAP_SYS_ADMIN){
		return "CAP_SYS_ADMIN";
	}else if (cap == CAP_SYS_PTRACE){
		return "CAP_SYS_PTRACE";
	}else if (cap == CAP_SETPCAP){
		return "CAP_SETPCAP";
	}else{
		return "UNKNOWN";
	}
}

void report_abnormal_root(const char* action, int cap){
	int index = 0;
	int result = 0;
	//feature enable ?
	if (!mt_rootprof_enabled){
		//do nothing
		return ;
	}
	//add to tmp result list
	result = add_root_result(action, cap);
	if(result != -1) {
		//add log to here
        	printk("[root_monitor] find a abnormal root action: PID=%d %s:%s-%d\n", 
			current->tgid, current->group_leader->comm, action, cap);
		//call aee API to record the result
#if defined (CONFIG_MTK_AEE_FEATURE) && defined (MTK_INTERNAL_BUILD) 
		//only default maybe use DB_OPT_DEFAULT|DB_OPT_DUMPSYS_ACTIVITY
		if(cap == -1){
			aee_kernel_warning_api("root_monitor", __LINE__, DB_OPT_DEFAULT, 
		        	"abnormal root action", "PID=%d %s %s\n", 
		        	current->tgid, current->group_leader->comm, action);
		}else{
			aee_kernel_warning_api("root_monitor", __LINE__, DB_OPT_DEFAULT, 
		        	"abnormal root action", "PID=%d %s %s-%s\n", 
		        	current->tgid, current->group_leader->comm, action, cap_to_string(cap));
		}
#endif

#ifdef SAVE_RESULT_TO_IMAGE
		if( save_count < ROOT_MAX_RECORD ){
			index = result;
			if(index >= 0){
				//TODO fixed me !!! if clean the result data, it maybe double use save_work, 
				//     and maybe the save action is not completed.
				INIT_WORK(&(root_results[index].save_work), save_result_to_image);
				schedule_work(&root_results[index].save_work);
				//TODO need wait for saving complete?		
			}
		}
#endif

	}
}

/**
 * show the root monitor result in this current runtime 
 */
static int mt_rootprof_show(struct seq_file *m, void *v){
	int i = 0;
	int count = root_count ;
	struct rtc_time tm_android;

	//Warning: We don't use mutex lock to protect root_count here.
        //         we don't care now root event, just older events.	

	if(mt_rootprof_enabled){
                SEQ_printf(m, "root monitor is enable\n");
	}else{
		SEQ_printf(m, "root monitor is disable\n");
	}
	SEQ_printf(m, "find abnormal root %d action\n", count);
	for(i=0; i< count; i++){
		//Process/thread action cap infomation
		if(root_results[i].fork_by_sh == 1){
			SEQ_printf(m, "[*Parent=%d %s, Process=%d %s,Thread=%d %s] action:%s, cap=%s,", 
				root_results[i].ppid, root_results[i].parent_name,
				root_results[i].pid,  root_results[i].process_name,
				root_results[i].tid,  root_results[i].thread_name, 
				root_results[i].action, cap_to_string(root_results[i].cap));
		}else{
			SEQ_printf(m, "[Parent=%d %s, Process=%d %s,Thread=%d %s] action:%s, cap=%s,", 
				root_results[i].ppid, root_results[i].parent_name,
				root_results[i].pid,  root_results[i].process_name,
				root_results[i].tid,  root_results[i].thread_name, 
				root_results[i].action, cap_to_string(root_results[i].cap));
		}
		//time
		root_results[i].tv.tv_sec -= sys_tz.tz_minuteswest*60;
		rtc_time_to_tm(root_results[i].tv.tv_sec, &tm_android);
		SEQ_printf(m, "time %d-%02d-%02d %02d:%02d:%02d\n",
				   tm_android.tm_year + 1900, tm_android.tm_mon + 1, tm_android.tm_mday,
				   tm_android.tm_hour, tm_android.tm_min, tm_android.tm_sec);
		root_results[i].tv.tv_sec += sys_tz.tz_minuteswest*60;
	}

	return 0;
}

static int mt_rootprof_open(struct inode *inode, struct file *file) 
{ 
    return single_open(file, mt_rootprof_show, inode->i_private); 
} 

static void mt_rootprof_switch(int on)
{
    mutex_lock(&mt_rootprof_lock);
    if (mt_rootprof_enabled ^ on){
	if (on){
		mt_rootprof_enabled = 1;
		//reset the result
		root_count = 0;
	}else{
		mt_rootprof_enabled = 0;
		//save old result ? now yes.
		//root_count = 0;		
	}
    }
    mutex_unlock(&mt_rootprof_lock);
}

static void mt_rootprof_clean()
{
	mutex_lock(&mt_rootprof_lock);
	root_count = 0;
	mutex_unlock(&mt_rootprof_lock);
}

/**
 * 1 > start, 0 > stop, 0000 > clean result
 *
 */
static ssize_t mt_rootprof_write(struct file *filp, const char *ubuf,
	   size_t cnt, loff_t *data)
{
    char buf[MAX_INPUT_SIZE];
    size_t copy_size = cnt;

    if (cnt >= sizeof(buf))
	copy_size = MAX_INPUT_SIZE - 1;

    if (copy_from_user(&buf, ubuf, copy_size))
	return -EFAULT;

    buf[copy_size] = 0;
    //printk("[root_monitor] input data is %s,size=%d\n", buf, cnt);

#ifdef CONFIG_MT_ENG_BUILD
    if(cnt==2){
	if(buf[0] == '0')
	    mt_rootprof_switch(0);
	else if(buf[0] == '1')
	    mt_rootprof_switch(1);
    }else if(cnt == 5){
        if(!strncmp(buf, "0000", 4)){
            mt_rootprof_clean();
        }
    }
#endif

    return cnt;

}

/**
 * to reduce "false alarm", we only open this feature as in normal use.
 *
 */
void update_rootprof_enabled(){
	int mode = get_boot_mode();
	if(mt_rootprof_enabled){
		if(mode != NORMAL_BOOT && mode != SW_REBOOT && mode != ALARM_BOOT){
			mt_rootprof_enabled = 0;
		}
	}
}

static const struct file_operations mt_rootprof_fops = { 
    .open = mt_rootprof_open, 
    .write = mt_rootprof_write,
    .read = seq_read, 
    .llseek = seq_lseek, 
    .release = single_release, 
};


static int __init init_root_prof(void)
{
    struct proc_dir_entry *pe;
    //check g_boot_mode to update mt_rootprof_enabled
    update_rootprof_enabled();
    //here we can start or stop root monitor prof
    //user build version close this feature? now No
    pe = proc_create("rootprof", 0664, NULL, &mt_rootprof_fops);
    if (!pe)
	return -ENOMEM;
    return 0;
}

__initcall(init_root_prof);


