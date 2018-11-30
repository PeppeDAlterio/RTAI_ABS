#ifndef _RT_HW2_LIB_
#define _RT_HW2_LIB_

#include <linux/module.h>
#include <asm/io.h>

#include <rtai.h>
#include <rtai_nam2num.h>
#include <rtai_shm.h>
#include <rtai_sem.h>
#include <rtai_msg.h>
#include <rtai_mbx.h>
#include <rtai_sched.h>

#define CTRL_PERIOD		45000000
#define TASK_PRIORITY 		1
#define STACK_SIZE 		15000
#define BUF_SIZE 		10
#define CPUMAP			0x1

#define SEN_SHM 121111
#define ACT_SHM 112112
#define BUF_SHM "bufshm"
#define BOX_WD1 "bxwd1"
#define BOX_WD2 "bxwd2"
#define REFSENS 111213

#define NUM_OF_WHEELS 2

#define ABS_THRESHOLD 	1
#define ABS 		3

#define NUM_OF_BLOCKS 7
#define NUM_OF_MEASUREMENT 100

enum {
	WHEEL_0=0,
	WHEEL_1=1
};

enum {

	ACQUIRE_W0_TASK,
	ACQUIRE_W1_TASK,
	FILTER_W0_TASK,
	FILTER_W1_TASK,
	CONTROL_W0_TASK,
	CONTROL_W1_TASK,
	ACTUATOR_W0_TASK,
	ACTUATOR_W1_TASK

};

struct buffer {
	
	int 	data[BUF_SIZE];
	int 	head,
		tail;
	
};

struct hbt_str {
	long id;
	
	RTIME time;
};


#endif
