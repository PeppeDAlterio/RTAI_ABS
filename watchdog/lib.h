#ifndef __HOMEWORK2_WD__
#define __HOMEWORK2_WD__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/errno.h>

#include <rtai_mbx.h>

#include <rtai.h>
#include <rtai_sched.h>

#define RT_STACK 10000

#define NUM_OF_WHEELS 2

#define BOX_WD1 "bxwd1"
#define BOX_WD2 "bxwd2"

#define WD_PERIOD 90000000

struct hbt_str {
	long id;
	
	RTIME time;
};

#endif