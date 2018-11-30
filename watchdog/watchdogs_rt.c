/**
 	HOMEWORK 2
 	GRUPPO:
 	D'Alterio Giuseppe 	N46002736
	Di Costanzo Fernando 	N46002743
	Miranda Simona		M58000216
 */

#include "lib.h"
#define MISURE 	300
#define WCET_UB	450000000 

RTIME now;

//mbx
static MBX* box_wd[NUM_OF_WHEELS];

static RT_TASK tasks[NUM_OF_WHEELS];

static void watchdog (int num)
{	

/* MISURE
	RTIME max;
	RTIME misure[MISURE];
	int conta=0, i;
*/
	int retval;
	RTIME rcv;

	while(1) {
		//rt_printk("[WD-%d] - Attesa messaggio...", num);
		retval=rt_mbx_receive_timed(box_wd[num], &rcv, sizeof(RTIME), nano2count(2*WCET_UB) );
		/*if(!retval)
			rt_printk("[WD-%d] - Ricevuto Ci=%lu [%d]", num, count2nano(rcv), retval );*/

		if(retval!=0 || count2nano(rcv)>WCET_UB) {
			rt_printk(" !!! ---- Controller %d offline o fuori uso! ---- !!!", num);
		}



/* MISURA WCET */
/*
		if(conta<MISURE) {
			misure[conta]=rcv;
			conta ++;
		} else {
			max = misure[0];
			for(i=1; i<MISURE; i++) {
				if(misure[i]>max)
					max=misure[i];
			}
			rt_printk("++++++++++ MAX C-%d: %lu ns +++++++++", num, count2nano(max) );
		}
*/
		rt_task_wait_period();
	}
}




int init_module(void)
{
	int i;
	RTIME wd_period;

	box_wd[0] = rt_typed_named_mbx_init( BOX_WD1, 1, FIFO_Q );
	box_wd[1] = rt_typed_named_mbx_init( BOX_WD2, 1, FIFO_Q );

	wd_period = nano2count(WD_PERIOD);
	
	for (i = 0; i < NUM_OF_WHEELS; i++) {
		rt_task_init(&tasks[i], (void*)watchdog, i, RT_STACK, 1, 0, 0);
		rt_task_make_periodic(&tasks[i], rt_get_time()+wd_period, wd_period);
	}

	return 0;
}

void cleanup_module(void)
{
	int i;
	
	for (i = 0; i < NUM_OF_WHEELS; i++) {
		rt_task_delete(&tasks[i]);
	}

	rt_named_mbx_delete(box_wd[0]);
	rt_named_mbx_delete(box_wd[1]);

	rt_printk("WDs bye");

}
