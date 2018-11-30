/**
 	HOMEWORK 2
 	GRUPPO:
 	D'Alterio Giuseppe 	N46002736
	Di Costanzo Fernando 	N46002743
	Miranda Simona		M58000216
 */

#include "lib.h"

static RT_TASK rt_task[4*NUM_OF_WHEELS];

//shms
static int* sensor		= NULL;
static int* actuator		= NULL;
static int* reference		= NULL;

//mbx
static MBX* box_wd_0;
static MBX* box_wd_1;

//sems
static SEM space_avail[NUM_OF_WHEELS];
static SEM msg_avail[NUM_OF_WHEELS];


static struct buffer * buff = NULL;


static void acquire(int arg) {

	int wheel;
	if(rt_whoami()==&rt_task[ACQUIRE_W0_TASK])
		wheel=WHEEL_0;
	else
		wheel=WHEEL_1;

	while (1)
	{
		
		// Lettura valore sensore ruota 0
		rt_sem_wait( &space_avail[wheel] );
		
		buff[wheel].data[ buff[wheel].head ] = sensor[wheel];
		buff[wheel].head = (buff[wheel].head+1)%BUF_SIZE;

//DEBUG:	rt_printk("letto da sensore: %d", buff[wheel].data[ buff[wheel].tail] );

		rt_sem_signal( &msg_avail[wheel] );

		rt_task_wait_period();

	}

}


static void filter(int arg) {

	int wheel;
	int cnt = BUF_SIZE;
	unsigned int sum = 0;
	unsigned int avg = 0;

	if(rt_whoami()==&rt_task[FILTER_W0_TASK])
		wheel=WHEEL_0;
	else
		wheel=WHEEL_1;

	while(1) {
		
		rt_sem_wait( &msg_avail[wheel] );

		sum += buff[wheel].data[ buff[wheel].tail] ;
		buff[wheel].tail = (buff[wheel].tail+1) % BUF_SIZE;

		rt_sem_signal( &space_avail[wheel] );
		
		cnt--;

		if (cnt == 0) {
			cnt = BUF_SIZE;
			avg = sum/BUF_SIZE;
			sum = 0;
			//Sends the average measure to the controller
			if(wheel==WHEEL_0) {
				rt_send(&rt_task[CONTROL_W0_TASK], avg);
			} else {
				rt_send(&rt_task[CONTROL_W1_TASK], avg);
			}
//DEBUG:		rt_printk("Wheel 0 avg: %d", avg);
		}

		rt_task_wait_period();

	}
	

}

static void control(int arg) {

	int wheel;

	unsigned int 	plant_state = 0,
		 	prec_plant_state = 0;
	int error = 0;
	int d = 0; //derivata t.d. (diff prima)
	int conta_uguali = ABS_THRESHOLD;
	int abs=0, abs_0=0, abs_1=0;
	unsigned int control_action = 0;

	RTIME t1, t2, tmp;

	if(rt_whoami()==&rt_task[CONTROL_W0_TASK])
		wheel=WHEEL_0;
	else
		wheel=WHEEL_1;

	while (1)
	{

		t1 = rt_get_time();

		if(wheel==WHEEL_0)
			rt_receive(&rt_task[FILTER_W0_TASK], &plant_state);
		else
			rt_receive(&rt_task[FILTER_W1_TASK], &plant_state);

		d = plant_state - prec_plant_state;
		
		//se d==0 , si sta frenando e l'auto è in moto allora inizio a contare le velocità uguali, se l'abs è spento
		if( d==0 && (*reference)==0 && abs==0 && plant_state>0 ) {

			conta_uguali--;

			if(conta_uguali==0) {
//DEBUG:
	 			rt_printk("+++++++++[CTRL%d] Attivo ABS+++++++++", wheel);
				abs=ABS;
				conta_uguali=ABS_THRESHOLD;
			}	
		} else {
			conta_uguali=ABS_THRESHOLD;
		}


	// SYNC
	if( (*reference)==0 && ( abs==0 || abs==ABS ) && plant_state>0 ) {
		abs_1 = 0; //abs_1 default value

		//rt_printk("[CTRL%d] ABS MIO = %d ", wheel, abs);
		if(wheel==WHEEL_0) {

			rt_send_timed(&rt_task[CONTROL_W1_TASK], abs, nano2count(CTRL_PERIOD/5) ) ;
			rt_receive_timed(&rt_task[CONTROL_W1_TASK], &abs_1, nano2count(CTRL_PERIOD/5) ) ;

			abs = abs | abs_1;	
		} else {
			rt_receive_timed(&rt_task[CONTROL_W0_TASK], &abs_0, nano2count(CTRL_PERIOD/5) ) ;
			rt_send_timed(&rt_task[CONTROL_W0_TASK], abs, nano2count(CTRL_PERIOD/5) ) ;
	
			abs = abs | abs_0;	
		}
	
	}




	if(abs>0) { //se l'abs è attivo, allora l'azione di controllo è 0	
		if( (*reference)==0 && plant_state>0 ) {
			control_action = 0;
			abs--;
			if(!abs) {
//DEBUG:
				rt_printk("[CTRL%d] Invio ultimo 0 e disattivo ABS: cascata terminata", wheel);
			}
		} else if (abs>0) {
			abs=0;
//DEBUG:
			rt_printk("[CTRL%d] ABS disattivato: ref!= o plant_state=0", wheel);
		}					
	} else {
		// computation of the control law
		error = (*reference) - plant_state;

		// frenata '4'<->'-2' SOLO se auto in moto e freno premuto
		if ( (*reference) == 0 && plant_state>0 ) {
			control_action = 4;
		} else {
			if (error > 0) control_action = 1;
			else if (error < 0) control_action = 2;
			else control_action = 3;
		}	
	}
	// sending the control action to the actuator
	if(wheel==WHEEL_0) {
		rt_send(&rt_task[ACTUATOR_W0_TASK], control_action);
	} else {
		rt_send(&rt_task[ACTUATOR_W1_TASK], control_action);
	}

	prec_plant_state = plant_state;


	t2 = rt_get_time();


	tmp = t2-t1;
		if(wheel==WHEEL_0)
			rt_mbx_send(box_wd_0, &(tmp), sizeof(RTIME) );
		else
			rt_mbx_send(box_wd_1, &(tmp), sizeof(RTIME) );

		rt_task_wait_period();

	}

}

static void actuator_task(int arg) {

	int wheel;

	unsigned int control_action = 0;
	int cntr = 0;

	if(rt_whoami()==&rt_task[ACTUATOR_W0_TASK])
		wheel=WHEEL_0;
	else
		wheel=WHEEL_1;


	while(1) {

		// receiving the control action from the controller
		rt_receive(0, &control_action);
		
		switch (control_action) {
			case 1: cntr = 1; break;
			case 2:	cntr = -1; break;
			case 3:	cntr = 0; break;
			case 4: cntr = -2; break; //frenata -2
			default: cntr = 0;
		}

		actuator[wheel] = cntr;

		rt_task_wait_period();
	}

}






/* INIT_MODULE */

int init_module(void) {

	RTIME ctrl_period;
	int i;

	rt_printk("hello :)");
	

	//shms
	sensor 		= rt_shm_alloc(SEN_SHM, sizeof(int)*NUM_OF_WHEELS, USE_VMALLOC);
	actuator 	= rt_shm_alloc(ACT_SHM, sizeof(int)*NUM_OF_WHEELS, USE_VMALLOC);
	reference	= rt_shm_alloc(REFSENS, sizeof(int), USE_VMALLOC);

	buff = rt_shm_alloc(nam2num(BUF_SHM), sizeof(struct buffer)*NUM_OF_WHEELS, USE_VMALLOC);


	box_wd_0 = rt_typed_named_mbx_init( BOX_WD1, 1, FIFO_Q );
	box_wd_1 = rt_typed_named_mbx_init( BOX_WD2, 1, FIFO_Q );

	//init
	for(i=0; i<NUM_OF_WHEELS; i++) {
		buff[WHEEL_0].head = 0;
		buff[WHEEL_0].tail = 0;
	}

	(*reference) = 110;


	//sems
	rt_typed_sem_init(&space_avail[WHEEL_0], BUF_SIZE, CNT_SEM | PRIO_Q);

	rt_typed_sem_init(&msg_avail[WHEEL_0], 0, CNT_SEM | PRIO_Q);

	rt_typed_sem_init(&space_avail[WHEEL_1], BUF_SIZE, CNT_SEM | PRIO_Q);

	rt_typed_sem_init(&msg_avail[WHEEL_1], 0, CNT_SEM | PRIO_Q);


	ctrl_period = nano2count(CTRL_PERIOD);


	//tasks

	/* WHEEL 0 */

	// ACQUIRE WHEEL 0 TASK
	rt_task_init(&rt_task[ACQUIRE_W0_TASK], (void*)acquire, 0, 
				STACK_SIZE, TASK_PRIORITY, 1, 0);

	rt_task_make_periodic(&rt_task[ACQUIRE_W0_TASK], 
				rt_get_time()+ctrl_period, ctrl_period);

	// FILTER WHEEL 0 TASK
	rt_task_init(&rt_task[FILTER_W0_TASK], (void*)filter, 0, 
				STACK_SIZE, TASK_PRIORITY, 1, 0);

	rt_task_make_periodic(&rt_task[FILTER_W0_TASK], 
				rt_get_time()+ctrl_period, ctrl_period);

	// CONTROLLER WHEEL 0 TASK
	rt_task_init(&rt_task[CONTROL_W0_TASK], (void*)control, 0, 
				STACK_SIZE, TASK_PRIORITY, 1, 0);

	rt_task_make_periodic(&rt_task[CONTROL_W0_TASK], 
				rt_get_time()+ctrl_period, BUF_SIZE*ctrl_period);

	// ACTUATOR WHEEL 0 TASK
	rt_task_init(&rt_task[ACTUATOR_W0_TASK], (void*)actuator_task, 0,
				STACK_SIZE, TASK_PRIORITY, 1, 0);

	rt_task_make_periodic(&rt_task[ACTUATOR_W0_TASK], 
				rt_get_time()+ctrl_period, BUF_SIZE*ctrl_period);

	/* WHEEL 1 */

	// ACQUIRE WHEEL 1 TASK
	rt_task_init(&rt_task[ACQUIRE_W1_TASK], (void*)acquire, 0, 
				STACK_SIZE, TASK_PRIORITY, 1, 0);

	rt_task_make_periodic(&rt_task[ACQUIRE_W1_TASK], 
				rt_get_time()+ctrl_period, ctrl_period);

	// FILTER WHEEL 1 TASK
	rt_task_init(&rt_task[FILTER_W1_TASK], (void*)filter, 0, 
				STACK_SIZE, TASK_PRIORITY, 1, 0);

	rt_task_make_periodic(&rt_task[FILTER_W1_TASK], 
				rt_get_time()+ctrl_period, ctrl_period);

	// CONTROLLER WHEEL 1 TASK
	rt_task_init(&rt_task[CONTROL_W1_TASK], (void*)control, 0, 
				STACK_SIZE, TASK_PRIORITY, 1, 0);

	rt_task_make_periodic(&rt_task[CONTROL_W1_TASK], 
				rt_get_time()+ctrl_period, BUF_SIZE*ctrl_period);

	// ACTUATOR WHEEL 1 TASK
	rt_task_init(&rt_task[ACTUATOR_W1_TASK], (void*)actuator_task, 0,
				STACK_SIZE, TASK_PRIORITY, 1, 0);

	rt_task_make_periodic(&rt_task[ACTUATOR_W1_TASK], 
				rt_get_time()+ctrl_period, BUF_SIZE*ctrl_period);


	return 0;

}

void cleanup_module(void) {

	int i;

	for(i=0; i<8; i++)
		rt_task_delete(&rt_task[i]);

	rt_shm_free(SEN_SHM);
	rt_shm_free(ACT_SHM);
	rt_shm_free(REFSENS);
	rt_shm_free(nam2num(BUF_SHM));

	rt_sem_delete(&space_avail[WHEEL_0]);
	rt_sem_delete(&msg_avail[WHEEL_0]);
	rt_sem_delete(&space_avail[WHEEL_1]);
	rt_sem_delete(&msg_avail[WHEEL_1]);

	rt_named_mbx_delete(box_wd_0);
	rt_named_mbx_delete(box_wd_1);

	rt_printk("bye :(");

	return;

}
