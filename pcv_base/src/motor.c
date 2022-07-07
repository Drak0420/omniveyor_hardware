/*------------------------------includes--------------------------------------*/
/* cpp - c cross compilation */
#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <mqueue.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <inttypes.h>

#include "../include/CAN_utils.h"
#include "../include/COB_ID.h"
#include "../include/CO_objects.h"
#include "../include/CO_message.h"
#include "../include/RT_utils.h"
#include "../include/motor_init_sequence.h"
#include "../include/motor.h"
#include <omniveyor_common/definitions.h>

/*------------------------------structs---------------------------------------*/
struct motor
{
	pthread_mutex_t lock;				/* lock for serial access */
	pthread_t listener;					/* the listener thread */
	mqd_t mQueue;						/* message queue */
	char *mQueue_name;
	int s;								/* socket for CAN transmission */
	uint8_t no;							/* motor number (1-8) */
	double cur_pos;						/* most recent position received */
	double cur_vel;						/* most recent velocity received */
	double cur_trq;						/* most recent torque received */
	double cur_voltage;					/* most recent DC supplied voltage of the motor */
	double cur_amp;						/* most recent DC current flowing through the motor */
	timer_t msg_timer;					/* timer for handling tx timeouts */
	timer_t heartbeat_timer;			/* timer for tracking heartbeat */
	enum ctrl_mode cm;					/* control mode (velocity or torque) */
	enum motor_type mt;					/* motor type (steering or rolling) */
	bool enabled;						/* is the motor software enabled? */
	bool enable_pin_active;             /* is the enable pin active */
	bool stopped;
	volatile bool fault;
	bool stale_pos;
	bool stale_vel;
	bool stale_trq;
	int32_t inputs;						/* digital inputs on the motor controller*/
	bool ok;							/* Quit indicator */
};

/*------------------------static function declarations------------------------*/
static int init_mQueue (struct motor *m);
static void flush_mQueue (struct motor *m, unsigned int threshold);
static int home_motor (struct motor *m);
static int init_motor (struct motor *m);
static int32_t get_cal_offset (struct motor *m);

static void *listener (void *aux);

static void msg_timer_handler (union sigval val);
static void heartbeat_timer_handler (union sigval val);

static int32_t position_SI_to_IU (double position_SI);
static double position_IU_to_SI (int32_t position_IU);
static int32_t velocity_SI_to_IU (double velocity_SI);
static double velocity_IU_to_SI (int32_t velocity_IU);
static int32_t accel_SI_to_IU (double velocity_SI);
static double accel_IU_to_SI (int32_t velocity_IU);
static int16_t torque_SI_to_IU (double torque_SI);
static double amp_to_torque_SI (double amp_SI);
static double amp_IU_to_SI (int16_t amp_IU);
static double volt_IU_to_SI (int16_t volt_IU);

/*------------------------static variable declarations------------------------*/
static const double home_offsets[] = {HOME_OFFSET_MTR_1,
										HOME_OFFSET_MTR_3,
										HOME_OFFSET_MTR_5,
										HOME_OFFSET_MTR_7};

/*---------------------------public functions---------------------------------*/

/*
 * This function initializes motor_no and sets it up for control mode cm,
 * given its particular type. Returns NULL pointer if any part fails
 */
struct motor *
motor_init (uint8_t motor_no, enum ctrl_mode cm, enum motor_type mt)
{
	printf("motor_no: %d\n", motor_no);
	printf("cm: %d\n", cm);
	printf("mt: %d\n", mt);

	assert (0 < motor_no && motor_no <= NUM_MOTORS);
	struct motor *m;
	bool success = false;

	/* malloc the struct */
	m = malloc (sizeof (struct motor));
	if (m == NULL) {
		return NULL;
	}

	/* initialize the motor struct member */
	pthread_mutex_init (&m->lock, NULL);

	m->no = motor_no;
	m->cm = cm;
	m->mt = mt;
	m->enabled = false;
	m->stopped = true;
	m->fault = false;
	m->enable_pin_active = true;
	m->stale_pos = true;
	m->stale_vel = true;
	m->stale_trq = true;
	m->ok = true;
	
	/* open a CAN socket for this motor no */
	m->s = create_can_socket (m->no, 0xF);
	if (m->s < 0){
		printf("CAN socket creation FAILED!\r\n");
		goto done;
	}
	
	/* initialize timers */
	if (init_rt_timer (&m->msg_timer, msg_timer_handler, m)){
		printf("Init RT MSG timer FAILED!\r\n");
		goto done;
	}

	if (init_rt_timer (&m->heartbeat_timer, heartbeat_timer_handler, m)){
		printf("Init RT HeartBeat timer FAILED!\r\n");
		goto done;
	}

	/* initialize the message queue for this motor */
	if (init_mQueue (m)){
		printf("Init mQueue FAILED!\r\n");
		goto done;
	}
	
	/* launch the listening thread */
	if (launch_rt_thread (listener, &m->listener, m, MAX_PRIO)){	// Communication thread has higher priority than control thread.
		printf("Launch RT thread FAILED!\r\n");
		goto done;
	}
	
	/* now perform the initialization for the particular drive mode */
	if (init_motor (m)){
		printf("Init motor drive FAILED!\r\n");
		goto done;
	}

	if (m->no==1){
		double supplyVoltage = 0.;
		bool isStale = true;
		int counter = 0;
		while (supplyVoltage < 24.){
			if (!isStale && counter%1000==0){
				puts ("Motor drives not powered yet... Is the E-STOP activated or RESET not pressed?\r\n");
				counter++;
			}
			struct CO_message msg;
			msg.type = SYNC;
			CO_send_message (m->s, 0, &msg);
			usleep( 100000ul );
			pthread_mutex_lock (&m->lock);
			supplyVoltage = m->cur_voltage;
			isStale = m->stale_trq;
			pthread_mutex_unlock (&m->lock);
		}
	}

	/* home the motor if STEERING motor */
	if (home_motor (m)){
		printf("Motor homing FAILED!\r\n");
		goto done;
	}

	success = true;
	
	done:
		/* perform cleanup if any step failed */
		if (!success)
		{
			motor_destroy (m);
			m = NULL;
		}
	return m;
}


/*
 * Sends out RPDO2 with the given velocity data (rad/s) to control a
 * new velocity
 */
void
motor_set_velocity (struct motor *m, double velocity)
{
	assert (m != NULL);

	/* ignore call if the motor is not in VELOCITY mode*/
	if (m->enabled && m->cm == VELOCITY)
	{
		uint32_t ui_vel = (uint32_t)velocity_SI_to_IU (velocity);
		struct CO_message msg = {RPDO, .m.PDO = {2, ui_vel, 4}};
		CO_send_message (m->s, m->no, &msg);
		//printf("Motor %d cmd sent\n", m->no);
	}
}


/*
 * Returns the most recently received velocity value in SI units (rad/s). If
 * no new update received since the last call to motor_get_velocity, the
 * function returns -1
 */
int
motor_get_velocity (struct motor *m, double *velocity)
{
	assert (m != NULL);
	assert (velocity != NULL);
	int retVal = -1;

	pthread_mutex_lock (&m->lock);
	*velocity = m->cur_vel;
	if (!m->stale_vel)
	{
		m->stale_vel = true;
		retVal = 0;
	}
	pthread_mutex_unlock (&m->lock);

	return retVal;
}


/*
 * Returns the most recently received position value in SI units (rad). If
 * no new update received since the last call to motor_get_position, the
 * function returns -1
 */
int
motor_get_position (struct motor *m, double *position)
{
	assert (m != NULL);
	assert (position != NULL);
	int retVal = -1;

	pthread_mutex_lock (&m->lock);
	*position = m->cur_pos;
	if (!m->stale_pos)
	{
		m->stale_pos = true;
		retVal = 0;
	}
	pthread_mutex_unlock (&m->lock);

	return retVal;
}


/*
 * Returns true if the bumper next to the motor is hit.
 */
bool
motor_get_inputs(struct motor *m)
{
	assert(m != NULL);
	pthread_mutex_lock(&m->lock);
	int32_t inputs = m->inputs;
	bool retVal = inputs & BIT21HI;
	pthread_mutex_unlock(&m->lock);
	return retVal;
}


/*
 * Sends out RPDO2 with the given torque data (n/m) to control a
 * new torque
 */
void
motor_set_torque (struct motor *m, double torque)
{
	assert (m != NULL);
	/* ignore call if the motor is not in torque mode*/
	if (m->enabled  && m->cm == TORQUE) {
	  	//printf("Motor torque: %f \r\n", torque);
		// if(abs(torque) > TORQUE_CONT){
		// 	printf( "Motor torque too high!");
	 //    	torque = ( (torque > 0) - (torque < 0) ) * TORQUE_CONT;
	 //    	printf("New torque: %f", torque);

	 //  	}
		uint16_t ui_trq = (uint16_t)torque_SI_to_IU (torque);
		//printf("UI torque: %u \r\n", ui_trq);
		struct CO_message msg = {RPDO, .m.PDO = {2, (ui_trq<<16), 4}};
		CO_send_message (m->s, m->no, &msg);
	}
}


/*
 * Returns the most recently received torque value in SI units (n/m). If
 * no new update received since the last call to motor_get_torque, the
 * function returns -1
 */
int
motor_get_torque (struct motor *m, double *torque)
{
	assert (m != NULL);
	assert (torque != NULL);
	int retVal = -1;

	pthread_mutex_lock (&m->lock);
	*torque = m->cur_trq;
	if (!m->stale_trq)
	{
		m->stale_trq = true;
		retVal = 0;
	}
	pthread_mutex_unlock (&m->lock);

	return retVal;
}


/*
 * Returns the most recently received Current and Voltage value in SI units (A) (V).
 */
void
motor_get_amp (struct motor *m, double *amp)
{
	assert (m != NULL);
	assert (amp != NULL);
	
	pthread_mutex_lock (&m->lock);
	*amp = m->cur_amp;
	pthread_mutex_unlock (&m->lock);
}

void
motor_get_volt (struct motor *m, double *volt)
{
	assert (m != NULL);
	assert (volt != NULL);
	
	pthread_mutex_lock (&m->lock);
	*volt = m->cur_voltage;
	pthread_mutex_unlock (&m->lock);
}


/* interface to the motor control mode */
void
motor_set_ctrl_mode (struct motor *m, enum ctrl_mode cm)
{
	assert (m != NULL);
	assert (!(m->enabled) || m->stopped);
	enum ctrl_mode old = motor_get_ctrl_mode(m);
	/* send the proper mode in the first message of enable sequence */
	struct CO_message msg = {SDO_Rx, .m.SDO = {MODES_OF_OPERATION, 0x00, 0x00, 1}};		/* Set mode of operation to homing (enum=0)*/
	switch (cm)
	{
		case HOMING:
			printf("Motor %d: Home mode case in motor_set_ctrl_mode\n", m->no);
			msg.m.SDO.data += HOME_MODE;
			printf("Value: %hhd\n", msg.m.SDO.data);
			break;

		case VELOCITY:
			printf("Motor %d: Velocity mode case in motor_set_ctrl_mode\n", m->no);
			msg.m.SDO.data += VELOCITY_MODE;
			printf("Value: %hhd\n", msg.m.SDO.data);
			break;

		case TORQUE:
			printf("Motor %d: Torque mode case in motor_set_ctrl_mode\n", m->no);
			msg.m.SDO.data += TORQUE_MODE;
			printf("Value: %hhd\n", msg.m.SDO.data);
			break;
		
		default:
			break;
	}
	flush_mQueue(m, 0);
	/* send first message to kick off enable sequence */
	struct event e;
	struct itimerspec itmr = {{0}};
	itmr.it_value.tv_sec = MSG_TIMEOUT;
	CO_send_message (m->s, m->no, &msg);
	timer_settime (m->msg_timer, 0, &itmr, NULL);
	while (1)
	{
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
		if (e.type == TIMEOUT)
		{
			puts ("msg timer timeout, motor_set_ctrl_mode failed\r\n");
			itmr.it_value.tv_sec = 0;
			timer_settime (m->msg_timer, 0, &itmr, NULL);
			m->cm = old;
			return;
		}
		else if (e.type == SDO_WR_ACK && e.param == MODES_OF_OPERATION)
		{
			itmr.it_value.tv_sec = 0;
			timer_settime (m->msg_timer, 0, &itmr, NULL);
			m->cm = cm;
			return;
		}
	}
	// puts ("motor_set_ctrl_mode not implemented");
}


/*
 * Query function for the control mode of the motor (VELOCITY or TORQUE)
 */
enum ctrl_mode
motor_get_ctrl_mode (struct motor *m)
{
	assert (m != NULL);
	return m->cm;
}


/*
 * Query function for the motor type (ROLLING or STEERING)
 */
enum motor_type
motor_get_type (struct motor *m)
{
	assert (m != NULL);
	return m->mt;
}

/*
 * Enables a motor, returns 0 on success and -1 on failure
 */
int
motor_enable (struct motor *m)
{
	assert (m != NULL);
	if (!(m->enabled)){
		struct event e;
		struct itimerspec itmr = {{0}};
		itmr.it_value.tv_sec = MSG_TIMEOUT;
		unsigned i = 0;
		CO_send_message (m->s, m->no, &enable_sequence[i]);
		timer_settime (m->msg_timer, 0, &itmr, NULL);

		while (i < NUM_ENABLE_STEPS)
		{
			mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);

			if (e.type == TIMEOUT)
			{
				puts ("msg timer timeout, motor_enable failed\r\n");
				itmr.it_value.tv_sec = 0;
				timer_settime (m->msg_timer, 0, &itmr, NULL);
				return -1;
			}
			else if (e.type == STATUS_WRD_REC &&
								(e.param & ENABLE_Rx_MASK) == enable_responses[i].param)
			{
				if (++i < NUM_ENABLE_STEPS)
				{
					CO_send_message (m->s, m->no, &enable_sequence[i]);
					timer_settime (m->msg_timer, 0, &itmr, NULL);
				}
			}
		}

		/* stop timer */
		itmr.it_value.tv_sec = 0;
		timer_settime (m->msg_timer, 0, &itmr, NULL);

		/* set to enabled */
		m->enabled = true;
		m->stopped = false;
		flush_mQueue(m, 0);
	}
	else if (m->stopped)
		motor_start(m);
	return 0;
}

int
motor_enable_fast (struct motor *m)		// uses tpdo1 / rpdo1
{
	assert (m != NULL);
	if (!(m->enabled)){
		struct event e;
		struct itimerspec itmr = {{0}};
		itmr.it_value.tv_sec = MSG_TIMEOUT;

		/* send first message to kick off enable sequence */
		unsigned i = 0;
		/* send the proper mode in the first message of enable sequence */
		struct CO_message msg = {RPDO, .m.PDO = {1, enable_sequence[i].m.SDO.data, 2}};

		CO_send_message (m->s, m->no, &msg);
		timer_settime (m->msg_timer, 0, &itmr, NULL);
		
		while (i < NUM_ENABLE_STEPS)
		{
			mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);

			if (e.type == TIMEOUT)
			{
				puts ("msg timer timeout, motor_enable failed\r\n");
				itmr.it_value.tv_sec = 0;
				timer_settime (m->msg_timer, 0, &itmr, NULL);
				return -1;
			}
			else if (e.type == STATUS_WRD_REC &&
								(e.param & ENABLE_Rx_MASK) == enable_responses[i].param)
			{
				if (++i < NUM_ENABLE_STEPS)
				{
					msg.m.PDO.data = enable_sequence[i].m.SDO.data;
					CO_send_message (m->s, m->no, &msg);
					timer_settime (m->msg_timer, 0, &itmr, NULL);
				}
			}
		}

		/* stop timer */
		itmr.it_value.tv_sec = 0;
		timer_settime (m->msg_timer, 0, &itmr, NULL);
		
		/* set to enabled */
		m->enabled = true;
		m->stopped = false;
		flush_mQueue(m, 0);
	}
	else if (m->stopped)
		motor_start(m);
	return 0;
}

/*
 * Interface for disabling the motor (disable voltage)
 */
void
motor_disable (struct motor *m)
{
	assert (m != NULL);
	struct CO_message msg = {SDO_Rx, .m.SDO = {CONTROL_WORD, 0x00, 0x0000, 2}};
	CO_send_message (m->s, m->no, &msg);
	m->enabled = false;
	m->stopped = true;
	flush_mQueue(m, 0);
}

void
motor_disable_fast (struct motor *m)
{	
	assert (m != NULL);
	struct CO_message msg = {RPDO, .m.PDO = {1, 0x0000, 2}};		// why was data 0x0006?
	CO_send_message (m->s, m->no, &msg);
	m->enabled = false;
	m->stopped = true;
	flush_mQueue(m, 0);	
}

/*
 * Stop the motor
 */
void
motor_stop (struct motor *m)
{
  	assert (m != NULL);
	m->stopped = true;
	struct event e;
	unsigned short n = m->fault? 0 : 4;	// if fault, no waiting on response. communication reset will handle it.
	struct CO_message msg = {SDO_Rx, .m.SDO = {CONTROL_WORD, 0x00, 0x07, 2}};
	CO_send_message (m->s, m->no, &msg);
	struct itimerspec itmr = {{0}};
	itmr.it_value.tv_sec = MSG_TIMEOUT;
	timer_settime (m->msg_timer, 0, &itmr, NULL);
	while (n)
	{
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
		if (e.type == TIMEOUT)
		{
			puts ("msg timer timeout, motor_stop failed, retrying...\r\n");
			n--;
			CO_send_message (m->s, m->no, &msg);
			timer_settime (m->msg_timer, 0, &itmr, NULL);
		}
		else if (e.type == STATUS_WRD_REC && (e.param & ENABLE_Rx_MASK) <= 0x0023){
			break;
		}
	}
	itmr.it_value.tv_sec = 0;
	timer_settime (m->msg_timer, 0, &itmr, NULL);
	flush_mQueue(m, 0);
}

void
motor_start (struct motor *m)
{
  	assert (m != NULL);
	if (m->enabled){
		struct event e;
		unsigned short n = 2;
		struct CO_message msg = {SDO_Rx, .m.SDO = {CONTROL_WORD, 0x00, 0x0F, 2}};
		CO_send_message (m->s, m->no, &msg);			// First step
		struct itimerspec itmr = {{0}};
		itmr.it_value.tv_sec = MSG_TIMEOUT;
		timer_settime (m->msg_timer, 0, &itmr, NULL);
		while (n)
		{
			mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
			if (e.type == TIMEOUT)
			{
				puts ("msg timer timeout, motor_start failed, retrying...\r\n");
				n--;
				CO_send_message (m->s, m->no, &msg);
				timer_settime (m->msg_timer, 0, &itmr, NULL);
			}
			else if (e.type == STATUS_WRD_REC && (e.param & ENABLE_Rx_MASK) == 0x0027){
				if (msg.m.SDO.data == 0x1F){			// Last step
					itmr.it_value.tv_sec = 0;
					timer_settime (m->msg_timer, 0, &itmr, NULL);
					break;
				} else {
					msg.m.SDO.data = 0x1F;				// Second step
					CO_send_message (m->s, m->no, &msg);
					timer_settime (m->msg_timer, 0, &itmr, NULL);
				}
			}
		}
		flush_mQueue(m, 0);
		itmr.it_value.tv_sec = HEARTBEAT_TIMEOUT;
		timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
		m->stopped = false;
	}
	else
		motor_enable(m);
}


/*
 * Destroy the motor object
 */
void motor_destroy (struct motor *m)
{
	if (m == NULL)
		return;

	motor_disable (m);
	m->ok = false;
	int status = pthread_kill(m->listener, SIGINT);
	if (status < 0)
		perror("pthread_kill motor thread failed");
	else
		printf("Stopped thread for Motor %i\r\n", m->no);
	timer_delete (m->msg_timer);
	timer_delete (m->heartbeat_timer);
	mq_close (m->mQueue);
	shutdown (m->s, 1);
	free (m->mQueue_name);
	//close(m->s);
	free (m);
}

bool
is_motor_enable_pin_active(struct motor *m){
	return m->enable_pin_active;
}

/*------------------------------static functions------------------------------*/
/*
 * The timer handler for the msg timer, sends a timeout event to the message
 * queue
 */
static void
msg_timer_handler (union sigval val)
{
	struct motor *m = val.sival_ptr;
	struct event e;
	e.type = TIMEOUT;
	e.param = (uintptr_t)m->msg_timer;
	mq_send (m->mQueue, (char *)&e, sizeof (e), 0);
	printf("MQueueERROR: TIME OUT\r\n");
}

/*
 * The timer handler for the msg timer, raises a SIGINT signal to signal
 * shutdown
 */
static void
heartbeat_timer_handler (union sigval val)
{
	struct motor *m = val.sival_ptr;
	if (!m->stopped){
		struct itimerspec itmr = {{0}};
		itmr.it_value.tv_sec = 0;
		timer_settime (m->msg_timer, 0, &itmr, NULL);
		timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
		m->fault = true;
		// Disable all motors EXCEPT the unresponsive one.
		raise (SIGTSTP);
		// Reset communication
		struct event e;
		printf ("Motor controller %d heartbeat timeout, resetting\r\n", m->no);
		/*
		struct CO_message msg_nmt = {NMT, .m.NMT = {0x80}};
		CO_send_message (m->s, m->no, &msg_nmt);
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
		printf ("Resetting... 20%% Motor response: type %x, param %x\r\n", e.type, e.param);
		msg_nmt.m.NMT.data = 0x01;
		CO_send_message (m->s, m->no, &msg_nmt);
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
		printf ("Resetting... 40%% Motor response: type %x, param %x\r\n", e.type, e.param);
		struct CO_message msg = {SDO_Rx, .m.SDO = {CONTROL_WORD, 0x00, 0x00, 2}};
		CO_send_message (m->s, m->no, &msg);
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
		printf ("Resetting... 60%% Motor response: type %x, param %x\r\n", e.type, e.param);
		msg.m.SDO.data = 0x06;
		CO_send_message (m->s, m->no, &msg);
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
		printf ("Resetting... 80%% Motor response: type %x, param %x\r\n", e.type, e.param);
		msg.m.SDO.data = 0x07;
		CO_send_message (m->s, m->no, &msg);
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
		printf ("Resetting...100%% Motor response: type %x, param %x\r\n", e.type, e.param);
		*/
		flush_mQueue(m, 0);
        /*
         * TODO: Sometimes will not work... needs a full reset.
		struct CO_message msg_nmt = {NMT, .m.NMT = {0x02}};
		CO_send_message (m->s, m->no, &msg_nmt);
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
		printf ("Resetting... 50%% Motor response: type %x, param %x\r\n", e.type, e.param);
		msg_nmt.m.NMT.data = 0x01;
		CO_send_message (m->s, m->no, &msg_nmt);
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
		printf ("Resetting... 100%% Motor response: type %x, param %x\r\n", e.type, e.param);
		printf ("Motor controller %d reset completed!\r\n", m->no);
		*/
        enum ctrl_mode cm = m->cm;
        // TODO: are these sufficient?
        while (m->fault){
			if (init_motor(m)!=0)
				continue;
			if (home_motor(m)!=0)
				continue;
			m->fault = false;
		}
        motor_set_ctrl_mode(m, cm);
        
		//if (m->enabled){
		//	motor_enable(m);
		//}
		// Enable all motors
		raise (SIGCONT);
		itmr.it_value.tv_sec = HEARTBEAT_TIMEOUT;
		timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
	}
}


/*
 * Functions for unit conversion. These output values at the motor shaft.
 * Gear ratios included at caster level.
 * Position SI = rad
 * Position IU = encoder ticks
 * Velocity SI = rad/s
 * Velocity IU = encoder ticks/slow loop sampling period
 * Torque SI = Nm
 * Torque IU = Motor phase current (A)
 */
static inline int32_t
position_SI_to_IU (double position_SI)
{
	return (int32_t) (position_SI * POS_MULTIPLIER *
		(Ns * ENCODER_TICKS / TWO_PI));
}

static inline double
position_IU_to_SI (int32_t position_IU)
{
	return (double) (position_IU *
		(TWO_PI / (Ns * ENCODER_TICKS)) / POS_MULTIPLIER);
}

static inline int32_t
velocity_SI_to_IU (double velocity_SI)
{
	return (int32_t) (velocity_SI * VEL_MULTIPLIER *
		(Ns * ENCODER_TICKS * SLOW_LOOP_SAMP_PERIOD / TWO_PI));
}

static inline double
velocity_IU_to_SI (int32_t velocity_IU)
{
	return  (double) velocity_IU * (TWO_PI / (Ns * ENCODER_TICKS *
		SLOW_LOOP_SAMP_PERIOD)) / VEL_MULTIPLIER;
}

static inline int32_t
accel_SI_to_IU (double accel_SI)
{
	return (int32_t) (accel_SI * ACCEL_MULTIPLIER * (Ns *
		ENCODER_TICKS * SLOW_LOOP_SAMP_PERIOD * SLOW_LOOP_SAMP_PERIOD / TWO_PI));
}

static inline double
accel_IU_to_SI (int32_t accel_IU)
{
	return (double) (accel_IU * (TWO_PI / (Ns * ENCODER_TICKS *
		SLOW_LOOP_SAMP_PERIOD * SLOW_LOOP_SAMP_PERIOD)) / ACCEL_MULTIPLIER);
}

static inline int16_t
torque_SI_to_IU (double torque_SI)
{
	return (int16_t) (torque_SI * CURRENT_DENOM /
		(TORQUE_CONSTANT * CURRENT_NUM * CURRENT_PEAK));
}

static inline double
amp_to_torque_SI (double amp_SI)
{
	return (double) (amp_SI * TORQUE_CONSTANT);
}

static inline double
amp_IU_to_SI (int16_t amp_IU)
{
	return (double) (amp_IU * CURRENT_NUM * CURRENT_PEAK / CURRENT_DENOM);
}

static inline double
volt_IU_to_SI (int16_t volt_IU)
{
	return (double) (volt_IU * VOLTAGE_NUM / VOLTAGE_DENOM);
}

/*
 * Initialize the message queue that is used for communication between the
 * listening thread and the thread that calls the motor API
 * To avoid using sudo for robot control, add the lines in /etc/sysctl.conf:
 * # increase message queue size
 * fs.mqueue.msg_max = 100
 */
static int
init_mQueue (struct motor *m)
{
	m->mQueue_name = malloc (MAX_NAME_LEN);
	if (m->mQueue_name == NULL)
	{
		perror ("call to malloc failed in init_mQueue\n");
		return -1;
	}

	sprintf(m->mQueue_name, "/motor %u message queue", m->no);
	mq_unlink (m->mQueue_name);
	struct mq_attr attr;
	attr.mq_maxmsg = QUEUE_SIZE;
	attr.mq_msgsize = sizeof (struct event);
	attr.mq_flags = 0;
	m->mQueue = mq_open (m->mQueue_name, O_RDWR | O_CREAT | O_CLOEXEC , 0664, &attr);	// Added O_CLOEXEC to prevent race conditions

	if (m->mQueue == -1)
	{
		perror ("failed to open message queue in init_mQueue\n");
		return -1;
	}

	return 0;
}

static void
flush_mQueue (struct motor *m, unsigned int threshold)
{
	struct mq_attr attr;
	if (!mq_getattr(m->mQueue, &attr)){
		struct event e;
		while (attr.mq_curmsgs > threshold) {
			mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
			attr.mq_curmsgs --;
		}
	}
}

/*
 * Implementation of an exponential moving average filter for the motor
 * velocity or torque measurement
 */
static inline double
filter (double old_vel, double new_sample, double coefficient)
{
	return (coefficient * old_vel + (1.0 - coefficient) * new_sample);
}


/*
 * This is the listener thread, it constantly reads from the CAN socket and
 * translates the frames into events that it sends to the message queue to
 * other threads that might be interested in the events
 */
static void *
listener (void *aux)
{
	struct motor *m = aux;
	struct event e;
	struct can_frame f;
	struct itimerspec itmr = {{0}};
	itmr.it_value.tv_sec = HEARTBEAT_TIMEOUT;
	printf("Motor listener thread launched!\r\n");
	
	uint16_t *data_u16;
	int16_t *data_16;
	int32_t *data_32;

	/* listen for messages forever, thread gets cancelled on motor_destroy call */
	while (m->ok) {
	  	if (read(m->s, &f, sizeof(struct can_frame)) >= 0) {
			//printf("Can ID: %X\n", f.can_id);
		  	/* translate the can frame */
	  		switch (f.can_id - m->no){
	  			case COB_ID_NMT_EC_TX_BASE:   /* NMT */
	  				if (f.data[0] == 0x05) {    /* heartbeat in operational state */
			  			/* restart the heartbeat timer */
			  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
			  		} else if (f.data[0] == 0x00) {   /* bootup */
			  			e.type = NMT_EC_REC;
			  			e.param = f.data[0];
			  			mq_send (m->mQueue, (char *)&e, sizeof (e), 0);
						//printf("MQueue Info - Motor NMT Bootup Event Sent\r\n");
			  		}
			  		break;
			  	case COB_ID_TPDO1_BASE:       /* TPDO1 - status word*/
			  		/* restart the heartbeat timer -- DIRTY FIX TO PREVENT TIMEOUT, NOT SECURED*/
		  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
					data_u16 = (uint16_t *)&f.data;
			  		e.type = STATUS_WRD_REC;
			  		e.param = data_u16[0];
			  		mq_send (m->mQueue, (char *)&e, sizeof (e), 0);
					//printf("MQueue Info - Motor TPDO1 Status Word Rec Event Sent\r\n");
			  		break;
			  	case COB_ID_TPDO2_BASE:       /* TPDO2 - {pos, vel/trq}*/
			  		data_32 = (int32_t *)&f.data; // why not uint32_t
			  		/* restart the heartbeat timer -- DIRTY FIX TO PREVENT TIMEOUT, NOT SECURED*/
		  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
			  		/* critical section */
			  		pthread_mutex_lock (&m->lock);
			  		double last_pos = m->cur_pos;
			  		m->cur_pos = position_IU_to_SI (data_32[0]);
					m->stale_pos = false;
					/* translate the last 4 bytes of data into vel */
					m->cur_vel = (last_pos == m->cur_pos)? 
								0. : filter(m->cur_vel, velocity_IU_to_SI (data_32[1]), LP_VEL_FILTER_COEFF);
					m->stale_vel = false;
					pthread_mutex_unlock (&m->lock);
					/* end of critical section */
					break;
				case COB_ID_TPDO3_BASE:       /* TPD03 - {current, modes of operation */
					data_16 = (int16_t *)&f.data; // why not uint16_t?
					/* restart the heartbeat timer -- DIRTY FIX TO PREVENT TIMEOUT, NOT SECURED*/
		  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
					//printf("data[0]: %X\ndata[1]: %X\n\n",data[0],data[1]);
					pthread_mutex_lock (&m->lock);
					m->cur_amp = amp_IU_to_SI (data_16[0]);
					m->cur_voltage = volt_IU_to_SI (data_16[1]);
					m->cur_trq = filter (m->cur_trq, amp_to_torque_SI (m->cur_amp), LP_TRQ_FILTER_COEFF);
					m->stale_trq = false;
					pthread_mutex_unlock (&m->lock);
					break;
				case COB_ID_TPDO4_BASE:       /* TPD04 - digital inputs */
					data_32 = (int32_t *)&f.data; // why not uint32_t?
					/* restart the heartbeat timer -- DIRTY FIX TO PREVENT TIMEOUT, NOT SECURED*/
		  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
					/* critical section */
			  		pthread_mutex_lock (&m->lock);
			  		m->inputs = data_32[0];
			  		pthread_mutex_unlock (&m->lock);
			  		/* end of critical section */
			  		break;
		  		case COB_ID_SDO_TX_BASE:      /* SDO */
		  			if (f.data[0] == CO_WRITE_Ack) {
			  			/* send an SDO_WR_ACK event and save the object index in param */
			  			e.type = SDO_WR_ACK;
			  			e.param = *(uint16_t *)&(f.data[1]);
			  			mq_send (m->mQueue, (char *)&e, sizeof (e), 0);
						//printf("MQueue Info - Motor SDO Word Ack Event Sent\r\n");
			  		}
					/* restart the heartbeat timer -- DIRTY FIX TO PREVENT TIMEOUT, NOT SECURED*/
		  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
		  			break;
	  			case COB_ID_EMCY_TX_BASE:     /* Emergency message */
	  				/* restart the heartbeat timer -- DIRTY FIX TO PREVENT TIMEOUT, NOT SECURED*/
		  			timer_settime (m->heartbeat_timer, 0, &itmr, NULL);
					if ((f.data[0] == 0x41) && (f.data[1] == 0x54)) {
						m->enable_pin_active = false;
						printf("Motor %d enable pin DISABLED\n", m->no);
					} else if ((f.data[0] == 0x00) && (f.data[1] == 0x00)) {
						printf("Error reset or no error \n");
						m->enable_pin_active = true;
					} else {
						printf("Emergency message received from Motor %d\n", m->no);
						printf("Data: %X, %X \n", f.data[1], f.data[0]);
						//while(1) {}
					}
					break;
				default:
					break;
	  		}
			//flush_mQueue(m, QUEUE_HIGH_WATER_MARK);
		} else {
			perror ("can raw socket read\n");
			exit (-1);
		}
	}
}


/*
 * Initializes the motor, this function steps through the array of messages
 * inside of motor_init_sequence.h. It sends a message and then waits for
 * an event verifying success of that transmission. If at any point a timeout
 * occurs, this is translated as an error and the function returns -1
 */
static int
init_motor (struct motor *m)
{
	struct event e;
	struct itimerspec itmr = {{0}};
	itmr.it_value.tv_sec = MSG_TIMEOUT;
	
	/* send first message to kick off */
	unsigned i = 0;
	CO_send_message (m->s, m->no, &act_init_sequence[i]);
	timer_settime (m->msg_timer, 0, &itmr, NULL);

	while (i < NUM_ACT_INIT_STEPS)
	{
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
		if (e.type == TIMEOUT)
		{
			printf("Motor %d init timer timeout, init_motor failed\r\n", m->no);
			return -1;
		}
		else if (e.type == act_init_responses[i].type && e.param == act_init_responses[i].param)
		{
			if (++i < NUM_ACT_INIT_STEPS)
			{
				struct CO_message cur = act_init_sequence[i];

				/* make sure to add the motor number to the data field for PDO inits
				   and home offsets */
				if (cur.type == SDO_Rx && cur.m.SDO.index == HOME_OFFSET)
				{
					cur.m.SDO.data = get_cal_offset (m);
				}

				CO_send_message (m->s, m->no, &cur);
				timer_settime (m->msg_timer, 0, &itmr, NULL);
			}
		}
		else {
			printf("Response error | ");
			printf("Type: %X, Param: %X\n", e.type, e.param);
			printf("Expected vals  | ");
			printf("Type: %X, Param: %X\n", act_init_responses[i].type, act_init_responses[i].param);
		}
	}

	/*Repeat for communication setup*/
	i = 0;
	CO_send_message (m->s, m->no, &comm_init_sequence[i]);
	timer_settime (m->msg_timer, 0, &itmr, NULL);
	while (i < NUM_COMM_INIT_STEPS)
	{
		mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);
		if (e.type == TIMEOUT)
		{
			printf("Motor %d init timer timeout, init_motor failed\r\n", m->no);
			return -1;
		}
		else if (e.type == comm_init_responses[i].type && e.param == comm_init_responses[i].param)
		{
			if (++i < NUM_COMM_INIT_STEPS)
			{
				struct CO_message cur = comm_init_sequence[i];

				/* make sure to add the motor number to the data field for PDO inits
				   and home offsets */
				if (cur.type == SDO_Rx)
				{
					//printf("Writing to index %X with subindex %X, %X\n", cur.m.SDO.index, cur.m.SDO.subindex, cur.m.SDO.data);
					switch (cur.m.SDO.index)
					{
						case RPDO1_COMM:
						case RPDO2_COMM:
						case RPDO3_COMM:
						case RPDO4_COMM:
						case TPDO1_COMM:
						case TPDO2_COMM:
						case TPDO3_COMM:
						case TPDO4_COMM:
							if (cur.m.SDO.subindex == 0x01)
								cur.m.SDO.data += m->no;
							break;
						default:
							break;
					}
				}

				CO_send_message (m->s, m->no, &cur);
				timer_settime (m->msg_timer, 0, &itmr, NULL);
			}
		}
		else {
			printf("Response error | ");
			printf("Type: %X, Param: %X\n", e.type, e.param);
			printf("Expected vals  | ");
			printf("Type: %X, Param: %X\n", comm_init_responses[i].type, comm_init_responses[i].param);
		}
	}

	/* stop timer */
	itmr.it_value.tv_sec = 0;
	timer_settime (m->msg_timer, 0, &itmr, NULL);
	flush_mQueue(m, 0);
	return 0;
}


/*
 * Homes the motor by setting the control mode field to HOMING and calling
 * enable, then waits for a STATUS_WORD with data 0xD637 signaling that
 * homing is finished
 */
static int
home_motor (struct motor *m)
{
	/* first enable the motor */
	if (motor_get_type(m) == STEERING){
		enum ctrl_mode old = motor_get_ctrl_mode(m);
		motor_set_ctrl_mode(m, HOMING);
		motor_enable (m);
		/* perform the homing sequence */
		struct event e;
		struct itimerspec itmr = {{0}};
		itmr.it_value.tv_sec = HOME_TIMEOUT;
		struct CO_message msg = {SDO_Rx, .m.SDO = {CONTROL_WORD, 0x00, 0x001F, 2}};	/* start homing move */
		CO_send_message (m->s, m->no, &msg);
		timer_settime (m->msg_timer, 0, &itmr, NULL);
		while (1)
		{
			mq_receive (m->mQueue, (char *)&e, sizeof (e), NULL);

			if (e.type == TIMEOUT)
			{
				puts ("msg timer timeout, home_motor failed\r\n");
				return -1;
			}
			if (e.type == STATUS_WRD_REC && e.param == 0xD637)
				break;
		}

		/* stop timer */
		itmr.it_value.tv_sec = 0;
		timer_settime (m->msg_timer, 0, &itmr, NULL);
		flush_mQueue(m, 0);
		motor_stop (m);
		motor_set_ctrl_mode(m, old);
	}
	else{
		motor_enable (m);
		/* stop motor */
		motor_stop (m);
	}
	flush_mQueue(m, 0);
	return 0;
}


/*
 * Returns the proper home offset to write during motor initialization.
 * If the calibration routine was run and the file exists with the offsets,
 * those are factored in to the value
*/
static int32_t
get_cal_offset (struct motor *m)
{
	double retVal, calVal, difference;
	char *buffer = NULL;
	size_t n;
	FILE *fp;

	/* assign initial offset, steer motors have offset of zero */
	if (m->mt == ROLLING)
		return 0;

	retVal = home_offsets[m->no/2];

	/* try to open motor_cal file */
	fp = fopen ("./.motor_cal.txt", "r");
	if (fp != NULL)
	{
		/* cal value is positioned by line in .motor_cal
			 i.e. cal of motor 1 = line 1
				  cal of motor 3 = line 2
				  cal of motor 5 = line 3 .... */
		int i;
		for (i = 0; i <= (m->no / 2); i++)
			getline (&buffer, &n, fp);

		calVal = atof (buffer);

		free (buffer);
		fclose (fp);

		/* calculate the offset value and add it to the normal offset */
		difference = retVal + calVal;
		retVal += difference;
	}

	return position_SI_to_IU (retVal * Ns);
}

/*---------------------------------test harness-------------------------------*/
#ifdef TEST_MOTOR

//#define LOG_COUPLED_MOTOR // if uncommented then intialize joint motor and log values

#define CONTROL_PERIOD_ns 	(700000)
#define CONTROL_PERIOD_s 	(0.007)
#define MAX_VEL				(7*3.14159)		/* [rad/s] */
#define MAX_TOR             (2.0)         /* [Nm] */
#define X_freq				(0.1)					/* [hz] */
#define MAX_VEL_IU			(0x100000)

static void * control_thread (void *aux);
static bool done = false;
static int test_mode = 0;

#ifdef LOG_COUPLED_MOTOR
static void * log_thread (void *aux);
#endif

void
main(char argc, char *argv[])
{
	if (argc < 4)
	{
		puts ("Usage: sudo ./test <motor_no> <test_mode> <duration>");
		exit (0);
	}

	/* prompt to elevate the robot for testing */
	puts("Begin motor test harness");
	puts ("Elevate the robot so that all casters can spin freely");
	puts ("Press any key to continue when done");
	getchar ();

	/* Test unit conversion */
	// puts("\nPos IU\t\t\tPos SI\t\t\tPos_IU (reconverted)");
	// for(int pos_IU = -49000; pos_IU < 49000; pos_IU += 500){
	// 	double pos_SI = position_IU_to_SI(pos_IU);
	// 	printf("%d\t\t\t%.4f\t\t\t%d\n",pos_IU,pos_SI,position_SI_to_IU(pos_SI));
	// }

	// puts("\nVel IU\t\t\tVel SI\t\t\tVel_IU (reconverted)");
	// for(int vel_IU = -4700000; vel_IU < 4700000; vel_IU += 20000){
	// 	double vel_SI = velocity_IU_to_SI(vel_IU);
	// 	printf("%d\t\t\t%.4f\t\t\t%d\n",vel_IU,vel_SI,velocity_SI_to_IU(vel_SI));
	// }

	// puts("\nAccel IU\t\t\tAccel SI\t\t\tAccel_IU (reconverted)");
	// for(int accel_IU = -23000; accel_IU < 23000; accel_IU += 100){
	// 	double accel_SI = accel_IU_to_SI(accel_IU);
	// 	printf("%d\t\t\t%.4f\t\t\t%d\n",accel_IU,accel_SI,accel_SI_to_IU(accel_SI));
	// }

	// puts("\nTorque IU\t\t\tTorque SI\t\t\tTorque_IU (reconverted)");
	// for(int torque_IU = -35000; torque_IU < 35000; torque_IU += 100){
	// 	double torque_SI = torque_IU_to_SI(torque_IU);
	// 	printf("%d\t\t\t%.4f\t\t\t%d\n",torque_IU,torque_SI,torque_SI_to_IU(torque_SI));
	// }

  // Set motor number, test mode, and test duration from arguments
	int motor_no = atoi (argv[1]);
	test_mode = atoi (argv[2]);
	int duration = atoi (argv[3]);

	if(duration <= 0) {
		puts ("Test duration must be a positive value");
		exit(0);
	}

	#ifdef TEST_VELOCITY
	struct motor *m = motor_init (motor_no, VELOCITY, motor_no % 2);
	home_motor(m);
	m->cm = VELOCITY;  // Home motor changes command mode to HOMING, must be changed back to VELOCITY
	#endif

	#ifdef TEST_TORQUE
	struct motor *m = motor_init (motor_no, TORQUE, motor_no % 2);
	// home_motor(m);
	m->cm = TORQUE;  // Home motor changes command mode to HOMING, must be changed back to TORQUE
	#endif

	if (m == NULL)
	{
		puts ("motor_init failed");
		goto done;
	}

	#ifdef LOG_COUPLED_MOTOR
	int log_motor_no = 4 * ((motor_no + 1) / 2) - 1 - motor_no;
	struct motor *m_log = motor_init(log_motor_no, VELOCITY, log_motor_no % 2);

	if(m_log == NULL) {
		puts("motor_init failed for log motor");
		goto done;
	}
	#endif

	if(motor_enable (m))
	{
		puts ("motor_enable failed");
		goto done;
	}

	#ifdef LOG_COUPLED_MOTOR
	if(motor_enable(m_log)) {
		puts("motor_enable failed for log motor");
		goto done;
	}
	#endif

	/* Launch control thread */
	pthread_t control;
	launch_rt_thread (control_thread, &control, m, MAX_PRIO);

	#ifdef LOG_COUPLED_MOTOR
	pthread_t log;
	launch_rt_thread(log_thread, &log, m_log, MAX_PRIO);
	#endif

	/* sleep for some time and then stop control thread */;
	sleep (duration);
	done = true;
	pthread_join (control, NULL);

	#ifdef LOG_COUPLED_MOTOR
	pthread_join(log, NULL);
	#endif

	done:
		motor_destroy (m);
		#ifdef LOG_COUPLED_MOTOR
		motor_destroy(m_log);
		#endif
		puts("");
		puts("End motor test harness");
}

static void
sleep_until (struct timespec *ts, long delay)
{
	ts->tv_nsec += delay;
	if (ts->tv_nsec >= 1000*1000*1000)
	{
		ts->tv_nsec -= 1000*1000*1000;
		ts->tv_sec++;
	}
	clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts,  NULL);
}

static void *
control_thread (void *aux)
{
	printf("control thread launched with success\n");
	struct motor *m = aux;
	int s = create_can_socket (0xF, 0xF);
	struct CO_message msg;
	msg.type = SYNC;

	struct timespec next;
	clock_gettime(CLOCK_MONOTONIC, &next);

	/* open dump file */
	
	FILE *fp;
	fp = fopen("./traces/trace.csv", "w");
	fprintf(fp, "t,vel-command,tor-command,pos-actual,vel-actual,tor-actual\n");

	if (fp == NULL){
		printf("file could not be opened");
		return 0;
	}

	/* variables for sin, cos waves */
	unsigned long ticks = 0;
	double vel_command = 0, tor_command = 0, vel_actual, pos_actual, tor_actual, prev_pos = 0, prev_prev_pos = 0;
	int32_t vel_command_iu = 0, vel_actual_iu, pos_actual_iu;

	/* send initial synch message */
	CO_send_message (s, 0, &msg);
	sleep_until(&next, CONTROL_PERIOD_ns/2);

	while (!done)
	{
	/* ---------- first half of control loop ---------- */
		/* send sync message */
		CO_send_message (s, 0, &msg);
		usleep (3000);

		motor_get_velocity (m, &vel_actual);
		motor_get_position (m, &pos_actual);
		motor_get_torque (m, &tor_actual);

		#ifdef TEST_VELOCITY
		motor_set_velocity (m, vel_command);
		#endif

		#ifdef TEST_TORQUE
		motor_set_torque (m, tor_command);
		#endif

		sleep_until (&next, CONTROL_PERIOD_ns/2);
	/* --------- second half of control loop --------- */
		/* send sync message */
		CO_send_message (s, 0, &msg);

	 	/* write to file */
	 	fprintf(fp, "%f,%f,%f,%f,%f,%f\n", (ticks*(CONTROL_PERIOD_s/2)), vel_command, tor_command, pos_actual, vel_actual, tor_actual);

		vel_command = 1;
		tor_command = 1;
	 	/* calculate next */
		switch(test_mode) {
		case 218: //balance
			tor_command = 0.974262;
		case 0: // Half speed
			vel_command = 0.5;
		case 1: // Full speed
			vel_command *= MAX_VEL;
			break;
		case 2: // Full speed sine wave
			vel_command = MAX_VEL;
		case 3: // Low speed sine wave
			vel_command *= -sin(2*PI*X_freq*(ticks*CONTROL_PERIOD_s));
			break;
		case 4: // Full speed square wave
			vel_command = MAX_VEL;
		case 5: // Low speed square wave
			vel_command *= (2 * signbit(sin(2*PI*X_freq*(ticks*CONTROL_PERIOD_s))) - 1);
			break;
		case 6: // Very slow constant speed
			vel_command = 2;
			break;
		case 100: // Half torque
			tor_command = 0.5;
		case 101: // Full torque
			tor_command *= MAX_TOR;
			break;
		case 102: // Super low sine wave
			tor_command = MAX_TOR * sin(2*PI*X_freq*(ticks*CONTROL_PERIOD_s));
			break;
		case 103: // Square wave
			tor_command = MAX_TOR*(2 * signbit(sin(2*PI*X_freq*(ticks*CONTROL_PERIOD_s))) - 1);
			break;
		default:
			puts("illegal test mode");
			vel_command = 0;
			break;
		}

		if(vel_command > MAX_VEL) {
			vel_command = MAX_VEL;
		} else if(vel_command < -MAX_VEL) {
			vel_command = -MAX_VEL;
		}

		if(tor_command > MAX_TOR) {
			tor_command = MAX_TOR;
		} else if(tor_command < -MAX_TOR) {
			tor_command = -MAX_TOR;
		}

		ticks++;

	  /* do nothing in this half */
		sleep_until (&next, CONTROL_PERIOD_ns/2);
	}
}

#ifdef LOG_COUPLED_MOTOR
static void *
log_thread (void *aux) {
	printf("log thread launched with success\n");
	struct motor *m_log = aux;
	int s = create_can_socket(0xF, 0xF);
	struct CO_message msg;
	msg.type = SYNC;

	struct timespec next;
	clock_gettime(CLOCK_MONOTONIC, &next);

	FILE *fp;
	fp = fopen("./traces/trace_log.csv", "w");
	fprintf(fp, "t,vel,pos\n");

	unsigned long ticks = 1;
	double pos_actual, vel_actual;

	CO_send_message(s, 0, &msg);
	sleep_until(&next, CONTROL_PERIOD_ns/2);

	while(!done) {
		CO_send_message(s, 0, &msg);
		usleep(3000);
		sleep_until(&next, CONTROL_PERIOD_ns/2);
		CO_send_message(s, 0, &msg);

		motor_get_velocity(m_log, &vel_actual);
		motor_get_position(m_log, &pos_actual);

		fprintf(fp, "%f,%f,%f\n", (ticks*CONTROL_PERIOD_s), vel_actual, pos_actual);
		ticks++;

		sleep_until(&next, CONTROL_PERIOD_ns/2);
	 }
}
#endif

#endif

/* cpp - c cross compilation */
