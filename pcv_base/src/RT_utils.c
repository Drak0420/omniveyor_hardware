/*-----------------------------includes--------------------------------------*/
/* cpp - c cross compilation */
#ifdef __cplusplus
extern "C" {
#endif

//#define _GNU_SOURCE
#include <assert.h>
#include <pthread.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sched.h>

#include "../include/RT_utils.h"

/*-----------------------------defines---------------------------------------*/
#define MY_STACK_SIZE   			(100*1024) 

/*-----------------------------structs---------------------------------------*/

/*-----------------------static function declarations------------------------*/

/*---------------------------public functions--------------------------------*/
int 
launch_rt_thread (void *(*func)(void *), pthread_t *t, void *aux, int priority)
{
	assert (t != NULL);
	pthread_attr_t attr;
	cpu_set_t cpuset;
	struct sched_param param;

	// It could be desireable to run the thread on an isolated core.
	//CPU_ZERO(&cpuset);
	//unsigned short ncpu = get_nprocs();
        //for (int j = 0; j < 1; j++)
        //       CPU_SET(ncpu-1, &cpuset);

	/*
	To NOT run the code as sudo, add the limits in /etc/security/limits.conf:
	username         hard    memlock         524288
	username         hard    priority        85
	username         hard    rtprio          85
	username         soft    rtprio          85
	**use ulimit -l to check the current memory lock size limit. may need to increase this.
	*/

	/* Lock memory */
	if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1) 
    exit(-2);

	/* Initialize pthread attributes (default values) */
	if (pthread_attr_init (&attr))
    return -1;

	/* Set a specific stack size  */
	if (pthread_attr_setstacksize (&attr, MY_STACK_SIZE))
    return -1;

	/* Set scheduler policy and priority of pthread */
	if (pthread_attr_setschedpolicy (&attr, SCHED_FIFO))
    return -1;

	param.sched_priority = priority; 
	//printf("%i",sched_get_priority_max(SCHED_FIFO));
	if (pthread_attr_setschedparam (&attr, &param))
    return -1;

	/* Use scheduling parameters of attr */
	if (pthread_attr_setinheritsched (&attr, PTHREAD_EXPLICIT_SCHED))
	return -1;

	//if (pthread_attr_setaffinity_np (&attr, sizeof(cpuset), &cpuset))
	//  return -1;

	/* Create a pthread with specified attributes */
	if (pthread_create(t, &attr, func, aux))
    return -1;

  return 0;
}

int
init_rt_timer (timer_t *t, void (*handler)(union sigval val), void *aux)
{
	pthread_attr_t attr = {{0}};
    pthread_attr_init (&attr);
	struct sched_param parm;
    parm.sched_priority = 255;
	pthread_attr_setschedparam (&attr, &parm);

	struct sigevent sev = {{0}};
	sev.sigev_notify = SIGEV_THREAD;
	sev.sigev_value.sival_ptr = aux;
	sev.sigev_notify_function = handler;

	if (timer_create (CLOCK_REALTIME, &sev, t))
	{
		perror ("failed to create timer\n");
		return -1;
	}

	return 0;
}


/*---------------------------static functions--------------------------------*/

/* cpp - c cross compilation */
#ifdef __cplusplus
} // closing brace for extern "C"
#endif
