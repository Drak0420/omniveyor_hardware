/*-----------------------------includes--------------------------------------*/
#include <fstream>
#include <iostream>
#include <math.h>                                   /* sin, cos */
#include <signal.h>
#include <string>
#include <unistd.h>
#include "../include/motor.h"
#include "../include/CO_message.h"
#include "../include/CAN_utils.h"
#include "../include/RT_utils.h"
#include "../include/vehicle.h"
#include "../include/caster.h"

using namespace std;

/*-----------------------------defines---------------------------------------*/
 // #define TEST_MOTOR  // Uncomment to test motor, comment to test caster
#define TEST_CASTER // Uncomment to test caster, comment to test motor
/*-----------------------------structs---------------------------------------*/

/*-----------------------static function declarations------------------------*/
static void * print_thread_rt (void *aux);
static void * control_thread (void *aux);

/*-----------------------static variable declarations------------------------*/
static uint8_t can_id;
static uint8_t duration = 5;
static int s;
static bool done = false, initialized = false, enabled = false, running = false, destroyed = false;
static float cthread_start = 0;
static uint8_t caster_no;
static double torque_des;
static Caster *caster;

struct motor *m;

/*--------------------------public functions---------------------------------*/
/* 
 * This program was useful for printing the raw can data to the terminal. It
 * takes one input which is the motor to listen to (0 listens to all motors)
 */
#ifdef TEST_MOTOR
int
main (int argc, char *argv[])
{
    /* print usage if error */
    if (argc < 2)
    {
        cout << "usage: ./canDump_motor <motor number>\n";
        return 0;
    }
    /* motor_no to track passed in command line */
    can_id = atoi (argv[1]);
    printf("Can ID: %X\n", can_id);

    /* open socket to retrieve messages */
    uint16_t mask = (can_id == 0) ? (0x7F0) : (0x7FF);
    s = create_can_socket (can_id + 0x280, 0x000); 
    pthread_t t;
    launch_rt_thread (print_thread_rt, &t, NULL, 80);
    
    printf("Calling motor init\n");
    m = motor_init (can_id, TORQUE, (motor_type) (can_id % 2));
    // while(!initialized){}

    printf("Calling motor enable\n");
    motor_enable(m);
    // while(!enabled){}
    enabled = true;

    printf("Launching control thread\n");
    pthread_t control;
    launch_rt_thread (control_thread, &control, NULL, MAX_PRIO);
    running = true;
    while(running){}    

    printf("Destroying motor\n");
    motor_destroy(m);
    while(!destroyed){}  
    
    printf("Done\n");
    done = true;
    return 0;
}
#endif

#ifdef TEST_CASTER
int 
main (int argc, char *argv[])
{
    printf("running test caster \n");
    /* print usage if error */
    // if (argc < 2)
    // {
    //     cout << "usage: ./canDump_motor <caster number> <torque desired>\n";
    //     return 0;
    // }
    /* caster_no to track passed in command line */
    caster_no = atoi (argv[1]);
    printf("Caster number: %X\n", caster_no);
    // get motor_no for steer and roll motors
    uint8_t steer_motor_no = 2*caster_no - 1;
    uint8_t roll_motor_no = 2*caster_no;

    // We set the can_id to the steer motor since that's the one we're concerned with
    // for torque testing. 
    can_id = steer_motor_no;


    // get the desired torque
    // float torque_des = atof(argv[2]);
    float torque_des = -3.897048;

    /* open socket to retrieve messages */
    uint16_t mask = (can_id == 0) ? (0x7F0) : (0x7FF);
    s = create_can_socket (can_id + 0x280, 0x000); 
    pthread_t t;
    launch_rt_thread (print_thread_rt, &t, NULL, 80);

    // Construct the caster
    caster = new Caster(caster_no);

    // initialize caster
    if (caster->init())
    {
        printf("Caster %d initialization failed \r \n", caster_no);
    }
    else
    {
        printf("Caster %d initialization success \r \n", caster_no);
    }
    // while(!initialized){}
    
    // enable the caster
    caster->enable();
    enabled = true;
    // while(!enabled){}
    // set the control mode
    caster->setCtrlMode(TORQUE);

    //Launch the control thread
    printf("Launching control thread\n");
    pthread_t control;
    launch_rt_thread (control_thread, &control, NULL, MAX_PRIO);
    running = true;
    while(running){}

    printf("Destroying caster\n");
    caster->~Caster();
    while(!destroyed){} 

    printf("Done\n");
    done = true;
    return 0; 


    /*
    // print usage if error 
    if (argc < 2)
    {
        cout << "usage: ./canDump_motor <motor number> <torque desired>\n";
        return 0;
    }
    //caster number to track passed in command line
    caster_no = atoi(argv[1]);
    uint8_t steer_motor_num = 2*caster_no -1;
    uint8_t roll_motor_num = 2*caster_no;
    torque_des = atof(argv[2]);


    can_id = steer_motor_num;
    // open socket to retrieve messages 
    uint16_t mask = (can_id == 0) ? (0x7F0) : (0x7FF);
    s = create_can_socket (can_id + 0x280, 0x000); 
    pthread_t t;
    launch_rt_thread (print_thread_rt, &t, NULL, 80);

    printf("Calling caster init\n");
    caster = new Caster(caster_no);
    
    if (caster->init())
    {
        printf("Caster %d initialization failed \r \n", caster_no);
    }
    // home_motor(caster->steerMotor); 
    // m->cm = VELOCITY;
    else
    {
        printf("Caster %d initialization success \r \n", caster_no);
    }
    printf("Enabling caster \r\n");
    // enable the caster 
    caster->enable();
    caster->setCtrlMode(TORQUE);
    // Launch control thread 
    pthread_t control;

    launch_rt_thread (control_thread, &control, NULL, MAX_PRIO);
    running = true;
     while(running){}
    printf("Running test\n");


    pthread_join (control, NULL);
    printf("Destroying motor\n");
    ~caster;
    while(!destroyed){}  
    done = true;
    return 0;
    */

}
#endif

// Statusword for switch on disabled
unsigned char INITIALIZED_STATUS[8] = {0x60, 0x46, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0}; 

// Statusword for enabled
unsigned char ENABLED_STATUS[8] = {0x37, 0xC2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

// Secondary statusword for switch on disabled
unsigned char DISABLED_STATUS[8] = {0x60, 0x42, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

/*-----------------------------static functions------------------------------*/
/* 
 * This thread runs constantly reading the raw CAN data on the can bus. It 
 * also takes data in TPDO2 and RPDO2 and converts and prints the value in IU.
 */
static void *
print_thread_rt (void *aux)
{
    FILE *fp;
    fp = fopen("./traces/trace_candump.txt", "w");


    struct can_frame f;
    struct timespec ts;
    float n_sec, now, start_f = 0.0;
    bool first = true;
    while (!done)
    { 
        read(s, &f, sizeof(struct can_frame));
        clock_gettime(CLOCK_MONOTONIC, &ts); 
    
        /* print time stamp */
        n_sec = (float)ts.tv_nsec / (1000*1000*1000);
        now = ts.tv_sec + n_sec - start_f;
        if (first)
        {
            start_f = now;
            first = false;
        }
    
        if((f.can_id & 0x7) == can_id || (f.can_id & 0x7) == 0) {
            printf ("%f\t",now); 
            printf ("0x%X\t", f.can_id);
            fprintf (fp, "0x%X\t", f.can_id);
            printf ("%X\t\t", f.can_dlc);
            fprintf (fp, "%X\t\t", f.can_dlc);
            for (int i = 0; i < 8; i++) {
                printf ("%02X ", f.data[i]);
                fprintf (fp, "%02X ", f.data[i]);
            }

            /* print data in iu for RPDO2, TPDO2, TPDO3, TPDO4*/
            int32_t *p_data = (int32_t *)&f.data;       
            if ((f.can_id & 0xFF0) == 0x300) {
                printf ("%i\t", p_data[0]);
                fprintf (fp, "%i\t", p_data[0]);
            }
            else if ((f.can_id & 0xFF0) == 0x280) {
                printf ("%i\t%i", p_data[0], p_data[1]);
                fprintf (fp, "%i\t%i", p_data[0], p_data[1]);
            }
            else if ((f.can_id & 0xFF0) == 0x380) {
                printf ("%i\t", p_data[0]);
                fprintf (fp, "%i\t", p_data[0]);        
            }
            else if ((f.can_id & 0xFF0) == 0x480) {
                printf ("%i\t", p_data[0]);
                fprintf (fp, "%i\t", p_data[0]);
            }
            printf("\n");
            fprintf(fp, "\n");

            if ((f.can_id & 0xFF0) == 0x180) {
                if (!memcmp(f.data, INITIALIZED_STATUS, sizeof(INITIALIZED_STATUS))) {
                    initialized = true;
                }
                else if (!memcmp(f.data, ENABLED_STATUS, sizeof(ENABLED_STATUS))) {
                    enabled = true;
                    cthread_start = now;
                }
                else if (!memcmp(f.data, DISABLED_STATUS, sizeof(DISABLED_STATUS))) {
                    destroyed = true;
                }
            }

            if (enabled && (now - cthread_start > duration)) {
                running = false;
            }
        }

    }
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
    printf("Control thread launched with success\n");
    int s = create_can_socket (0xF, 0XF);
    struct CO_message msg;
    msg.type = SYNC;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    unsigned long ticks = 1;
    double pos_actual, vel_actual, trq_actual;
    double vel_command = 0, trq_command = 0;
    sleep_until(&next, CONTROL_PERIOD_ns/2);
    
    while(!done)
    {
        CO_send_message (s, 0, &msg);
        usleep (3000);

        
        #ifdef TEST_MOTOR
        motor_get_position(m, &pos_actual);
        motor_get_velocity(m, &vel_actual);
        motor_get_torque(m, &trq_actual);
        //motor_set_velocity(m, vel_command);
        motor_set_torque(m, trq_command);  
        printf("Setting torque: %f", trq_command);      

        sleep_until (&next, CONTROL_PERIOD_ns/2);
        CO_send_message (s, 0, &msg);

        trq_command = 0.974262;//25 * sin(6 * 0.1 * ticks * CONTROL_PERIOD_s);
        
        sleep_until (&next, CONTROL_PERIOD_ns/2);
        ticks++;
        #endif
        
        #ifdef TEST_CASTER
        trq_command = -3.897048;
        caster->setTorques(trq_command, 0);        

        sleep_until (&next, CONTROL_PERIOD_ns/2);
        CO_send_message (s, 0, &msg);

        trq_command = torque_des;//25 * sin(6 * 0.1 * ticks * CONTROL_PERIOD_s);
        
        sleep_until (&next, CONTROL_PERIOD_ns/2);
        ticks++;
        #endif

    }
}