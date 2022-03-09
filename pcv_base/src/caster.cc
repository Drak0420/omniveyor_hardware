#include "../include/caster.h"
#include "../include/motor.h"
#include "../include/CO_message.h"
#include "../include/CAN_utils.h"
#include "../include/RT_utils.h"
#include <unistd.h>
#include <utility>
#include <iostream>
#include <math.h>
#include <omniveyor_common/definitions.h>

using namespace std;

/* Public member functions */

/* Constructor 
 * Caster number starts at 1
 */
Caster::Caster(int casterNum) 
{
  // Set caster number
  number = casterNum;

  // Set default field values
  control = TORQUE; // change back to velocity
  
  initialized = false;
  enabled = false;
  stopped = false;
  lastSteerMotorPos = 0.;
  lastRollMotorPos = 0.;
  steerPos = 0.;
  lastRollPos = 0.;
  steerVel = 0.;
  rollVel = 0.;
  steerTorque = 0.;
  rollTorque = 0.;
}

/* Destructor */
Caster::~Caster() 
{
  motor_destroy(steerMotor);
  motor_destroy(rollMotor);
}

/* Initialize caster motors */
int Caster::init() 
{
  // Determine motor numbers
  int steerMotorNum = 2*number - 1;
  int rollMotorNum = 2*number;
  
  #ifdef TEST_VELOCITY
  control = VELOCITY;
  #endif

  #ifdef TEST_TORQUE
  control = TORQUE;
  #endif

  // Initialize steering motor
  steerMotor = motor_init(steerMotorNum, control, STEERING);
  if(steerMotor == NULL) 
  {
      cout << "failed to initialize steer motor." << endl;
      return -1; // failed to initialize
  }
    
  // Initialize rolling motor
  rollMotor = motor_init(rollMotorNum, control, ROLLING);
  if (rollMotor == NULL)
  {
    cout << "failed to initialize roll motor." << endl;
    // motor_destroy(steerMotor);
    return -1; // failed to initialize
  }

  // success
  initialized = true;
  return 0;
}

/* Enable caster motors */
int Caster::enable()
{
  if(!initialized)
  {
    return -1; // not initialized
  }

  // Enable motors
  if (is_motor_enable_pin_active(steerMotor)){
    motor_enable(steerMotor);
  } else {
    cout << "Enable pin of steer motor of caster " << number << " is DISABLED" << endl; 
  }
  usleep(100000);
  if (is_motor_enable_pin_active(rollMotor)){
    motor_enable(rollMotor); 
  } else {
        cout << "Enable pin of roll motor of caster " << number << " is DISABLED" << endl; 
  }
  usleep(100000);
  enabled = true;
  stopped = false;
  return 0;
}

/* Disable caster motors */
int Caster::disable()
{
  if(!initialized)
  {
    return -1; // not initialized
  }

  // Disable motors
  motor_disable(steerMotor);
  motor_disable(rollMotor); 

  enabled = false;
  return 0;
}

/* Stop caster motors */
int Caster::stop()
{
  stopped = true;
  motor_stop(steerMotor);
  motor_stop(rollMotor);
  return 0;
}

/* Start caster motors */
int Caster::start()
{
  stopped = false;
  motor_start(steerMotor);
  motor_start(rollMotor);
  return 0;
}

/* Set control mode to velocity or torque control */
int Caster::setCtrlMode(enum ctrl_mode cm)
{
  if(enabled && !stopped)
  {
    return -1; // can't change mode while enabled
  }

  // Set control modes
  control = cm;
  motor_set_ctrl_mode(steerMotor, control);
  motor_set_ctrl_mode(rollMotor, control);

  return 0;
}


/* Set caster motor velocities in rad/s */
int Caster::setVelocities(double steerVel, double rollVel) 
{
  if(control != VELOCITY)
  {
    return -1; // must be in velocity mode
  }

  // Calculate motor velocities from joint velocities with kinematic coupling
  double steerMotorVel = -Ns * steerVel;
  double rollMotorVel = (-Nr * steerVel) + (Nr * Nw * rollVel);

  // Set motor velocities
  motor_set_velocity(steerMotor, steerMotorVel);
  motor_set_velocity(rollMotor, rollMotorVel);
  
  return 0;
}


/* Set caster motor torques in Nm*/
int Caster::setTorques(double steerTorque, double rollTorque) 
{
  if (control != TORQUE)
  {
    cout << "not in torque mode " << endl; 
    return -1; // must be in torque mode
  }

  // Calculate motor torques from joint torques with kinematic coupling
  double Ni_00 = -1.0 / Ns;
  double Ni_10 = -1.0 / (Ns * Nw);
  double Ni_11 = 1.0 / (Nr * Nw);

  double steerMotorTorque = Ni_00 * steerTorque + Ni_10 * rollTorque;
  double rollMotorTorque  =                       Ni_11 * rollTorque;

  // Set motor torque
  motor_set_torque(steerMotor, steerMotorTorque);
  motor_set_torque(rollMotor, rollMotorTorque);
  
  return 0;
}


/* Get caster number */
int Caster::getNumber() const
{
  return number;  
}


/* Get control mode (velocity/torque) */
enum ctrl_mode Caster::getCtrlMode() const 
{
  return control;
}


/* Get steering angle of caster wheel in rad */
double Caster::getSteerPosition() 
{
  double steerMotorPos; // variable to fill in motor function

  // Query motor for position. Update if new position data available
  if(!motor_get_position(steerMotor, &steerMotorPos))
  {
    steerPos = -steerMotorPos / Ns;
  }
  else
  {
    // cout << "Got stale caster position" << endl; 
  }

  return steerPos;
}


/* Get steering angle of caster wheel in rad */
double Caster::getSteerDeltaQ() 
{
  double steerMotorPos; // variable to fill in motor function
  double steerDeltaQ;

  // Query motor for position. Update if new position data available
  if(!motor_get_position(steerMotor, &steerMotorPos))
  {
    double steerMotorDeltaQ = steerMotorPos - lastSteerMotorPos;
    steerDeltaQ = -steerMotorDeltaQ / Ns;
    lastSteerMotorPos = steerMotorPos;
  }
  else
  {
    double steerMotorDeltaQ = steerMotorPos - lastSteerMotorPos;
    steerDeltaQ = -steerMotorDeltaQ / Ns;
    lastSteerMotorPos = steerMotorPos;
  }

  return steerDeltaQ;
}

// Check gear box matrix -- is there supposed to be a negative sign in gear ratio? 
/* Get steering angle of caster wheel in rad */
double Caster::getRollDeltaQ() 
{
  double rollMotorPos; // variable to fill in motor function
  double steerMotorPos;
  double rollDeltaQ;

  // Query motor for position. Update if new position data available
  if(!motor_get_position(rollMotor, &rollMotorPos))
  {
    motor_get_position(steerMotor, &steerMotorPos);
    double rollMotorDeltaQ = rollMotorPos - lastRollMotorPos;
    double steerMotorDeltaQ = steerMotorPos - lastSteerMotorPos;
    rollDeltaQ = (steerMotorDeltaQ + (rollMotorDeltaQ / Nr)) / Nw;
    double rollPos = lastRollPos + rollDeltaQ;
    lastRollPos = rollPos;
    lastRollMotorPos = rollMotorPos;
  }
  else
  {
    motor_get_position(steerMotor, &steerMotorPos);
    double rollMotorDeltaQ = rollMotorPos - lastRollMotorPos;
    double steerMotorDeltaQ = steerMotorPos - lastSteerMotorPos;
    rollDeltaQ = (steerMotorDeltaQ + (rollMotorDeltaQ / Nr)) / Nw;
    lastRollMotorPos = rollMotorPos;
  }

  return rollDeltaQ;
}


// Check for error (does rollVel not depend on Ns?)
/* Get steer and roll joint velocities in rad/s 
 * Returns pair with first = steer, second = roll
 */
pair<double,double> Caster::getVelocities() 
{
  double steerMotorVel;
  double rollMotorVel;

  // Calculate motor velocities from joint velocities with kinematic coupling
  int steerStale = motor_get_velocity(steerMotor, &steerMotorVel);
  int rollStale = motor_get_velocity(rollMotor, &rollMotorVel);
  if(!(steerStale || rollStale)) 
  {
    steerVel = -steerMotorVel / Ns;
    // rollVel = (rollMotorVel - steerMotorVel) / (Nr * Nw); // Haley
    rollVel = -steerMotorVel/ (Ns * Nw) + rollMotorVel / (Nr * Nw); // Marion
    // rollVel = (steerVel + (rollMotorVel / Nr)) / Nw;
  }
  else
  {
    //std::cout << "got stale caster vels" << std::endl;
  }
  return make_pair(steerVel,rollVel);
}


// Checked but need to confirm gear matrix 
/* Get steer and roll joint torques in N/m 
 * Returns pair with first = steer, second = roll
 */
pair<double,double> Caster::getTorques() 
{
  double steerMotorTorque;
  double rollMotorTorque;

  // Calculate joint torques from motor torques with kinematic coupling
  int steerStale = motor_get_torque(steerMotor, &steerMotorTorque);
  int rollStale = motor_get_torque(rollMotor, &rollMotorTorque);
  if(!(steerStale || rollStale)) 
  {
    steerTorque = -Ns * steerMotorTorque;
    rollTorque = (-Nr * steerMotorTorque) + (Nr * Nw * rollMotorTorque);
  }

  // return pair of velocities
  return make_pair(steerTorque,rollTorque);
}


void Caster::getAmps(double *steerMotorAmp, double *rollMotorAmp)
{
	motor_get_amp(steerMotor, steerMotorAmp);
  	motor_get_amp(rollMotor, rollMotorAmp);
}

void Caster::getVolts(double *steerMotorVolt, double *rollMotorVolt)
{
	motor_get_volt(steerMotor, steerMotorVolt);
  	motor_get_volt(rollMotor, rollMotorVolt);
}

/* Returns true if bumper is hit */
bool Caster::getBumperState() 
{
  return motor_get_inputs(rollMotor); 
}


/* Get initialization status of caster motors */
bool Caster::isInitialized() const 
{
  return initialized;
}


/* Get enable status of caster motors */
bool Caster::isEnabled() const 
{
  return enabled;
}


/* Get stopped status of caster motors */
bool Caster::isStopped() const 
{
  return stopped;
}

/*********************** TEST HARNESS *****************************************/

// Test caster code
#ifdef TEST_CASTER

#define PI                  (3.14159)
#define CONTROL_PERIOD_ns   (7000000)
#define CONTROL_PERIOD_s    (0.007)
#define MAX_VEL             (5*PI)    /* [rad/s] */
#define MAX_TOR             (10.0)     // N*m
#define X_freq              (0.1)         /* [hz] */
#define MAX_VEL_IU          (0x100000)

// Different types of tests to run
enum TEST_TYPE
{
  TEST_INIT,       // 0
  TEST_STREAM_POS, // 1
  TEST_VEL_RAMP,   // 2
  TEST_VEL_SINE,   // 3
  TEST_TOR_STEER   // 4
};

static void * control_thread (void *aux);
static enum TEST_TYPE test_mode = TEST_INIT;

/* test static variables */
static bool done = false;
static Caster *caster;

int main (int argc, char *argv[])
{
  cout << "Begin caster test harness" << endl;

    if(argc < 4) {
        puts("Usage: sudo ./test <caster_no> <test_mode_int> <duration>");
        exit(0);    
    }

    int caster_no = atoi (argv[1]);
    caster = new Caster(caster_no);

  int test_mode_int = atoi (argv[2]);
  test_mode = (enum TEST_TYPE) test_mode_int;


  printf("TEST MODE %d",test_mode);

    int duration = atoi (argv[3]);


  /* initialize */
  if (caster->init())
    cout << "Caster " << caster_no << " initialization failed" << endl;
    // home_motor(caster->steerMotor); 
    // m->cm = VELOCITY;
  else
    cout << "Caster " << caster_no << " initialization success " << endl;

  // Switch on tes type
  switch(test_mode){

    case TEST_STREAM_POS:
    {
      int s = create_can_socket (0xF, 0xF);
      cout << "Testing Steer Position Streaming" << endl;
      cout << "streaming position data, press any key to start, then press any key to stop " << endl;
      getchar ();

      while (1)
      {
        struct CO_message msg;
        msg.type = SYNC;

        /* send sync */
        CO_send_message (s, 0, &msg);
        usleep (1000);

        cout << "Steering Position (rad): " << caster->getSteerPosition() << endl;
      }
    }
      break;

    default:
    {
      /* enable the caster */
      caster->enable();
      
      printf("Launch control thread for testing");

      /* Launch control thread */
      pthread_t control;
      launch_rt_thread (control_thread, &control, NULL, MAX_PRIO);

      /* sleep for some time and then stop control thread */;
      sleep (duration);
      done = true;
      pthread_join (control, NULL);
    }
      break;

    //default:
    //        puts("Unspecified test mode");
    //  break;
  }

  // Delete caster
  delete caster;

  cout << "End caster test harness" << endl;
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
  int s = create_can_socket (0xF, 0xF);
  struct CO_message msg;
  msg.type = SYNC;

  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  
  /* open dump file */
  FILE *fp;
  fp = fopen("traces/trace_caster.csv", "w");
  fprintf(fp, "t,steer_vel_command,roll_vel_command,steer_vel_actual,roll_vel_actual,steer_pos_actual,steer_tor_command,steer_tor_actual\n");

  /* variables for sin, cos waves */
  unsigned long ticks = 0;
  double steer_pos, steer_vel_command = 0, roll_vel_command = 0;
  double steer_tor_command = 0, roll_tor_command = 0;
  std::pair<double,double> vels;
  std::pair<double,double> torques;

  /* send initial synch message */
  CO_send_message (s, 0, &msg);
  sleep_until (&next, CONTROL_PERIOD_ns/2);

  while (!done) 
  {
/* ---------- first half of control loop ---------- */
    /* send sync message */
    CO_send_message (s, 0, &msg);
    usleep (3000);

    vels = caster->getVelocities();
    torques = caster->getTorques();
    steer_pos = caster->getSteerPosition();

    #ifdef TEST_VELOCITY 
    caster->setVelocities(steer_vel_command, roll_vel_command);
    #endif
    
    #ifdef TEST_TORQUE
    caster->setTorques(steer_tor_command,roll_tor_command);
    #endif

    sleep_until (&next, CONTROL_PERIOD_ns/2);
/* --------- second half of control loop --------- */
    /* send sync message */
    CO_send_message (s, 0, &msg);

    /* write to file */
    fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f\n", (ticks*CONTROL_PERIOD_s), steer_vel_command, 
      roll_vel_command, vels.first, vels.second, steer_pos,steer_tor_command,torques.first);

    roll_vel_command = 0;
    steer_vel_command = 0; 
    steer_tor_command = 0;
    roll_tor_command = 0;

    /* calculate next commands based on test mode */
    switch(test_mode){
      case TEST_VEL_RAMP:
        roll_vel_command = MAX_VEL*sin(2*PI*X_freq*(ticks*CONTROL_PERIOD_s));
        break;
      case TEST_VEL_SINE: 
        steer_vel_command = MAX_VEL*sin(6*PI*X_freq*(ticks*CONTROL_PERIOD_s));
        break;
      case TEST_TOR_STEER:
      {
        double m_bar = 0.27; //kg
        double m_mass = 1.; //1.; //kg
        double m_caster = 0.; //2.72; //kg
        double l = .3708; //m
        double d_hole = .3476;
        double d = 0.01; //.0508 + .003;// m -- TODO: this is a guess!! distance from motor axis to com of caster
        double g = 9.8; // m/s^2
        double CCW = -1.;
        double CW = 1.;

        steer_tor_command = CCW*(m_caster*g*d + m_bar*g*(l/2) + m_mass*g*d_hole); //Ns;
        printf("Caster torque: %f, Bar torque: %f, Mass torque: %f -> Commanded torque: %f \r\n", m_caster*g*d, m_bar*g*(l/2), m_mass*g*l, steer_tor_command);
      }
        break;
      default:
        puts("Illegal test mode");
        roll_vel_command = 0;
        steer_vel_command = 0; 
        roll_tor_command = 0;
        steer_tor_command = 0; 
    }

    // Cap velocities
    if (roll_vel_command > MAX_VEL){
      roll_vel_command = MAX_VEL;
    }
    else if (roll_vel_command < -MAX_VEL){
      roll_vel_command = -MAX_VEL;
    }

      // Cap torques
    if (steer_tor_command > MAX_TOR){
      steer_tor_command = MAX_TOR;
    }
    else if (steer_tor_command < -MAX_TOR){
      steer_tor_command = -MAX_TOR;
    }

    ticks++;

    /* do nothing in this half */
    sleep_until (&next, CONTROL_PERIOD_ns/2);
  }
}

#endif
