/*------------------------------includes--------------------------------------*/
#include <fcntl.h>
#include <stdio.h>
#include <linux/joystick.h>
#include "../include/event.h"
#include "../include/buttons.h"
#include "../include/RT_utils.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <mqueue.h>
#include <pthread.h>
#include <signal.h>

/*------------------------------defines---------------------------------------*/
#define QUEUE_SIZE    (32)

/*------------------------------structs---------------------------------------*/
enum state
{
	UNPAIRED,
	PAIRED_INACTIVE,
	PAIRED_ACTIVE
};

/*------------------------static function declarations------------------------*/
static void init_mQueue (mqd_t *m);
static int read_event (int fd, struct js_event *e);
static struct event translate_js_event (struct js_event *js_e);
static void handler (int sig);
static void rumble (void);

/*------------------------static variable declarations------------------------*/
static pid_t child, rumble_child;;
static enum state state = UNPAIRED;
static const char *mq_name = "/joystick message queue"; 

/*---------------------------public functions---------------------------------*/
void 
main (void)
{
  int fd;
  const char *device = "/dev/input/js0";
  // const char *cmd = "vehicle";
  const char *cmd = "/home/iprl/Mobile_Manipulation/bin/vehicle";

  char *argv[] = {NULL};
  struct js_event js_e;
  struct event e;
  mqd_t m;

  /* create message queue to talk with vehicle program*/
  init_mQueue (&m);

  /* install signal handler for SIGCHLD and SIGINT signals */
  signal (SIGCHLD, handler);
  signal (SIGINT, handler);

  while (1)
  {
    /* if we are paired, then read from the descriptor  */
    if (state != UNPAIRED)
    {
      if (read_event (fd, &js_e))
      {
        /* error reading from js0, kill child process if currently paired */
        puts ("error reading from js0, unpairing");
        if (state == PAIRED_ACTIVE)
          kill (child, SIGINT);

        state = UNPAIRED;
      }
      else
      {
        e = translate_js_event (&js_e);
      }
    }

  	switch (state)
    { 
      case UNPAIRED:
        /* try to open the js0 descriptor*/
        close (fd);
        fd = open (device, O_RDONLY);
          
        if (fd == -1)
        {
          /* if opening the js0 descriptor failed, then sleep for 1 s
             before trying again to open it */
          sleep (1);
          puts ("not paired");
        }
        else
        {
          state = PAIRED_INACTIVE;
          puts ("paired inactive");
          rumble ();
        }
        break;
      
      case PAIRED_INACTIVE:
        if (e.type == BUTTON_PRESSED && e.param == START_BUTTON)
        {
          //launch the vehicle program and assume success
          //let signal handler account for errors 
          state = PAIRED_ACTIVE;
          puts ("paired active");
          
          /* Spawn child. */
          puts ("launching vehicle program");
          child = fork();
          if (child == 0) 
          {
            execv (cmd, argv); 
            // execvp (cmd, argv); // execute "vehicle" program is currently in /usr/bin

            /* if exec fails, then just exit */
            exit(0);
          }
        }
        break;

      case PAIRED_ACTIVE:
        if (e.type == BUTTON_PRESSED && e.param == START_BUTTON)
          kill (child, SIGINT);         
        else if (e.type != NO_EVENT)
          mq_send (m, (char *)&e, sizeof (e), 0);
        break;
    }
  }
}

/*-----------------------------static functions-------------------------------*/
/*
 * This function initializes the message queue used to pass events to the 
 * vehicle program when in the PAIRED_ACTIVE state. To understand the message
 * queue interface visit http://man7.org/linux/man-pages/man7/mq_overview.7.html
 */
static void
init_mQueue (mqd_t *m)
{	
	struct mq_attr attr = {0};
	attr.mq_maxmsg = QUEUE_SIZE;
	attr.mq_msgsize = sizeof (struct event);

	*m = mq_open (mq_name, O_RDWR | O_CREAT, 0664, &attr);

	if (*m == -1)
	{
		perror ("failed to open message queue in init_mQueue\n");
		exit (-1);
	}
}

/*
 * Reads from the joystick file descriptor (fd) and populates the js_event 
 * struct. If there is an error in reading from the descriptor, this function
 * returns -1. It returns 0 on success.
 */
static int
read_event (int fd, struct js_event *e)
{
	ssize_t bytes;

	bytes = read (fd, e, sizeof (*e));

	if (bytes == sizeof (*e))
  {
		return 0;
  }

	/* error in trying to read from fd */
	return -1;
}

/*
 * The function takes the js_event struct that is the linux interface to the 
 * js0 input devices and translates it to a struct event that we use to pass to 
 * the vehicle. 
 * See: https://www.kernel.org/doc/Documentation/input/joystick-api.txt
 */
static struct event
translate_js_event (struct js_event *js_e)
{
  struct event e;
  int32_t js_value;

  /* respond to button events and joystick events */
  if (js_e->type == JS_EVENT_BUTTON)
  {
    /* event only for button pressed, store button in event param */
    if (js_e->value)
    {
      e.type = BUTTON_PRESSED;
      e.param = js_e->number;
    }
    else
      e.type = NO_EVENT;
  }
  else if (js_e->type == JS_EVENT_AXIS)
  {
    js_value = js_e->value;
    switch (js_e->number)
    {
      case LJS_X:
        // e = (struct event){NEW_Xd_COMMAND, js_value};
        e = (struct event){NEW_Yd_COMMAND, -js_value}; // Right arrow on joystick is negative y direction on platform
        break;

      case LJS_Y:
        // e = (struct event){NEW_Yd_COMMAND, -js_value};
        e = (struct event){NEW_Xd_COMMAND, -js_value}; // Up arrow on joystick is positive x direction on platform
        break;

      case RJS_X:
        e = (struct event){NEW_THETAd_COMMAND, -js_value};
        break;

      default:
        e.type = NO_EVENT;
        break;

    }
  }

  return e;
}

/*
 * The signal handler for the joystick program. SIGCHILD signals are used for
 * when the vehicle child process exits unexpectedly (we need to move back to
 * UNPAIRED). SIGINT signal triggers a shutdown of this program.
 */
static void 
handler (int sig)
{
  if (sig == SIGCHLD)
  {
  	pid_t c = wait (NULL);
    if (c == child)
    {
      /* reap child and set state to unpaired */
      puts ("vehicle program shutdown");
      state = UNPAIRED;
    }
    else if (c == rumble_child)
      puts ("rumble shutdown");
  }
  else if (sig == SIGINT)
  { 
    /* clean up the message queue before exiting */
    printf ("\n");
    printf ("SIGINT received in joystick\n"); 
    printf ("joystick exiting\n");
    mq_unlink (mq_name);
    exit (0);
  }
}


/*
 * TODO: implement a better rumble function, right now this just launches a 
 * test program for the rumble pack and lets it run for 2 seconds before
 * killing the program.
 */
static void 
rumble (void)
{
  char *argv[] = {"fftest", 
  "/dev/input/by-id/usb-Logitech_Logitech_Cordless_RumblePad_2-event-joystick",
  NULL};

  int fds[2];
  pipe (fds);

  rumble_child = fork ();
  if (rumble_child == 0)
  {
    /* redirect stdin and stdout */
    dup2 (fds[0], STDIN_FILENO);
    dup2 (fds[1], STDOUT_FILENO);
    execvp (argv[0], argv);

    // Reached only on failure to execute 
    puts ("failed to execute rumble child");
    exit (0);
  }

  dprintf (fds[1], "0\n");
  sleep (2);
  kill (rumble_child, SIGINT);
  close (fds[0]);
  close (fds[1]);
}
