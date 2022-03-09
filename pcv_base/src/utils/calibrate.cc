/*-----------------------------includes--------------------------------------*/
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include "../include/motor.h"
#include "../include/CO_message.h"
#include "../include/CAN_utils.h"
#include "../include/RT_utils.h"
#include "../include/vehicle.h"
#include "../include/caster.h"

using namespace std;

/*-----------------------------defines---------------------------------------*/

/*-----------------------------structs---------------------------------------*/

/*-----------------------static function declarations------------------------*/
static int read_and_store_calibration (Caster *casters[]);
static int s;

/*-----------------------static variable declarations------------------------*/

/*--------------------------public functions---------------------------------*/
int 
main (void)
{
	int success = true; 

	/* construct the caster objects*/
	Caster *casters[NUM_CASTERS];

	for (int i = 0; i < NUM_CASTERS; i++)
		casters[i] = new Caster(i+1);

	/* remove calFile so that we start off clean */
	if (remove ("./.motor_cal.txt") != 0)
		cout << "error removing file" << endl;

	/* prompting user to elevate robot  */
	cout << "Elevate the robot so that all casters can spin freely" << endl;
	cout << "Press any key to continue once done" << endl;
	// getchar ();
	// while( getchar() != '\n');
	cout << "Homing casters...\t" << flush;

	/* perform homing */
	for (int i = 0; i < NUM_CASTERS; i++)
	{
		if (casters[i]->init())
			success = false;
	}

	/* move on to reading offsets if homing succeeded */
	if (success)
	{
		cout << "Homing complete" << endl;
		cout << "Use a straight edge to align diagonal casters" << endl;
		cout << "Press any key to continue once done" << endl;
		getchar ();
		if (read_and_store_calibration (casters))
			cout << "Calibration failed" << endl;
		else
			cout << "Calibration complete" << endl;
	}
	else 
	{
		cout << "Homing failed" << endl;
		cout << "Calibration failed" << endl;
	}

	/* cleanup */
	for (int i = 0; i < NUM_CASTERS; i++)
		delete casters[i];

	/* reset comm on exit */
	struct CO_message msg;
	msg.type = NMT;
	msg.m.NMT.data = 0x81;
	CO_send_message (s, 0, &msg);

	return 0;
}

/*-----------------------------static functions------------------------------*/

/* reads and stores calibration values after homing */
static int 
read_and_store_calibration (Caster *casters[])
{
	/* open file to write cal values to (hidden file) */
	ofstream calFile("./.motor_cal.txt");
  if (!calFile) {
   	cout << "Unable to open calibration file";
    return -1;
   }

  /* send sync so motor position is sent */
  s = create_can_socket (0xF, 0xF);
  struct CO_message msg;
	msg.type = SYNC;
	CO_send_message (s, 0, &msg);
	usleep (1000);
	CO_send_message (s, 0, &msg);
	usleep (1000);
	CO_send_message (s, 0, &msg);
	usleep (1000);

	/* now read the positions and store in file */
	for (int i = 0; i < NUM_CASTERS; i++)
	{
		double offset = casters[i]->getSteerPosition();
		calFile << offset << endl;
	}

	/* close the file and return success */
	calFile.close ();

	return 0;
}
