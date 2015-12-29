/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
 
/**
 * @file pump_filter.c
 * Minimal application example for PX4 autopilot.
 */
 

#include <px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/rbp_pump_state.h>   // Getting the signal from the Rasberry pie
#include <uORB/topics/rbp_pump_display.h> // To publish actual speed of the pump
#include <uORB/topics/vehicle_status.h>   // Offboard conrol on?

#include <uORB/topics/laser_distance.h> //Testing

 
extern "C" __EXPORT int pump_filter_main(int argc, char *argv[]);

class PumpFilter{
public:
	/**
	 * Constructor
	 */
	PumpFilter();

	/**
	 * Destructor, also kills the main task
	 */
	~PumpFilter();

	float 		manual_mode_filter(float rc_aux_input);

	float		automatic_mode_filter(float rbp_aux_input);

	float 		pump_percentage(float Input);

	bool 		pump_auto_is_true(int Nav_Status, int Act_Status);

	bool		go_on_check();

	void		setDef();

	void		setGoTrue(int which_go_on);



private:
	
	bool go_on1 = false;
	bool go_on2 = false;
	bool go_on3 = false;


};




//Constructor
PumpFilter::PumpFilter(){
}

//Destructor
PumpFilter::~PumpFilter(){
}

float PumpFilter::manual_mode_filter(float rc_aux_input){

	return rc_aux_input;
}

float PumpFilter::automatic_mode_filter(float rbp_aux_input){

	return rbp_aux_input;
}

float PumpFilter::pump_percentage(float Input){

	return ( (Input+1)/2 ) * 100;
}

bool PumpFilter::pump_auto_is_true(int Nav_Status, int Act_Status){

	bool result = false;

	if (Nav_Status == Act_Status){
		
		result = true;
	}

	return result;
}

bool PumpFilter::go_on_check(){

	bool result = false;

	if (go_on1 || go_on2 || go_on3){
		
		result = true;
	}

	return result;
}



void PumpFilter::setDef(){

	go_on1=false;
	go_on2=false;
	go_on3=false;
}


void PumpFilter::setGoTrue(int which_go_on){

	if (which_go_on==1){
		go_on1= true;
	}
	if (which_go_on==2){
		go_on2= true;
	}
	if (which_go_on==3){
		go_on3= true;
	}


}

int pump_filter_main(int argc, char *argv[])
{
	PumpFilter PumpReg;

	
	

	/* subscribe to manual_control_setpoint topic */
	int manual_sub_fd = orb_subscribe(ORB_ID(manual_control_setpoint));
	orb_set_interval(manual_sub_fd, 1000);

	int rbp_pump_sub_fd = orb_subscribe(ORB_ID(rbp_pump_state));
	orb_set_interval(rbp_pump_sub_fd, 1000);
	
	int vehicle_status_sub_fd = orb_subscribe(ORB_ID(vehicle_status));
	orb_set_interval(vehicle_status_sub_fd, 1000);

	// Limited to 1 s (1000 ms)PumpReg::pump_subscriptions();



	/* advertise manual_control_setpoint topic */
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	orb_advert_t manual_pub_fd = orb_advertise(ORB_ID(manual_control_setpoint), &manual);

	struct rbp_pump_display_s rbp_pump;
	memset(&rbp_pump, 0, sizeof(rbp_pump));
	orb_advert_t rbp_pump_pub_fd = orb_advertise(ORB_ID(rbp_pump_display), &rbp_pump);


		//only for testing
		struct laser_distance_s distance;
		memset(&distance, 0, sizeof(distance));
		orb_advert_t laser_dis_pub_fd = orb_advertise(ORB_ID(laser_distance), &distance);
		//only for testing

 
 	

	pollfd fds[3];

	fds[0].fd = manual_sub_fd;
	fds[0].events = POLLIN;

	fds[1].fd = rbp_pump_sub_fd;
	fds[1].events = POLLIN;

	fds[2].fd = vehicle_status_sub_fd;
	fds[2].events = POLLIN;
	
	struct rbp_pump_state_s pum;
	struct manual_control_setpoint_s man;
	struct vehicle_status_s vehs;

 	int error_counter = 0;

	while (true) {
		/* wait for sensor update of 3 file descriptor for 1000 ms (1 second) */
		int poll_ret = poll(fds, 3, 1000);
 
		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			printf("[pump_filter] Got no data within a second\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[pump_filter] ERROR return value from poll(): %d\n"
					, poll_ret);
			}
			error_counter++;
		} else {
 
			if (fds[0].revents & POLLIN) {

				/* copy sensors raw1 data into local buffer */
				orb_copy(ORB_ID(manual_control_setpoint), manual_sub_fd, &man);

				PumpReg.setGoTrue(1);
			}

			if (fds[1].revents & POLLIN) {

				orb_copy(ORB_ID(rbp_pump_state), rbp_pump_sub_fd, &pum);

				PumpReg.setGoTrue(2);
			}

			if (fds[2].revents & POLLIN) {

				orb_copy(ORB_ID(vehicle_status), vehicle_status_sub_fd, &vehs);

				PumpReg.setGoTrue(3);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
			 
			 
			 if (PumpReg.go_on_check()){

			 	if (!PumpReg.pump_auto_is_true(vehs.nav_state,vehs.NAVIGATION_STATE_POSCTL 	)){


			 		manual.aux1 = PumpReg.manual_mode_filter(man.aux1raw);
					orb_publish(ORB_ID(manual_control_setpoint), manual_pub_fd, &manual);

			 	}else{

			 		manual.aux1 = PumpReg.automatic_mode_filter(pum.ps_input);			 
					orb_publish(ORB_ID(manual_control_setpoint), manual_pub_fd, &manual);
				}

				rbp_pump.ps_observe = PumpReg.pump_percentage(manual.aux1);
				orb_publish(ORB_ID(rbp_pump_display),rbp_pump_pub_fd, &rbp_pump);


					distance.min_distance=PumpReg.pump_percentage(manua.aux1);
					orb_publish(ORB_ID(laser_distance),laser_dis_pub_fd, &distance);


			 

			 }
			 
			 PumpReg.setDef();
			 


		}
	}
 
	return 0;
}