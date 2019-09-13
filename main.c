#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>

#include "camera/camera.h"
#include "camera/dcmi_camera.h"
#include "sensors/ground.h"
#include "sensors/proximity.h"
#include "motors.h"
#include "behaviors.h"
#include "leds.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

uint16_t timer_sec;

// Autocharge demo variables
#define GROUND_THR 550
uint8_t autocharge_state = 0;
uint16_t last_timer_count = 0;
unsigned char lineFound = 0;
unsigned int outOfLine = 0;
unsigned char chargeContact = 0;
unsigned char directionChanged=0;
signed int lineFollowSpeed=0;
unsigned char escapeDir = 0;
unsigned char timerTen = 0;
unsigned char randomDirection = 0;
uint8_t *img_buff_ptr;

void cliff_autocharge_reset_vars(void) {
	if(autocharge_state != 3) {	// keep the value of the timer when in state3 (charge) in order to avoid to pass from state2 (follow line)
							// to state3 (charge) and back to state2 without any timeout
		timer_sec = 0;
	}
	last_timer_count = 0;
	lineFound = 0;
	outOfLine = 0;
	chargeContact = 0;
	directionChanged = 0;
	lineFollowSpeed = 0;
	timerTen = 0;
}

static THD_WORKING_AREA(autocharge_thd_wa, 2048);

static THD_FUNCTION(autocharge_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    cam_advanced_config(FORMAT_GREYSCALE, 315, 235, 10, 10, SUBSAMPLING_X1, SUBSAMPLING_X1);
    cam_set_exposure(512, 0);
    dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
    dcmi_prepare();

    messagebus_topic_t *ground_topic = messagebus_find_topic_blocking(&bus, "/ground");
    ground_msg_t ground_values;

    calibrate_ir();

	while(1) {

		messagebus_topic_wait(ground_topic, &ground_values, sizeof(ground_values));

		switch(autocharge_state) {

			case 0:	// Move around.
				enable_obstacle_avoidance(700);

				set_body_led(0);
				set_front_led(0);

				if(timer_sec >= 15) {	// about 15 seconds
					cliff_autocharge_reset_vars();
					autocharge_state = 1;
					break;
				}

				if((timer_sec%5==0) && (last_timer_count != timer_sec)) {	// change state every 5 seconds
					last_timer_count = timer_sec;
				}

				break;

			case 1:	// Search for a line
				set_body_led(0);
				if(timer_sec&0x01) {
					set_front_led(1);
				} else {
					set_front_led(0);
				}

				if(timer_sec > 20) {	// the robot seems to be blocked somehow, so after about 20 seconds (with camera activated) restart from state=0
					cliff_autocharge_reset_vars();
					autocharge_state = 0;
					break;
				}

				if(outOfLine > 0) {		// go out of the line
					outOfLine--;
				} else {
					if(ground_values.delta[0]<GROUND_THR || ground_values.delta[1]<GROUND_THR || ground_values.delta[2]<GROUND_THR || ground_values.delta[3]<GROUND_THR || ground_values.delta[4]<GROUND_THR) {
						lineFound++;
						if(lineFound > 10) {
							dcmi_capture_start();
							cliff_autocharge_reset_vars();
							autocharge_state = 2;
							break;
						}
					} else {
						lineFound = 0;
					}
				}
				break;

			case 2:	// line found, follow it
				disable_obstacle_avoidance();
				set_front_led(0);
				if(timer_sec&0x01) {
					set_body_led(1);
				} else {
					set_body_led(0);
				}

				if(timer_sec > 10) {	// the robot seems to be blocked somehow, try to escape...
					if(escapeDir == 0) {
						left_motor_set_pos(0);
						left_motor_set_speed(-150);
						right_motor_set_speed(0);
						while(left_motor_get_pos() > -75);
						right_motor_set_pos(0);
						left_motor_set_speed(0);
						right_motor_set_speed(-150);
						while(right_motor_get_pos() > -75);
					} else {
						right_motor_set_pos(0);
						left_motor_set_speed(0);
						right_motor_set_speed(-150);
						while(right_motor_get_pos() > -75);
						left_motor_set_pos(0);
						left_motor_set_speed(-150);
						right_motor_set_speed(0);
						while(left_motor_get_pos() > -75);
					}
					timer_sec = 0;
					escapeDir = 1 - escapeDir;
					timerTen++;
				}

				if(timerTen >= 3) {	// the robot seems to be blocked somehow and we cannot escape, so after about 30 seconds (with camera activated) restart from state=0
					left_motor_set_pos(0);
					left_motor_set_speed(-300);
					right_motor_set_speed(-300);
					while(left_motor_get_pos() > -250);
					cliff_autocharge_reset_vars();
					autocharge_state = 0;
					break;
				}

				if(!image_is_ready()) {
					wait_image_ready();
				}
				img_buff_ptr = cam_get_last_image_ptr();

				if(img_buff_ptr[0]>180) {
					chargeContact++;
					if(chargeContact > 10) {
						left_motor_set_speed(0);
						right_motor_set_speed(0);
						dcmi_capture_start();
						cliff_autocharge_reset_vars();
						autocharge_state = 3;
						break;
					}
				} else {	// the is turned off so the contact isn't good yet
					chargeContact = 0;
				}

				if(ground_values.delta[0]>900 && ground_values.delta[1]>900 && ground_values.delta[2]>900) {	// i'm going to go out of the line
					outOfLine++;
					if(outOfLine > 10) {
						cliff_autocharge_reset_vars();
						outOfLine = 10000;
						autocharge_state = 1;
						break;
					}
				} else {
					outOfLine = 0;
				}

				dcmi_capture_start();

				if(ground_values.delta[4] < GROUND_THR && ground_values.delta[0]>GROUND_THR && ground_values.delta[1]>GROUND_THR && ground_values.delta[2]>GROUND_THR && ground_values.delta[3]>GROUND_THR) { // left cliff inside line, turn left
					left_motor_set_speed(-300);
					right_motor_set_speed(300);
					directionChanged=1;
				} else if(ground_values.delta[3] < GROUND_THR && ground_values.delta[0]>GROUND_THR && ground_values.delta[1]>GROUND_THR && ground_values.delta[2]>GROUND_THR && ground_values.delta[4]>GROUND_THR) {
					left_motor_set_speed(300);
					right_motor_set_speed(-300);
					directionChanged=1;
				} else if(ground_values.delta[0] > GROUND_THR && ground_values.delta[2] < GROUND_THR && directionChanged==0) {	// left leaving the line => turn right
					left_motor_set_speed((ground_values.delta[0]-GROUND_THR)/2);
					right_motor_set_speed(-((ground_values.delta[0]-GROUND_THR)/2));
					directionChanged=1;
					lineFollowSpeed = 1024-ground_values.delta[0];
				} else if(ground_values.delta[2] > GROUND_THR && ground_values.delta[0] < GROUND_THR && directionChanged==0) {	// right leaving the line => turn left
					left_motor_set_speed(-((ground_values.delta[2]-GROUND_THR)/2));
					right_motor_set_speed((ground_values.delta[2]-GROUND_THR)/2);
					directionChanged=1;
					lineFollowSpeed = 1024-ground_values.delta[2];
				} else {	// within the line
					if(directionChanged==0) {
						lineFollowSpeed = 300;
						lineFollowSpeed = 300;
					} else {
						if(lineFollowSpeed > 300) {
							lineFollowSpeed=300;
						}
						if(lineFollowSpeed<100) {
							lineFollowSpeed=100;
						}
					}
					left_motor_set_speed(lineFollowSpeed);
					right_motor_set_speed(lineFollowSpeed);
					directionChanged=0;
				}
				break;

			case 3:	// charge for some time
				set_body_led(0);
				set_front_led(0);
				clear_leds();

				if(!image_is_ready()) {
					wait_image_ready();
				}
				img_buff_ptr = cam_get_last_image_ptr();

				if(img_buff_ptr[0]<170) {	// the contact is lost
					dcmi_capture_start();
					cliff_autocharge_reset_vars();
					autocharge_state = 2;
					break;
				}

				dcmi_capture_start();

				if(timer_sec > 30) {		// about 30 seconds of charge with camera activated
					left_motor_set_pos(0);
					left_motor_set_speed(-300);
					right_motor_set_speed(-300);
					while(left_motor_get_pos() > -250);
					//randomDirection = rand()%2;
					randomDirection = 1 - randomDirection;
					if(randomDirection) {
						left_motor_set_pos(0);
						left_motor_set_speed(300);
						while(left_motor_get_pos() < 250);
					} else {
						right_motor_set_pos(0);
						right_motor_set_speed(300);
						while(right_motor_get_pos() < 250);
					}
					cliff_autocharge_reset_vars();
					timer_sec=0;
					autocharge_state = 0;
					break;
				}
				break;
		}
	} // while

}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Init the peripherals.
	clear_leds();
	set_body_led(0);
	set_front_led(0);
	dcmi_start();
	cam_start();
	motors_init();
	proximity_start();
	ground_start();
	behaviors_start();

    chThdCreateStatic(autocharge_thd_wa, sizeof(autocharge_thd_wa), NORMALPRIO, autocharge_thd, NULL);

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
        timer_sec++;
        if(timer_sec == 0xFFFF) {
        	timer_sec = 0;
        }
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
