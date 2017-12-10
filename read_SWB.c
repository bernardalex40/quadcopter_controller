//
// jfs9 10/19/17
// v1 Skeleton code to exploit the Preempt-RT patch - used in lab 4
// Edited by: arb392 11/15/2017 - used to read PWM signals

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>
#include <wiringPi.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#define MY_PRIORITY (45)

#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */

#define NSEC_PER_SEC    (1000000000) /* The number of nsecs per sec. */

void stack_prefault(void) {
        unsigned char dummy[MAX_SAFE_STACK];
        memset(dummy, 0, MAX_SAFE_STACK);
        return;
}

int main(int argc, char* argv[])
{
       struct timespec t;
       struct sched_param param;
       int interval = 250000;
       int PinValue = 0;  // used to toggle output pin
       float freq;
       int current_sec, start_sec;
      
      
       // things for recording PWM signals
       float uptime5;
       float downtime5;
       float dc5;
       int current_nsec5 = 0;
       int pin5_then = 0;
       int pin5_now;
       int swb_then = 0;
       int swb_now;
   
       // outut buffering stuff
       setvbuf(stderr, NULL, _IOLBF, 0);

       if ( argc>=2 && atoi(argv[1] ) >0 ) { // if positive argument
          interval = atoi(argv[1]);          // ... use this to change frequency interval
       }
       // Display Interval setting and derived frequency
       freq =  NSEC_PER_SEC * ( (1/(float) (2*interval) ) );
       

       wiringPiSetup();   // initialize WiringPi

       /* Set priority and process scheduler algorithm */
       param.sched_priority = MY_PRIORITY;
       if(sched_setscheduler(0, SCHED_RR, &param) == -1) {
               perror("sched_setscheduler failed");
               exit(-1);
       }
       /* Lock memory */
       if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
               perror("mlockall failed");
               exit(-2);
       }
       /* Pre-fault the stack */
       stack_prefault();

       clock_gettime(CLOCK_MONOTONIC ,&t);
       /* start after one second */
       t.tv_sec++;
       
       
       // more timing stuff

       
       start_sec = t.tv_sec;
       current_sec = 0;
       
       pinMode (21, INPUT) ; // wiringPi pin 21 = GPIO pin 5
       pullUpDnControl(21, PUD_UP);

       while( 1 ) {  
               clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
               
               pin5_now = digitalRead (21) ;
     
               // SWB 
               if ((!pin5_then) && (pin5_now)) {
                    uptime5 = current_nsec5;
               } else if ((pin5_then) && (!pin5_now)) {
		            downtime5 = current_nsec5;
		            dc5 = (downtime5 - uptime5) / (20000000.0);
		            current_nsec5 = 0;
		            swb_now = (dc5 > 0.075);
	           }
	           
	           if ((!swb_then) && (swb_now)) {
					fprintf(stdout,"1\n");
				    fflush(stdout);
					
			   } else if ((swb_then) && (!swb_now)) {
	                fprintf(stdout,"0\n");
				    fflush(stdout);
		       }
		       
		       
		       
	           
               pin5_then = pin5_now;
               
               swb_then = swb_now;

               PinValue = PinValue ^ 1;

               current_nsec5 += interval;
               if (current_nsec5 >= NSEC_PER_SEC) {
				   current_nsec5 = NSEC_PER_SEC;
			   }
               
               t.tv_nsec += interval;

               while (t.tv_nsec >= NSEC_PER_SEC) {   // This accounts for 1 sec rollover
                   t.tv_nsec -= NSEC_PER_SEC;
                   t.tv_sec++;
                   current_sec = t.tv_sec - start_sec;  // how many seconds since we started?
               }
        }
}

