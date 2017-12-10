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

#define MY_PRIORITY (44) 

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
       float uptime6;
       float downtime6;
       float dc6;
       int current_nsec6 = 0;
       int pin6_then = 0;
       int pin6_now;
       int swa_then = 0;
       int swa_now;
   
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
       
       pinMode (22, INPUT) ; // wiringPi pin 22 = GPIO pin 6
       pullUpDnControl(22, PUD_UP);

       while( 1 ) {  
               clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
               
               pin6_now = digitalRead (22) ;
     
              
	           // SWA 
               if ((!pin6_then) && (pin6_now)) {
                    uptime6 = current_nsec6;                    
               } else if ((pin6_then) && (!pin6_now)) {
		            downtime6 = current_nsec6;
		            dc6 = (downtime6 - uptime6) / (20000000.0);
		            current_nsec6 = 0;
		            swa_now = (dc6 > 0.075);
					fflush(stdout);
	           }
	           
	           
	           
	           if ((!swa_then) && (swa_now)) {
				    fprintf(stdout,"1\n");
				    fflush(stdout);
			   } else if ((swa_then) && (!swa_now)) {
	                fprintf(stdout, "0\n");
	                fflush(stdout);
		       }
     
     
               pin6_then = pin6_now;
               
               swa_then = swa_now;

               PinValue = PinValue ^ 1;
               
               current_nsec6 += interval;
               
               if (current_nsec6 >= NSEC_PER_SEC) {
				   current_nsec6 = NSEC_PER_SEC;
			   }

               t.tv_nsec += interval;

               while (t.tv_nsec >= NSEC_PER_SEC) {   // This accounts for 1 sec rollover
                   t.tv_nsec -= NSEC_PER_SEC;
                   t.tv_sec++;
                   current_sec = t.tv_sec - start_sec;  // how many seconds since we started?
               }
        }
}

