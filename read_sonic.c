//
// jfs9 10/19/17
// v1 Skeleton code to exploit the Preempt-RT patch - file from lab 4
// Edited by: arb392 11/15/2017 - used to interface with sonic sensor

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

#define MY_PRIORITY (43) 

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
       int interval = 75000; // run dis loop at around 33 khz
       float freq;
       int current_sec, start_sec;
      
      
       // things for recording PWM signals
       float uptime19;
       float downtime19;
       float dc19;
       float current_nsec_sqr = 0;
       int current_nsec19 = 0;
       int pin13 = 0;
       int pin19_then = 0;
       int pin19_now;
       float wide = 0.3*NSEC_PER_SEC;
   
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
       
       pinMode (23, OUTPUT) ; // wiringPi pin 23 = GPIO pin 13
       pinMode (24, INPUT)  ; // wiringPi pin 24 = GPIO pin 19
       pullUpDnControl(24, PUD_UP);

       while( 1 ) {  // this one runs at 33 khz
               clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
               
               
               // first generate the ol' square wave at 30 Hz
               // 1Hz is 1E9 nsec wide
               // doing this on pin 13
               if (current_nsec_sqr > wide){
				   current_nsec_sqr = 0;
				   pin13 = pin13^1;
				   if (pin13){
					   digitalWrite (23, HIGH);
				   }else {
					   digitalWrite (23, LOW);
				   }
			   }
               
               
               
               pin19_now = digitalRead (24) ;
     
               // echo
               if ((!pin19_then) && (pin19_now)) {
                    uptime19 = current_nsec19;
               } else if ((pin19_then) && (!pin19_now)) {
		            downtime19 = current_nsec19;
		            dc19 = (downtime19 - uptime19) / 1000000000.0 / 2 * 344.02 * 100;
		            current_nsec19 = 0;
		            fprintf(stdout,"%f\n", dc19);
		            fflush(stdout);
	           }


               pin19_then = pin19_now;

               current_nsec_sqr += interval;
               if (current_nsec_sqr >= NSEC_PER_SEC) {
				   current_nsec_sqr = NSEC_PER_SEC;
			   }
               current_nsec19 += interval;
               if (current_nsec19 >= NSEC_PER_SEC) {
				   current_nsec19 = NSEC_PER_SEC;
			   }
               
               t.tv_nsec += interval;

               while (t.tv_nsec >= NSEC_PER_SEC) {   // This accounts for 1 sec rollover
                   t.tv_nsec -= NSEC_PER_SEC;
                   t.tv_sec++;
                   current_sec = t.tv_sec - start_sec;  // how many seconds since we started?
               }
        }
}

