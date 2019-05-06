#include "../timer.h"
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
void timer_deinit(void){}
void _T1Interrupt();

#include <time.h>
struct timespec past;
int avg=0;
int count = 0;
int count_n = 1000;

int avg2=1000000;
int vals[10]={1000000,1000000,1000000,1000000,1000000,1000000,1000000,1000000,1000000,1000000};
int a_ptr=0;
#include <stdlib.h>

struct timespec diff(struct timespec start, struct timespec end) {
    struct timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}

static void time_measure_diff() {
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	struct timespec tmp=diff(past,now);
	
	avg += tmp.tv_nsec;
	if(++count >= count_n) {
		int passed = avg/count;
		// printf("time passed: %d\n", passed);
		
		int cur = avg2 * 10 - vals[a_ptr];
		vals[a_ptr++] = passed;
		a_ptr = a_ptr % 10;
		avg2 = (cur + passed) / 10;
		
		count = 0;
		avg=0;
	}
	past=now;
}

static void* timer_thread() {
	unsigned int d_calc = 1000000-5000;
	int t=0;
	while(1) {
		time_measure_diff();
		regulator_interrupt();
		struct timespec past,now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		past=now;
		
		// printf("time passed: %ld\n", tmp.tv_nsec);
		past=now;
		usleep(800); // 1ms timer
		clock_gettime(CLOCK_MONOTONIC, &now);
		unsigned int d = diff(past,now).tv_nsec;
		
		while(d < d_calc) {
		// while(d < 1000000) {
			clock_gettime(CLOCK_MONOTONIC, &now);
			d = diff(past,now).tv_nsec;
		}
		
		if(++t > 1000) {
			if(avg2 < 1500000) {
				d_calc += (1000000 - avg2) / 10;
				// printf("fix with: %u %u\n", d_calc, avg2);
			}
			t=0;
		}
	}
}

void timer_init(void) {
	pthread_t thread;
	pthread_create(&thread, NULL, timer_thread, 0);
	pthread_detach(thread);
}
