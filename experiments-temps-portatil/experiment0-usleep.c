#include<stdio.h>
#include<stdlib.h>
#include<time.h>
#include<unistd.h>
/* experiment 0, amb usleep()

usleep() description:
The usleep() function suspends execution of the calling thread for (at least) usec microseconds. The sleep may be lengthened slightly by any system activity or by the time spent processing the call or by the granularity of system timers.


Exemples d'execució (HP Elitebook 2570p, cpu:i5-3340m @ 2.7 GHz

$ bin/experiment0-usleep 
N:100 delay:  1 ms -> | min:  66.14 us | avg:  77.92 us | max: 105.57 us|
N:100 delay: 10 ms -> | min:  59.31 us | avg:  93.55 us | max: 124.17 us|
N:100 delay:100 ms -> | min:  60.57 us | avg:  92.03 us | max: 159.50 us|

$ nice -20 bin/experiment0-usleep 
N:100 delay:  1 ms -> | min:  56.94 us | avg:  70.30 us | max:  79.44 us|
N:100 delay: 10 ms -> | min:  58.82 us | avg:  93.71 us | max: 125.86 us|
N:100 delay:100 ms -> | min:  54.46 us | avg:  89.16 us | max: 130.62 us|
*/

// Definició RESTA_TIMESPEC(): calcula t1-t0,
// valors obtinguts de clock_gettime(). Dona resultat en nanosegons
#define RESTA_TIMESPEC(t1,t0) (((t1.tv_sec - t0.tv_sec) * 1E9) + (t1.tv_nsec - t0.tv_nsec))

#define N 100

float dtt[N];

void calculs(int n, useconds_t dela) // fer N vegades bucle amb retard de dela (microsegons)
{
  int i;
  struct timespec t0, t1;
  for (i=0;i<n;i++)
    {
      clock_gettime(CLOCK_REALTIME, &t0); // obtenim temps inicial
      usleep(dela); // retard de dela microsegons
      clock_gettime(CLOCK_REALTIME, &t1); // obtenim temps final després del retard
      dtt[i] = RESTA_TIMESPEC(t1,t0) - dela*1000L;
    }
}

void resultats(int n, useconds_t dela, float t[N])
{
  int i;
  float min, max, avg;
  min=t[0]; max=t[0]; avg=t[0];
  for (i=1;i<n;i++)
    {
      if (t[i] < min) min=t[i];
      if (t[i] > max) max=t[i];
      avg += t[i];
    }
  avg /= n;
  printf("N:%3d delay:%3.0f ms -> | min:%7.2f us | avg:%7.2f us | max:%7.2f us|\n", n, dela/1000.0, min/1000, avg/1000, max/1000); 
}
  
void main(void)
{
  calculs(N,1000);   // N vegades, 1 ms (1000 us)
  resultats(N,1000,dtt); // presenta min, max i avg
  calculs(N,10000);  // N vegades, 10 ms
  resultats(N,10000,dtt); // presenta min, max i avg
  calculs(N,100000); // N vegades, 100 ms
  resultats(N,100000,dtt); // presenta min, max i avg
}
