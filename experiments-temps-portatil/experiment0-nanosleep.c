#include<stdio.h>
#include<stdlib.h>
#include<time.h>
#include<unistd.h>
/* experiment 0, amb nanosleep

Description

nanosleep() suspends the execution of the calling thread until either at least the time specified in *req has elapsed, or the delivery of a signal that triggers the invocation of a handler in the calling thread or that terminates the process. 

Exemples d'execució (HP Elitebook 2570p, cpu:i5-3340m @ 2.7 GHz

$ bin/experiment0-nanosleep 
N:100 delay:  1 ms -> | min:  63.43 us | avg:  74.48 us | max: 100.77 us|
N:100 delay: 10 ms -> | min:  53.54 us | avg:  97.46 us | max: 124.97 us|
N:100 delay:100 ms -> | min:  61.33 us | avg:  97.48 us | max: 217.57 us|

$ nice -20 bin/experiment0-nanosleep 
N:100 delay:  1 ms -> | min:  61.38 us | avg:  73.37 us | max:  90.88 us|
N:100 delay: 10 ms -> | min:  59.40 us | avg:  83.07 us | max: 141.79 us|
N:100 delay:100 ms -> | min:  60.86 us | avg:  93.08 us | max: 142.66 us|

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
  struct timespec req, rem;
  req.tv_sec = 0;
  req.tv_nsec = dela*1000; // convert us to ns
  for (i=0;i<n;i++)
    {
      clock_gettime(CLOCK_REALTIME, &t0); // obtenim temps inicial
      nanosleep(&req, &rem); // retard de dela microsegons
      clock_gettime(CLOCK_REALTIME, &t1); // obtenim temps final després del retard
      dtt[i] = RESTA_TIMESPEC(t1,t0) - req.tv_nsec;
      //dtt[i] = ((float)(t1 - t0)) / (CLOCKS_PER_SEC* 1.0E-6); // obtenim us de 'retard' en tornar del retard
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
  printf("N:%3d delay:%3.0f ms -> | min:%7.2f us | avg:%7.2f us | max:%7.2f us|\n", n, dela/1000.0, min/1000, avg/1000, max/1000); // mostrem microsegons
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
