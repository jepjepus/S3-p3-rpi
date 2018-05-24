#include<stdio.h>
#include<stdlib.h>
#include<time.h>
#include<unistd.h>
/* experiment 0, amb nanosleep

Description

nanosleep() suspends the execution of the calling thread until either at least the time specified in *req has elapsed, or the delivery of a signal that triggers the invocation of a handler in the calling thread or that terminates the process. 

Exemples d'execució (HP Elitebook 2570p, cpu:i5-3340m @ 2.7 GHz

$ ./a.out 
N:100 delay:  1 ms -> | min:  11.00 us | avg:  17.47 us | max:  32.00 us|
N:100 delay: 10 ms -> | min:  11.00 us | avg:  17.00 us | max:  42.00 us|
N:100 delay:100 ms -> | min:   9.00 us | avg:  21.65 us | max:  43.00 us|

$ nice -20 ./a.out 
N:100 delay:  1 ms -> | min:   8.00 us | avg:  17.58 us | max:  36.00 us|
N:100 delay: 10 ms -> | min:  13.00 us | avg:  23.30 us | max:  52.00 us|
N:100 delay:100 ms -> | min:   7.00 us | avg:  22.86 us | max:  55.00 us|

*/




#define N 100

float dtt[N];

void calculs(int n, useconds_t dela) // fer N vegades bucle amb retard de dela (microsegons)
{
  int i;
  clock_t t0, t1;
  struct timespec req, rem;
  req.tv_sec = 0;
  req.tv_nsec = dela*1000; // convert us to ns
  for (i=0;i<n;i++)
    {
      t0 = clock(); // obtenim temps inicial
      nanosleep(&req, &rem); // retard de dela microsegons
      t1 = clock(); // obtenim temps final després del retard
      dtt[i] = ((float)(t1 - t0)) / (CLOCKS_PER_SEC* 1.0E-6); // obtenim us de 'retard' en tornar del retard
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
  printf("N:%3d delay:%3.0f ms -> | min:%7.2f us | avg:%7.2f us | max:%7.2f us|\n", n, dela/1000.0, min, avg, max); 
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
