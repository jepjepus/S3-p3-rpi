#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>


/* experiment 2, amb nanosleep

Description

nanosleep() suspends the execution of the calling thread until either at least the time specified in *req has elapsed, or the delivery of a signal that triggers the invocation of a handler in the calling thread or that terminates the process. 

Exemple d'execució (HP Elitebook 2570p, cpu:i5-3340m @ 2.7 GHz
(provat usant tty0tty = emulació de connexió entre 2 ports sèrie en la mateixa màquina)
https://sourceforge.net/projects/tty0tty/

$ ./exp2.out 
L:   1 | min:   0.00 us | avg:   1.56 us | max:   7.00 us|
L:  10 | min:   1.00 us | avg:   1.64 us | max:   2.00 us|
L:  20 | min:   1.00 us | avg:   1.68 us | max:   3.00 us|
L: 100 | min:   1.00 us | avg:   1.78 us | max:   3.00 us|
L: 200 | min:   1.00 us | avg:   1.62 us | max:   3.00 us|
L:1000 | min:   1.00 us | avg:   1.45 us | max:   2.00 us|
*/


#define N 100
float dtt[N];
char buffer[2000];

// open_port(): Open a serial port.
// Returns the file descriptor on success or -1 on error.
int open_port(void)
{
  char * port = "/dev/pts/1";
  int fd; /* File descriptor for the port */

  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
  {
   /* Could not open the port. */
    perror("open_port: Unable to open serial port - ");
  }
  else
    fcntl(fd, F_SETFL, 0);
  return (fd);
}

int close_port(int fd)
{
  close(fd);
}

// serial_read(): llegim l caràcters del port. es descarten.
void serial_read(int port, int l)
{
 read(port, buffer, l);
}

// calculs(): donats un port, una l,
// espera a tenir l caràcters al port i llegeix l caràcter/s.
// retorna dtt.
void calculs(int port, int l)
{
  int i, bytes_avail;
  clock_t t0, t1;
  for(i=0;i<N;i++)
    {
      do ioctl(port, FIONREAD, &bytes_avail);
	  while (bytes_avail<l); // hi ha l bytes_avail
     t0 = clock(); // obtenim temps inicial
     serial_read(port, l); // lectura (descartada) de l elements
     t1 = clock(); // obtenim temps final després del retard
     dtt[i] = ((float)(t1 - t0)) / (CLOCKS_PER_SEC * 1.0E-6); // obtenim us de 'retard'
    }
}

void resultats(int l, float t[N])
{
  int i;
  float min, max, avg;
  min=t[0]; max=t[0]; avg=t[0];
  for (i=1;i<N;i++)
    {
      if (t[i] < min) min=t[i];
      if (t[i] > max) max=t[i];
      avg += t[i];
    }
  avg /= N;
  printf("L:%4d | min:%7.2f us | avg:%7.2f us | max:%7.2f us|\n", l, min, avg, max); 
}

#define TESTS 6

void main(void)
{
  int port_serie;
  const int L[TESTS]={1,10,20,100,200,1000}; // tests de 0 a TESTS-1
  port_serie=open_port();
  for (int i=0;i<TESTS;i++)
    {
      calculs(port_serie, L[i]); // L[i] lectures de port sèrie
      resultats(L[i],dtt); // L[i] vegades; presenta min, max i avg
    }
  close_port(port_serie);
}
