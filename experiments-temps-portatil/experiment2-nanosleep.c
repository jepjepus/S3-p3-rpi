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
(amb arduino port a 115200, enviant binari 8 bits - port obert per binari!)

$ bin/experiment2-nanosleep 
L:   1 | min:   0.45 us | avg:   0.52 us | max:   3.37 us|
L:  10 | min:   0.44 us | avg:   0.61 us | max:   2.74 us|
L:  20 | min:   0.46 us | avg:   0.72 us | max:   2.70 us|
L: 100 | min:   0.61 us | avg:   0.95 us | max:   3.03 us|
L: 200 | min:   0.68 us | avg:   1.11 us | max:   3.16 us|
L:1000 | min:   0.88 us | avg:   1.89 us | max:   3.76 us|

$ bin/experiment2-nanosleep 
L:   1 | min:   0.45 us | avg:   0.57 us | max:   8.87 us|
L:  10 | min:   0.46 us | avg:   0.65 us | max:   2.54 us|
L:  20 | min:   0.46 us | avg:   1.14 us | max:   2.79 us|
L: 100 | min:   0.62 us | avg:   0.88 us | max:   3.14 us|
L: 200 | min:   0.70 us | avg:   1.37 us | max:   3.18 us|
L:1000 | min:   0.76 us | avg:   1.97 us | max:   4.03 us|

*/

// Definició RESTA_TIMESPEC(): calcula t1-t0,
// valors obtinguts de clock_gettime(). Dona resultat en nanosegons
#define RESTA_TIMESPEC(t1,t0) (((t1.tv_sec - t0.tv_sec) * 1E9) + (t1.tv_nsec - t0.tv_nsec))

#define N 100
float dtt[N];
char buffer[2000];

// open_port(): Open a serial port.
// Returns the file descriptor on success or -1 on error.
int open_port(void)
{
  //char * port = "/dev/pts/1"; // for emulated serial port
  char * port = "/dev/ttyUSB3"; // for real serial port to Arduino
  int fd; /* File descriptor for the port */

  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
  {
   /* Could not open the port. */
    perror("open_port: Unable to open serial port - ");
  }
  else
    {
    fcntl(fd, F_SETFL, 0); // normal (blocking) behavior
    struct termios options;
    tcgetattr(fd, &options); //this gets the current options set for the port

    // setting the options

    cfsetispeed(&options, B115200); //input baudrate
    cfsetospeed(&options, B115200); // output baudrate
    options.c_cflag |= (CLOCAL | CREAD); // ?? enable receicer and set local mode
    //options.c_cflag &= ~CSIZE; /* mask the character size bits */
    options.c_cflag |= CS8;    /* select 8 data bits */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // choosing raw input
    options.c_iflag &= ~INPCK; // disable parity check
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // disable software flow control
    options.c_oflag |= OPOST; // ?? choosing processed output
    options.c_cc[VMIN] = 0; // Wait until x bytes read (blocks!)
    options.c_cc[VTIME] = 0; // Wait x * 0.1s for input (unblocks!)

    // settings for no parity bit
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    tcsetattr(fd, TCSANOW, &options); //set the new options ... TCSANOW specifies all option changes to occur immediately
    }
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
void calculs(int port, int l)
{
  int i, bytes_avail;
  struct timespec t0, t1;
  for(i=0;i<N;i++)
    {
      do ioctl(port, FIONREAD, &bytes_avail);
	  while (bytes_avail<l); // hi ha l bytes_avail
      clock_gettime(CLOCK_REALTIME, &t0); // obtenim temps inicial
      serial_read(port, l); // lectura (descartada) de l elements
      clock_gettime(CLOCK_REALTIME, &t1); // obtenim temps final després del retard
      dtt[i] = RESTA_TIMESPEC(t1,t0); // obtenim durada
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
  printf("L:%4d | min:%7.2f us | avg:%7.2f us | max:%7.2f us|\n", l, min/1000, avg/1000, max/1000); 
}

#define TESTS 6

void main(void)
{
  int port_serie;
  const int L[TESTS]={1,10,20,100,200,1000}; // tests de 0 a TESTS-1
  port_serie=open_port();
  tcflush(port_serie,TCIFLUSH); //buidem caràcters d'entrada port sèrie
  for (int i=0;i<TESTS;i++)
    {
      calculs(port_serie, L[i]); // L[i] lectures de port sèrie
      resultats(L[i],dtt); // L[i] vegades; presenta min, max i avg
    }
  close_port(port_serie);
}
