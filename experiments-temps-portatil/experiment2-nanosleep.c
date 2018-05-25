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

(emulació, sense arduino, sense programar port a 115200)
$ ./exp2.out 
L:   1 | min:   0.00 us | avg:   1.56 us | max:   7.00 us|
L:  10 | min:   1.00 us | avg:   1.64 us | max:   2.00 us|
L:  20 | min:   1.00 us | avg:   1.68 us | max:   3.00 us|
L: 100 | min:   1.00 us | avg:   1.78 us | max:   3.00 us|
L: 200 | min:   1.00 us | avg:   1.62 us | max:   3.00 us|
L:1000 | min:   1.00 us | avg:   1.45 us | max:   2.00 us|

(amb arduino port a 115200, enviant 'A' contínuament)
$ ./exp2-n.out 
L:   1 | min:   0.00 us | avg:   1.78 us | max:   9.00 us|
L:  10 | min:   0.00 us | avg:   0.88 us | max:   2.00 us|
L:  20 | min:   0.00 us | avg:   0.96 us | max:   3.00 us|
L: 100 | min:   1.00 us | avg:   1.41 us | max:   3.00 us|
L: 200 | min:   1.00 us | avg:   1.93 us | max:   5.00 us|
L:1000 | min:   1.00 us | avg:   2.91 us | max:   6.00 us|

(amb arduino port a 115200, enviant binari 8 bits - port obert per binari!)
$ ./exp2-n.out 
L:   1 | min:   0.00 us | avg:   1.69 us | max:   8.00 us|
L:  10 | min:   0.00 us | avg:   1.02 us | max:   3.00 us|
L:  20 | min:   0.00 us | avg:   1.09 us | max:   2.00 us|
L: 100 | min:   1.00 us | avg:   1.53 us | max:   4.00 us|
L: 200 | min:   1.00 us | avg:   1.51 us | max:   5.00 us|
L:1000 | min:   1.00 us | avg:   2.56 us | max:   5.00 us|



*/


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
