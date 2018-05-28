#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */



/* experiment 1, amb nanosleep

Description

nanosleep() suspends the execution of the calling thread until either at least the time specified in *req has elapsed, or the delivery of a signal that triggers the invocation of a handler in the calling thread or that terminates the process. 

Exemples d'execució (HP Elitebook 2570p, cpu:i5-3340m @ 2.7 GHz
(abans de provar amb Arduino es provà usant tty0tty = emulació de connexió entre 2 ports sèrie en la mateixa màquina) (/dev/pts/3 <-> /dev/pts4)
https://sourceforge.net/projects/tty0tty/

(amb arduino port a 115200, enviant binari 8 bits - port obert per binari!)
$ bin/experiment1-nanosleep 
L:   1 |dela:         5 ms |dtt:      6.03 us|
L:  10 |dela:         5 ms |dtt:      2.61 us|
L:  20 |dela:         5 ms |dtt:      1.92 us|
L: 100 |dela:        25 ms |dtt:      9.09 us|
L: 200 |dela:        50 ms |dtt:      8.71 us|
L:1000 |dela:       250 ms |dtt:     12.65 us|

$ bin/experiment1-nanosleep 
L:   1 |dela:         5 ms |dtt:      8.08 us|
L:  10 |dela:         5 ms |dtt:      5.72 us|
L:  20 |dela:         5 ms |dtt:      5.71 us|
L: 100 |dela:        25 ms |dtt:     12.60 us|
L: 200 |dela:        50 ms |dtt:     10.95 us|
L:1000 |dela:       250 ms |dtt:      9.69 us|

$ bin/experiment1-nanosleep 
L:   1 |dela:         5 ms |dtt:     11.99 us|
L:  10 |dela:         5 ms |dtt:     13.71 us|
L:  20 |dela:         5 ms |dtt:     11.16 us|
L: 100 |dela:        25 ms |dtt:      8.91 us|
L: 200 |dela:        50 ms |dtt:      2.21 us|
L:1000 |dela:       250 ms |dtt:     21.87 us|

(amb tty0tty en emulació /dev/pts/3 <-> /dev/pts4.
Estrany que el codi 'llegeix' tot i no estar enviant el remot)
$ sudo bin/experiment1-nanosleep 
L:   1 |dela:         5 ms |dtt:     15.68 us|
L:  10 |dela:         5 ms |dtt:     12.49 us|
L:  20 |dela:         5 ms |dtt:     11.18 us|
L: 100 |dela:        25 ms |dtt:     12.70 us|
L: 200 |dela:        50 ms |dtt:      4.52 us|
L:1000 |dela:       250 ms |dtt:     20.47 us|

*/

// Definició RESTA_TIMESPEC(): calcula t1-t0,
// valors obtinguts de clock_gettime(). Dona resultat en nanosegons
#define RESTA_TIMESPEC(t1,t0) (((t1.tv_sec - t0.tv_sec) * 1E9) + (t1.tv_nsec - t0.tv_nsec))

char buffer[2000];


// open_port(): Open a serial port.
// Returns the file descriptor on success or -1 on error.
int open_port(void)
{
  //char * port = "/dev/pts/3"; // for emulated serial port
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
    options.c_cflag |= (CLOCAL | CREAD); // ?? enable receiver and set local mode
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

// calculs(): donats un port, una l i un retard dela (ns),
// espera dela i llegeix l caràcter/s.
// retorna dtt en nanosegons.
float calculs(int port, int l, long dela)
{
  struct timespec t0, t1;
  struct timespec req, rem;
  req.tv_sec = 0;
  req.tv_nsec = dela; // delay in ns (less than 1 s!)
  nanosleep(&req, &rem); // retard de dela microsegons
  clock_gettime(CLOCK_REALTIME, &t0); // obtenim temps inicial
  serial_read(port, l); // lectura (descartada) de l elements
  clock_gettime(CLOCK_REALTIME, &t1); // obtenim temps final després del retard
 return RESTA_TIMESPEC(t1,t0); // obtenim us de 'retard' en ns
}

void resultats(int l, long nsdela, float dtt)
{
  printf("L:%4d |dela:%10ld ms |dtt:%10.2f us|\n", l, nsdela/1000000, dtt/1000); 
}

#define TESTS 6

void main(void)
{
  int port_serie;
  const int L[TESTS]={1,10,20,100,200,1000}; // tests de 0 a TESTS-1
  const long nsdela[TESTS]={5000000,5000000,5000000,25000000,50000000,250000000};
  int i;
  float dtt;
  port_serie=open_port();
  tcflush(port_serie,TCIFLUSH); //buidem caràcters d'entrada port sèrie
  for (i=0;i<TESTS;i++)
    {
      dtt=calculs(port_serie, L[i], nsdela[i]); // L[i] lectures de port sèrie
      resultats(L[i],nsdela[i],dtt); // per a cada L[i], mostrem dtt
    }
  close_port(port_serie);
}
