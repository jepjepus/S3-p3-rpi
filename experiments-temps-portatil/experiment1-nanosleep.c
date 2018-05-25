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
(provat usant tty0tty = emulació de connexió entre 2 ports sèrie en la mateixa màquina)
https://sourceforge.net/projects/tty0tty/

(emulació, sense arduino, sense programar port a 115200)
$ ./exp1.out 
L:   1 |dela:         5 ms |dtt:     65.00 us|
L:  10 |dela:         5 ms |dtt:     40.00 us|
L:  20 |dela:         5 ms |dtt:     34.00 us|
L: 100 |dela:        25 ms |dtt:      6.00 us|
L: 200 |dela:        50 ms |dtt:      5.00 us|
L:1000 |dela:       250 ms |dtt:     39.00 us|

(amb arduino port a 115200, enviant 'A' contínuament)
$ ./exp1-n.out 
L:   1 |dela:         5 ms |dtt:     63.00 us|
L:  10 |dela:         5 ms |dtt:     41.00 us|
L:  20 |dela:         5 ms |dtt:     26.00 us|
L: 100 |dela:        25 ms |dtt:      4.00 us|
L: 200 |dela:        50 ms |dtt:      5.00 us|
L:1000 |dela:       250 ms |dtt:     46.00 us|

(amb arduino port a 115200, enviant binari 8 bits - port obert per binari!)
$ ./exp1-n.out 
L:   1 |dela:         5 ms |dtt:     71.00 us|
L:  10 |dela:         5 ms |dtt:     49.00 us|
L:  20 |dela:         5 ms |dtt:     33.00 us|
L: 100 |dela:        25 ms |dtt:     12.00 us|
L: 200 |dela:        50 ms |dtt:      5.00 us|
L:1000 |dela:       250 ms |dtt:     31.00 us|



*/

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
    options.c_cflag |= (CLOCAL | CREAD); // ?? enable receivcer and set local mode
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

// calculs(): donats un port, una l i un retard dela,
// espera dela i llegeix l caràcter/s.
// retorna dtt.
float calculs(int port, int l, useconds_t dela)
{
  clock_t t0, t1;
  struct timespec req, rem;
  req.tv_sec = 0;
  req.tv_nsec = dela*1000; // convert us to ns
  t0 = clock(); // obtenim temps inicial
  nanosleep(&req, &rem); // retard de dela microsegons
  serial_read(port, l); // lectura (descartada) de l elements
  t1 = clock(); // obtenim temps final després del retard
 return ((float)(t1 - t0)) / (CLOCKS_PER_SEC * 1.0E-6); // obtenim us de 'retard'
}

void resultats(int l, long nsdela, float dtt)
{
  printf("L:%4d |dela:%10d ms |dtt:%10.2f us|\n", l, nsdela/1000000, dtt); 
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
  for (i=0;i<TESTS;i++)
    {
      dtt=calculs(port_serie, L[i], nsdela[i]); // L[i] lectures de port sèrie
      resultats(L[i],nsdela[i],dtt); // per a cada L[i], mostrem dtt
    }
  close_port(port_serie);
}
