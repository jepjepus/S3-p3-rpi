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
#include <math.h>


/* DTMF Rpi

Detector de codis DTMF basat en l'algorisme de Goertzel.

Rebrà de l'Arduino, per port sèrie (115200-N-8-1), les dades de la finestra de mostratge.

Per als càlculs de temps d'utilitza nanosleep():
nanosleep() suspends the execution of the calling thread until either at least the time specified in *req has elapsed, or the delivery of a signal that triggers the invocation of a handler in the calling thread or that terminates the process.

*/

#define N 196 // longitud de finestra
#define FM 8000 // freqüència de mostratge
#define NUMFREQ 8 // seran les freqüències de 0 a 7

const float FS[NUMFREQ]={697,770,852,941,1209,1336,1477,1633};
const long LLINDAR[NUMFREQ]={400000L,400000L,400000L,400000L,400000L,400000L,200000L,110000L}; //maxim dels falsos mes un 10%
long XK2[NUMFREQ]; //el fem long per comparar amb LLINDAR

const char dial_pad[4][4]={{'1','2','3','A'},{'4','5','6','B'},{'7','8','9','C'},{'*','0','#','D'}};

float K[NUMFREQ]; // valors K per a cada freqüència. float que anem 'sobrats'
float A[NUMFREQ]; // valors A per a cada freqüència. És float, té decimals 

// init_A_FS(): calcula valors de K i A per a Goertzel
void init_A_FS(void)
{
  int i;
  for(i=0;i<NUMFREQ;i++){
    K[i]=round(FS[i]*N/FM);
    A[i]=2*cos(2*M_PI*(K[i]/N));  
  }
}


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

// serial_read(): llegim n caràcters del port i es desen a buf.
void serial_read(int port, unsigned char * buf, int n)
{
 read(port, buf, n);
}

// calculs(): donats un port, una l,
// espera a tenir l caràcters al port i llegeix l caràcter/s.
// retorna dtt.
/* void calculs(int port, int l) */
/* { */
/*   int i, bytes_avail; */
/*   clock_t t0, t1; */
/*   for(i=0;i<N;i++) */
/*     { */
/*      t0 = clock(); // obtenim temps inicial */
/*      serial_read(port, l); // lectura (descartada) de l elements */
/*      t1 = clock(); // obtenim temps final després del retard */
/*      dtt[i] = ((float)(t1 - t0)) / (CLOCKS_PER_SEC * 1.0E-6); // obtenim us de 'retard' */
/*     } */
/* } */

/* void resultats(int l, float t[N]) */
/* { */
/*   int i; */
/*   float min, max, avg; */
/*   min=t[0]; max=t[0]; avg=t[0]; */
/*   for (i=1;i<N;i++) */
/*     { */
/*       if (t[i] < min) min=t[i]; */
/*       if (t[i] > max) max=t[i]; */
/*       avg += t[i]; */
/*     } */
/*   avg /= N; */
/*   printf("L:%4d | min:%7.2f us | avg:%7.2f us | max:%7.2f us|\n", l, min, avg, max);  */
/* } */

void llegeix_finestra(int port, unsigned char * buf, int n)
{
  int bytes_avail;
  do ioctl(port, FIONREAD, &bytes_avail);
  while (bytes_avail<n); // hi ha n bytes_avail
  serial_read(port, buf, n);
  /* for(int i=0;i<n;i++) printf("%x",buf[i]); */
  /* printf("\n"); */
}

const char calc_boto(void){ //CAPA 2: detecció de les 2 freqüències + botó corresponent
  int low=-1, high=-1;
  int i, count;
  for (count=0, i=0; i<4; i++){ //mirem freq. baixes
    if (LLINDAR[i]<XK2[i]){
       count++;
       if (count>1) return 'E'; //mes de 2 freq. detectades: error
       low=i;
    }
  } 
  for (count=0, i=0; i<4; i++){ //mirem freq. altes
    if (LLINDAR[i+4]<XK2[i+4]){
       count++;
       if (count>1) return 'E'; //mes de 2 freq. detectades: error
       high=i;
    }
  }
  if((low==-1)&&(high==-1)) return 'S'; //cap freq. detectada: space
  if((low==-1)||(high==-1)) return 'E'; //detectada 1 freq. sola: error
  //printf("%c",dial_pad[low][high]);
  return dial_pad[low][high]; // low i high tenen les posicions freq. detectades: key
}

void capa_3(char capa_2){ //CAPA 3: màquina d'estats. Filtre casos d'error, espais, durada de pulsació de boto...
  static int estat=0;
  //printf("[%d]",estat);
  switch (estat){
  case 0:
    if (!((capa_2=='S') || (capa_2=='E'))) {
      printf("%c\n", capa_2); estat=1;
      return;
    }
    //else{printf("-");}
    break;
  case 1:
    if (capa_2=='S') {
      estat=0;
      return;
    }
    //else printf("%d",estat);
    break;
  default:
    printf("%d",estat);
  }
}
 
void goertzel(unsigned char * buf, int n)
{
  char rebut; // dígit rebut
  float Sn[NUMFREQ], Sn_1[NUMFREQ]={0,0,0,0,0,0,0,0}, Sn_2[NUMFREQ]={0,0,0,0,0,0,0,0};
  int i,j;
  for(j=0;j<n;j++) // tractem finestra. per a cada mostra j...
    {
      for(i=0;i<NUMFREQ;i++) // per a cada freqüència, calculem filtre
	{
	  Sn[i]= buf[j] + A[i]*Sn_1[i] - Sn_2[i];
	  Sn_2[i]=Sn_1[i];
	  Sn_1[i]=Sn[i];
	}
    }  
  for (i=0;i<NUMFREQ;i++) // calculem XK2 per a cada frequencia
    {
      XK2[i] = Sn_1[i]*Sn_1[i] + Sn_2[i]*Sn_2[i] - A[i]*Sn_1[i]*Sn_2[i];
      // printf("%8ld ",XK2[i]);
    }
  //printf("XK2\n");
  // valors calculats. Ara capa 2
  rebut=calc_boto(); // determinem el que es rep: caracter, espai(S) o error(E) 
  //printf("%c\n", rebut);
  capa_3(rebut); // la capa3 es maquina d'estats que determina el caracter rebut, L'envia a pantalla. 
}
 
void main(void)
{
  int port_serie;
  unsigned char buffer[N+1]; // buffer per recollir les dades del port sèrie
  init_A_FS(); // Càlculs previs: A per a Goertzel
  //for(int i=0;i<NUMFREQ;i++) printf("%10f %10f\n",K[i], A[i]);
  //exit(0);
  port_serie=open_port();
  printf("Port sèrie obert.\n");
  while(-1)
    {
      //printf("I");
      llegeix_finestra(port_serie, buffer, N); //  lectura N valors de port sèrie
      //printf("L\n");
      goertzel(buffer,N);

      /* calculs(buffer, N); */
      /* resultats(L[i],dtt); // L[i] vegades; presenta min, max i avg */
    }
  close_port(port_serie);
}
