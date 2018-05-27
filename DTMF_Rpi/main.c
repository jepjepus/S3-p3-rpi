#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <stdio_ext.h> /* to check buffer size? */
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

Temps obtinguts (en buit, sense injectar senyal però amb Arduino, és clar):

 |Tser:28229.00 us|Tgoe:   9.00 us|Total:28238.00 us|
 |Tser:24205.00 us|Tgoe:   8.00 us|Total:24213.00 us|
 |Tser:24219.00 us|Tgoe:   8.00 us|Total:24227.00 us|
 |Tser:24253.00 us|Tgoe:   8.00 us|Total:24261.00 us|
 |Tser:24228.00 us|Tgoe:   8.00 us|Total:24236.00 us|
 |Tser:24224.00 us|Tgoe:   9.00 us|Total:24233.00 us|
 |Tser:24239.00 us|Tgoe:   9.00 us|Total:24248.00 us|
 |Tser:24219.00 us|Tgoe:   9.00 us|Total:24228.00 us|
 |Tser:28260.00 us|Tgoe:   8.00 us|Total:28268.00 us|
 |Tser:24170.00 us|Tgoe:   8.00 us|Total:24178.00 us|
 |Tser:24236.00 us|Tgoe:   9.00 us|Total:24245.00 us|
 |Tser:24221.00 us|Tgoe:   9.00 us|Total:24230.00 us|
 |Tser:24217.00 us|Tgoe:   8.00 us|Total:24225.00 us|
 |Tser:24184.00 us|Tgoe:   9.00 us|Total:24193.00 us|
 |Tser:24197.00 us|Tgoe:   9.00 us|Total:24206.00 us|
 |Tser:24248.00 us|Tgoe:   9.00 us|Total:24257.00 us|
 |Tser:28264.00 us|Tgoe:   9.00 us|Total:28273.00 us|

Temps obtinguts descodificant (test8.wav):

0
 |Tser:24244.00 us|Tgoe:  18.00 us|Total:24262.00 us|
 |Tser:28241.00 us|Tgoe:   9.00 us|Total:28250.00 us|
 |Tser:24215.00 us|Tgoe:   9.00 us|Total:24224.00 us|
 |Tser:24205.00 us|Tgoe:   9.00 us|Total:24214.00 us|
 |Tser:24204.00 us|Tgoe:   8.00 us|Total:24212.00 us|
 |Tser:24201.00 us|Tgoe:   9.00 us|Total:24210.00 us|
 |Tser:24210.00 us|Tgoe:   9.00 us|Total:24219.00 us|
 |Tser:24230.00 us|Tgoe:   9.00 us|Total:24239.00 us|
A
 |Tser:24194.00 us|Tgoe:  29.00 us|Total:24223.00 us|
 |Tser:28263.00 us|Tgoe:   9.00 us|Total:28272.00 us|
 |Tser:24192.00 us|Tgoe:   9.00 us|Total:24201.00 us|
 |Tser:24234.00 us|Tgoe:   9.00 us|Total:24243.00 us|
 |Tser:24211.00 us|Tgoe:   9.00 us|Total:24220.00 us|
 |Tser:24178.00 us|Tgoe:   9.00 us|Total:24187.00 us|
 |Tser:24210.00 us|Tgoe:  15.00 us|Total:24225.00 us|
 |Tser:24240.00 us|Tgoe:   9.00 us|Total:24249.00 us|
 |Tser:24194.00 us|Tgoe:   8.00 us|Total:24202.00 us|
B
 |Tser:28271.00 us|Tgoe:  29.00 us|Total:28300.00 us|
 |Tser:24220.00 us|Tgoe:   8.00 us|Total:24228.00 us|
 |Tser:24186.00 us|Tgoe:  10.00 us|Total:24196.00 us|
 |Tser:24242.00 us|Tgoe:   8.00 us|Total:24250.00 us|
 |Tser:24159.00 us|Tgoe:  10.00 us|Total:24169.00 us|
 |Tser:24280.00 us|Tgoe:   8.00 us|Total:24288.00 us|
 |Tser:24264.00 us|Tgoe:   9.00 us|Total:24273.00 us|
 |Tser:24138.00 us|Tgoe:   8.00 us|Total:24146.00 us|
C
 |Tser:28269.00 us|Tgoe:  16.00 us|Total:28285.00 us|
 |Tser:24220.00 us|Tgoe:   9.00 us|Total:24229.00 us|
 |Tser:24191.00 us|Tgoe:   9.00 us|Total:24200.00 us|
 |Tser:24233.00 us|Tgoe:   8.00 us|Total:24241.00 us|

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
    exit(1); //afegida perquè si no obre, acabi programa
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
    //options.c_cflag &= ~CRTSCTS; // disable RTS/CTS
    //options.c_cflag &= ~CSIZE; /* mask the character size bits */
    options.c_cflag |= CS8;    /* select 8 data bits */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // choosing raw input
    // options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | TOSTOP); // choosing raw input, no signal does not affect to flush time
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
  //tcflush(port,TCIOFLUSH); //prova de reset de bufer
  //tcdrain(port); //prova de reinici de bufer
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
      printf("%c", capa_2); estat=1; // cal cridar fflush() per forçar el buidatge de buffer de printf si no cal acabar printf amb \n
      //fflush(stdout); //alternativa a fflush(stdout): setbuf(stdout, NULL) a l'inici del programa
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

int main(int argc, char * argv[]) // poden entrar -debug per mostrar temps
{
  int debug=0;
  clock_t t0, t1, t2;
  float T1, T2;
  int port_serie;
  unsigned char buffer[N+1]; // buffer per recollir les dades del port sèrie
  setbuf(stdout, NULL); //alternativa a fer fflush(stdout) a cada printf() sense \n per mostrar-lo immediatament
  init_A_FS(); // Càlculs previs: A per a Goertzel
  printf("UPC-EPSEM-Sistemes Encastatas -  Pràctica 3 - PC i Raspberry Pi\n");
  printf("Descodificació DTMF.\n");
  if ((argc==2) && strcmp(argv[1],"-debug")==0) debug=-1; // han entrat -debug" a línia d'ordres
  if (debug) printf("Mode debug activat\n");
  else printf("Teniu l'opció -debug per mostrar els temps de finestra i de càlcul\n");
  port_serie=open_port();
  printf("Port sèrie obert.\n");
  while(-1)
    {
      t0 = clock(); // obtenim temps inicial 
      llegeix_finestra(port_serie, buffer, N); //  lectura N valors de port sèrie
      t1 = clock(); // obtenim temps final després del retard
      goertzel(buffer,N);
      t2 = clock(); // obtenim temps final després del retard
      T1 = ((float)(t1 - t0)) / (CLOCKS_PER_SEC * 1.0E-6); // obtenim us de la lectura
      T2 = ((float)(t2 - t1)) / (CLOCKS_PER_SEC * 1.0E-6); // obtenim us de la lectura
      if (debug) printf(" |Tser:%7.2f us|Tgoe:%7.2f us|Total:%7.2f us|\n", T1, T2, T1+T2);
    }
   close_port(port_serie);

}
