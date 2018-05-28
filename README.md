# S3-p3-rpi

Pràctica de sistemes encastats.

Detecció de DTMF amb Raspberry pi o PC comunicada via sèrie usb amb un Arduino.
L'Arduino realitza l'adquisició de dades utilitzant la Shield DIPSE.

Carpetes:

a) arduino_AD2serial: L'Arduino connecta via port sèrie USB amb el programa que executa l'algorisme de Goertzel.

b) DTMF_Rpi: Programa per a PC/Raspberry Pi amb algorisme de Goertzel que processa finestres N=196 rebudes des d'un Arduino via port sèrie.

c) experiments-temps-portatil: Experiments 0, 1 i 2 que mostren valors dels retards generats en les crides a sistema operatiu per accedir al port sèrie.

Cada carpeta disposa d'un Makefile. Es compila+enllaça cridant "make" dins de cada carpeta.

a) es genera el codi ARduino amb make i es puja via sèrie.
b) i c) es compilen amb make. La carpeta ./bin conté els executables resultants.
