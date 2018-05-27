# S3-p3-rpi

Pràctica de sistemes encastats.

Detecció de DTMF amb Raspberry pi o PC comunicada via sèrie usb amb un Arduino.

L'Arduino realitza l'adquisició de dades utilitzant la Shield DIPSE.

Carpetes:

arduino_AD2serial: L'Arduino connecta via port sèrie USB amb el programa que executa l'algorisme de Goertzel.

DTMF_Rpi: Programa per a PC/Raspberry Pi amb algorisme de Goertzel que processa finestres N=196 rebudes des d'un Arduino via port sèrie.

experiments-temps-portatil: Experiments 0, 1 i 2 que mostren valors dels retards generats en les crides a sistema operatiu per accedir al port sèrie.

