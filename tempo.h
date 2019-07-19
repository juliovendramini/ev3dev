
#ifndef TEMPO_H
#define TEMPO_H

#include <stdio.h>    /* Standard input/output definitions */
#include <stdlib.h> 
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include <time.h>
#include <pthread.h>
#include <chrono>
#include <thread>


struct timespec libTempoPassado;
int libTempoExecutando = 0;
volatile unsigned int libTempoMillisPassados = 0;

void* libTempoThreadMillis(void *nada){
    clock_gettime(CLOCK_MONOTONIC, &libTempoPassado);
    //libTempoZero.tv_nsec = 0;
    //libTempoZero.tv_sec = 0;
    long tempoAux, ultimoTempo;
    tempoAux = ultimoTempo = libTempoPassado.tv_nsec;
    while(1){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        clock_gettime(CLOCK_MONOTONIC, &libTempoPassado);
        tempoAux = (libTempoPassado.tv_nsec);
        if(tempoAux < ultimoTempo){
             libTempoMillisPassados += (999999999 - ultimoTempo + tempoAux)/1000000;
        }
        else{
            libTempoMillisPassados += (tempoAux - ultimoTempo)/1000000;
        }
        //printf("%d\n",(tempoAux - ultimoTempo)/1000000);
        ultimoTempo = tempoAux;
    }
    
}


unsigned int millis(){
    if(!libTempoExecutando){
        libTempoExecutando = 1 ;
        int err;
        pthread_t id_thread;
        err = pthread_create(&id_thread, NULL, &libTempoThreadMillis, NULL);
            if (err != 0)
                printf("\nNão foi possível iniciar a função tempo, favor contate seu professor: [%s]", strerror(err));
        return 0;
    }
    else{
        return libTempoMillisPassados;
    }
}

#endif
