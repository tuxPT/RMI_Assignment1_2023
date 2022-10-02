/* mainRob.C
*
* Basic Robot Agent
* Very simple version for demonstration
*
* For more information about the CiberRato Robot Simulator
* please see http://microrato.ua.pt/ or contact us.
*/

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>

#include "RobSock.h"

#include "robfunc.h"
#include "controller.h"

#define true 1
#define false 0

/*
* getXvel
*
* Computes current velocity along x axis.
*
* For internal use only.
*/
float getXvel(void);

int main(int argc, char *argv[])
{
  char host[100]="localhost";
  char rob_name[20]="robsample";
  float lPow,rPow;
  int rob_id = 1;
  char lmap[CELLROWS*2-1][CELLCOLS*2-1]; // in this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to lmap[i*2][j*2].
  // to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of lmap[i*2+1][j*2] is space or not
  float xVel;

  float velSetPoint=0.1;
  bool moving = false;

  printf( " Sample Robot\n Copyright (C) 2001-2019 Universidade de Aveiro\n" );

  /* processing arguments */
  while (argc > 2) /* every option has a value, thus argc must be 1, 3, 5, ... */
  {
    if (strcmp(argv[1], "--host") == 0 || strcmp(argv[1], "-h") == 0)
    {
      strncpy(host, argv[2], 99);
      host[99]='\0';
    }
    else if (strcmp(argv[1], "--robname") == 0 || strcmp(argv[1], "-r") == 0)
    {
      strncpy(rob_name, argv[2], 19);
      rob_name[19]='\0';
    }
    else if (strcmp(argv[1], "--pos") == 0 || strcmp(argv[1], "-p") == 0)
    {
      if(sscanf(argv[2], "%d", &rob_id)!=1)
      argc=0; /* error message will be printed */
    }
    else if (strcmp(argv[1], "--map") == 0 || strcmp(argv[1], "-m") == 0)
    {
      ReadMap(argv[2],lmap);
      for(int r=CELLROWS*2-2; r>=0; r--) {
        for(int c=0; c<CELLCOLS*2-1; c++) {
          printf("%c", lmap[r][c]);
        }
        printf("\n");
      }
    }
    else
    {
      break; /* the while */
    }
    argc -= 2;
    argv += 2;
  }

  if (argc != 1)
  {
    fprintf(stderr, "Bad number of parameters\n"
    "SYNOPSIS: mainRob [--host hostname] [--robname robotname] [--pos posnumber]\n");

    return 1;
  }

  /* Connect Robot to simulator */
  if(InitRobot(rob_name, rob_id, host)==-1)
  {
    printf( "%s Failed to connect\n", rob_name);
    exit(1);
  }
  printf( "%s Connected\n", rob_name );

  /* Open a file for writing values */
  FILE *fd=fopen("output.txt","w+");


  /* Main cycle */
  while(1)
  {

    /* Reading next values from Sensors */
    ReadSensors();

    if(GetFinished()) /* Simulator has received Finish() or Robot Removed */
    {
      printf(  "%s Exiting\n", rob_name );
      exit(0);
    }

    /* Test if reached end of labyrinth */
    if(GetX() > 26.0){
      printf("Reached end of maze! Terminating...\n");
      fclose(fd);
      exit(0);
    }

    if(GetStopButton() && (GetTime()>0) ){
      fclose(fd);
      printf("Stop button pressed! Terminating...\n");
      exit(0);
    }

    /* Change velocity setpoint according to X position */
    if(((int)GetX()/8) % 2){
      velSetPoint = 0.15;
    }
    else{
      velSetPoint = 0.05;
    }

    /* Read current speed */
    xVel = getXvel();

    /* Compute control value */
    lPow = rPow = controller(activeController, velSetPoint, xVel);

    /* Act on the system */
    DriveMotors(lPow,rPow);

    /* Test if experiment has started */
    if( GetStartButton()){
      moving = true;
    }

    /* If robot is moving, start printing values */
    if(moving){
      fprintf(fd,"%u\t",GetTime());
      fprintf(fd,"%4.5f\t%4.5f\t",GetX(),GetY());
      fprintf(fd,"%4.5f\t",velSetPoint);
      fprintf(fd,"%4.5f\t",xVel);
      fprintf(fd,"%4.5f\t",lPow);   /* lPow (or rPow) is equal to u */

      fprintf(fd,"\n");

      printf("%u\t",GetTime());
      printf("%4.5f\t%4.5f\t",GetX(),GetY());
      printf("%4.5f\t",velSetPoint);
      printf("%4.5f\t",xVel);
      printf("%4.5f\t",lPow);
      printf("\n");
    }
  }

  fclose(fd);
  return 1;
}

/**
* float getXvel(void)
*
* Computes current velocity along X axis.
*/
float getXvel(void){

  float xVel;
  float currentXpos;
  static float lastXpos=0;
  static int lastTime;
  int currTime;

  currentXpos = GetX();
  currTime = GetTime();

  if(currTime > lastTime){
    xVel = (currentXpos - lastXpos)/(currTime - lastTime);
  }
  else{
    xVel = 0;
  }

  /* Store for future memory... */
  lastXpos = currentXpos;
  lastTime = currTime;

  return xVel;

}
