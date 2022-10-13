
#include <stdio.h>
#include "controller.h"

int main(void){

  float e,u;

  while(scanf("%f\n", &e)!=EOF){
    u = controller(CONTROLMODE,e);
    printf("%f\t%f\n",e,u);
  }

}
