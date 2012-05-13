
#include "kalman.h"

#include <stdio.h>
#include <stdlib.h>

int main(void)
{
   srand(1);
   kalman_out_t kalman_state = {100.0, 0.0};
   
   kalman_config_t lateral_kalman_config = 
   {
      0.03, 0.0001, 1.01
   };

   /* create kalman filters: */
   kalman_t *kalman = kalman_create(&lateral_kalman_config, &kalman_state);
   int i;
   for (i = 0; i < 1000; i++)
   {
      float z = 5.0 + 2 * (float)rand() / (float)RAND_MAX + i / 100.0;
      float a = 1.0 - 0.5 * (float)rand() / (float)RAND_MAX;
      kalman_in_t in = {z, a};
      kalman_out_t out;
      kalman_run(&out, kalman, &in);
      printf("%f %f %f\n", z, out.pos, out.speed);
   }
}


