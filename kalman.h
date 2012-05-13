
/*
 * file: kalman.h
 * 
 * implements kalman filter for linear system:
 *
 * | 1 dt | * | p | + | 0.5 * dt ^ 2 | * | a | = | p |
 * | 0  1 | * | v |   |           dt |           | v |
 * 
 * authors:
 *    Jan Roemisch, Ilmenau University of Technology
 *    Tobias Simon, Ilmenau University of Technology
 */


#ifndef __KALMAN_H__
#define __KALMAN_H__


typedef struct
{
   float pos;
   float speed;
}
kalman_out_t;


typedef struct
{
   float pos; /* position in m */
   //float speed; /* speed in m/s */
   float acc; /* acceleration min m/s^2 */
}
kalman_in_t;


typedef struct
{
   float dt; /* delta t in sec */
   float process_var;
   float measurement_var;
}
kalman_config_t;


struct kalman;
typedef struct kalman kalman_t;


/*
 * executes kalman predict and correct step
 */
void kalman_run(kalman_out_t *out, kalman_t *kalman, const kalman_in_t *in);


/*
 * allocates and initializes a kalman filter
 */
kalman_t *kalman_create(const kalman_config_t *config, const kalman_out_t *init_state);


/*
 * deletes the kalman filter
 */
void kalman_free(kalman_t *kalman);


#endif /* __KALMAN_H__ */

