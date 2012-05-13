/*
 * file: kalman_meschach.c
 *
 * implements kalman filter for linear system:
 *
 * | 1 dt | * | p | + | 0.5 * dt ^ 2 | * | a | = | p |
 * | 0  1 | * | v |   |     dt       |   | v |
 *
 * authors:
 * Tobias Simon, Ilmenau University of Technology
 * Jan Roemisch, Ilmenau University of Technology
 */


#include <meschach/matrix.h>
#include <meschach/matrix2.h>


#include "kalman.h"


struct kalman
{
   /* configuration and constant matrices: */
   MAT *Q; /* process noise */
   MAT *R; /* measurement noise */
   MAT *I; /* identity matrix */

   /* state and transition vectors/matrices: */
   VEC *x; /* state (location and velocity) */
   VEC *z; /* measurement (location) */
   MAT *A; /* system matrix */
   MAT *B; /* control matrix */
   MAT *P; /* error covariance */
   VEC *u; /* control (acceleration) */
   MAT *H; /* observer matrix */
   MAT *K; /* kalman gain */

   /*  vectors and matrices for calculations: */
   VEC *t0;
   VEC *t1;
   MAT *T0;
   MAT *T1;
};


static void kalman_init(kalman_t *kf, float dt, float q, float r, float pos, float speed)
{
   kf->t0 = v_get(2);
   kf->t1 = v_get(2);
   kf->T0 = m_get(2, 2);
   kf->T1 = m_get(2, 2);
   
   kf->I = m_get(2, 2);
   m_ident(kf->I);

   /* set initial state: */
   kf->x = v_get(2);
   v_set_val(kf->x, 0, pos);
   v_set_val(kf->x, 1, speed);

   /* no measurement or control yet: */
   kf->z = v_get(2);
   kf->u = v_get(1);

   kf->P = m_get(2, 2);
   m_ident(kf->P);
   
   /* set up noise: */
   kf->Q = m_get(2, 2);
   sm_mlt(q, kf->I, kf->Q);
   kf->R = m_get(2, 2);
   sm_mlt(r, kf->I, kf->R);
   
   kf->K = m_get(2, 1);
   kf->H = m_get(2, 2);
   m_set_val(kf->H, 0, 0, 1.0);
   //m_ident(kf->H);
   
   /* A = | 1.0   dt  |
          | 0.0   1.0 | */
   kf->A = m_get(2, 2);
   m_set_val(kf->A, 0, 0, 1.0);
   m_set_val(kf->A, 0, 1, dt);
   m_set_val(kf->A, 1, 0, 0.0);
   m_set_val(kf->A, 1, 1, 1.0);

   /* B = | 0.5 * dt ^ 2 |
          |     dt       | */
   kf->B = m_get(2, 1);
   m_set_val(kf->B, 0, 0, 0.5 * dt * dt);
   m_set_val(kf->B, 1, 0, dt);
}


static void kalman_predict(kalman_t *kf, float a)
{
   /* x = A * x + B * u */
   v_set_val(kf->u, 0, a);
   mv_mlt(kf->A, kf->x, kf->t0);
   mv_mlt(kf->B, kf->u, kf->t1);
   v_add(kf->t0, kf->t1, kf->x);

   /* P = A * P * AT + Q */
   m_mlt(kf->A, kf->P, kf->T0);
   mmtr_mlt(kf->T0, kf->A, kf->T1);
   m_add(kf->T1, kf->Q, kf->P);
}



static void kalman_correct(kalman_t *kf, float p, float v)
{
   /* K = P * HT * inv(H * P * HT + R) */
   m_mlt(kf->H, kf->P, kf->T0);
   mmtr_mlt(kf->T0, kf->H, kf->T1);
   m_add(kf->T1, kf->R, kf->T0);
   m_inverse(kf->T0, kf->T1);
   mmtr_mlt(kf->P, kf->H, kf->T0);
   m_mlt(kf->T0, kf->T1, kf->K);

   /* x = x + K * (z - H * x) */
   mv_mlt(kf->H, kf->x, kf->t0);
   v_set_val(kf->z, 0, p);
   v_set_val(kf->z, 1, v);
   v_sub(kf->z, kf->t0, kf->t1);
   mv_mlt(kf->K, kf->t1, kf->t0);
   v_add(kf->x, kf->t0, kf->t1);
   v_copy(kf->t1, kf->x);
   
   /* P = (I - K * H) * P */
   m_mlt(kf->K, kf->H, kf->T0);
   m_sub(kf->I, kf->T0, kf->T1);
   m_mlt(kf->T1, kf->P, kf->T0);
   m_copy(kf->T0, kf->P);
}


/*
 * executes kalman predict and correct step
 */
void kalman_run(kalman_out_t *out, kalman_t *kalman, const kalman_in_t *in)
{
   kalman_predict(kalman, in->acc);
   kalman_correct(kalman, in->pos, 0.0);
   out->pos = v_entry(kalman->x, 0);
   out->speed = v_entry(kalman->x, 1);
}


/*
 * allocates and initializes a kalman filter
 */
kalman_t *kalman_create(const kalman_config_t *config, const kalman_out_t *init_state)
{
   kalman_t *kalman = (kalman_t *)malloc(sizeof(kalman_t));
   kalman_init(kalman, config->dt, config->process_var, config->measurement_var, init_state->pos, init_state->speed);
   return kalman;
}
