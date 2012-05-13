
/*
 * file: kalman.c
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


#include <opencv/cv.h>

#include "kalman.h"


struct kalman
{
   CvKalman *filter; /* kalman filter state/context */
   CvMat *uk; /* control vector */
   CvMat *zk; /* measurement vector */
};



static void f2cv(CvMat *d, float *s, int len)
{
   memcpy(d->data.fl, s, sizeof(float) * len);
}


static void cv2f(float *d, CvMat *s, int len)
{
   memcpy(d, s->data.fl, sizeof(float) * len);
}


void kalman_run(kalman_out_t *out, kalman_t *kalman, const kalman_in_t *in)
{
#define TRANSITION_MATRIX_SIZE 4
#define CONTROL_MATRIX_SIZE 2
#define MEASUREMENT_INPUT_SIZE 1
   float measurement_input[MEASUREMENT_INPUT_SIZE] = {in->pos};

#define CONTROL_INPUT_SIZE 1
   float control_input[CONTROL_INPUT_SIZE] = {in->acc};

   /* convert to OpenCV's kalman representation */
   f2cv(kalman->zk, measurement_input, MEASUREMENT_INPUT_SIZE);
   f2cv(kalman->uk, control_input, CONTROL_INPUT_SIZE);

   /* predict via state model */
   cvKalmanPredict(kalman->filter, kalman->uk);

#define OUTPUT_SIZE 2
   float kalman_out[OUTPUT_SIZE];

   /* correct using measurement */
   cvKalmanCorrect(kalman->filter, kalman->zk);
   cv2f(kalman_out, kalman->filter->state_post, OUTPUT_SIZE);


   /* convert back from OpenCV's kalman representation */
   out->pos = kalman_out[0];
   out->speed = kalman_out[1];
}


kalman_t *kalman_create(const kalman_config_t *config, const kalman_out_t *init_state)
{
   kalman_t *kalman = (kalman_t *)malloc(sizeof(kalman_t));

   kalman->filter = cvCreateKalman(2, 1, 1);
   kalman->uk = cvCreateMat(1, 1, CV_32FC1);
   kalman->zk = cvCreateMat(1, 1, CV_32FC1);

   cvSetIdentity(kalman->filter->measurement_matrix, cvRealScalar(1));
   cvSetIdentity(kalman->filter->process_noise_cov, cvRealScalar(config->process_var));
   cvSetIdentity(kalman->filter->measurement_noise_cov, cvRealScalar(config->measurement_var));
   cvSetIdentity(kalman->filter->error_cov_post, cvRealScalar(1));

   float state[2] =
   {
      init_state->pos,
      init_state->speed
   };
   
   float transition_matrix[TRANSITION_MATRIX_SIZE] =
   {
      1.0, config->dt,
      0, 1.0
   };
   
   float control_matrix[CONTROL_MATRIX_SIZE] =
   {
      0.5 * config->dt * config->dt,
      config->dt
   };

   f2cv(kalman->filter->transition_matrix, transition_matrix, TRANSITION_MATRIX_SIZE);
   f2cv(kalman->filter->control_matrix, control_matrix, CONTROL_MATRIX_SIZE);

   memcpy(kalman->filter->state_post->data.fl, state, sizeof(state));
   return kalman;
}

