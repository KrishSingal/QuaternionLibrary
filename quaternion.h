#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h> 

typedef struct quaternion {
    double q0;
    double* q;
} Quaternion;

#ifndef M_PI
    #define M_PI (3.14159265358979323846)
#endif

/* Function Declarations */

void print_quaternion(char *str, Quaternion *q);

Quaternion* new_quaternion();

Quaternion* conjugate(Quaternion* q);

double norm(Quaternion* q);

Quaternion* normalize(Quaternion* q);

double dot_product(double *p, double *q);

double* scale_vector(double k, double *q);

double* cross_product(double *p, double *q);

double* vector_add(double *p, double *q);

Quaternion* quaternion_add(Quaternion* a, Quaternion* b);

Quaternion* quaternion_product(Quaternion *p, Quaternion *q);

Quaternion* quaternion_from_rotation_angle(double angle, double* axis_of_rotation);




