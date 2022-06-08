#include "quaternion.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h> 

const int DEBUG = 0;

double to_radians(double degrees) {
    return degrees * (M_PI / 180.0);
}

void print_quaternion(char *str, Quaternion *q){
    printf("%s: %f + %f i + %f j + %f k\n", str, q->q0, q-> q[0], q->q[1], q->q[2]);
}

Quaternion* new_quaternion(){
    Quaternion* new = malloc(sizeof(Quaternion));
    new -> q = malloc(3*sizeof(double));

    return new;
}

Quaternion* conjugate(Quaternion* q){
    Quaternion* conj = new_quaternion();
    conj -> q0 = q -> q0;
    conj -> q[0] = - q->q[0];
    conj -> q[1] = - q->q[1];
    conj -> q[2] = - q->q[2];

    return conj;
}

double norm(Quaternion* q){
    return sqrt(quaternion_product(conjugate(q), q) -> q0);
}

Quaternion* normalize(Quaternion* q){
    Quaternion* normalized = new_quaternion();
    double n = norm(q);

    normalized->q0 = q->q0 / n;
    normalized->q[0] = q->q[0] / n;
    normalized->q[1] = q->q[1] / n;
    normalized->q[2] = q->q[2] / n;

    return normalized;
}

double dot_product(double *p, double *q){
    double dp = 0;
    for(int i=0; i<3 ; i++){
        dp += p[i] * q[i];
    }

    return dp;
}

double* scale_vector(double k, double *q){
    double* scaled_q = malloc(sizeof(double)*3);

    for(int i=0; i< 3; i++){
        scaled_q[i] = k*q[i];
    }

    return scaled_q;
}

double* cross_product(double *p, double *q){
    double* cp = malloc(sizeof(double)*3);
    cp[0] = p[1]*q[2] - p[2]*q[1];
    cp[1] = p[2]*q[0] - p[0]*q[2];
    cp[2] = p[0]*q[1] - q[0]*p[1];

    return cp;
}

double* vector_add(double *p, double *q){
    double* sum = malloc(sizeof(double)*3);

    for(int i=0; i<3; i++){
        sum[i] = p[i] + q[i];
    }

    return sum;
}

double vector_norm(double* vector){
    return sqrt(pow(vector[0], 2) + pow(vector[1], 2) + pow(vector[2], 2));
}

Quaternion* quaternion_add(Quaternion* a, Quaternion* b) {
    if(DEBUG){
        printf("here\n");
    }
    Quaternion* result = new_quaternion();

    result -> q0 = a -> q0 + b-> q0;
    for(int i = 0; i<3; i++){
        result -> q[i] = a->q[i] + b->q[i];
    }
    
    return result;
}

Quaternion* quaternion_product(Quaternion *p, Quaternion *q){
    Quaternion* prod = new_quaternion();

    prod -> q0 = p->q0 * q-> q0 - dot_product(p->q, q->q);
    if(DEBUG){
        printf("computed q0 as %f",  prod->q0);
    }
    double *vec = vector_add(vector_add(scale_vector(p->q0, q->q), scale_vector(q->q0, p->q)), cross_product(p->q, q->q));

    for(int i =0; i< 3; i++){
        prod -> q[0] = vec[0];
    }
    
    return prod;
}

Quaternion* quaternion_from_rotation_angle(double theta, double* axis_of_rotation){
    Quaternion* from_angle = new_quaternion();
    theta = to_radians(theta);

    from_angle -> q0 = cos(theta/2);
    from_angle -> q[0] = sin(theta/2) * axis_of_rotation[0];
    from_angle -> q[1] = sin(theta/2) * axis_of_rotation[1];
    from_angle -> q[2] = sin(theta/2) * axis_of_rotation[2];
    
    return from_angle;
}

double* rotate(Quaternion* q, double* v){
    double* term1 = scale_vector(pow(q->q0, 2) - pow(vector_norm(q->q), 2), v);
    double* term2 = scale_vector(2*dot_product(q->q, v), q->q);
    double* term3 = scale_vector(2*q->q0, cross_product(q->q, v));

    return vector_add(term1, vector_add(term2, term3));
}

int main(){
    double axis_a[3] = {0,0,1};
    Quaternion* a = quaternion_from_rotation_angle(90, axis_a);
    double axis_b[3] = {1,0,0};
    Quaternion* b = quaternion_from_rotation_angle(45, axis_b);

    print_quaternion("a", a);
    print_quaternion("b", b);

    Quaternion* add = quaternion_add(a, b);
    print_quaternion("Sum", add);

    Quaternion* prod = quaternion_product(a, b);
    print_quaternion("Product", prod);

    Quaternion* normed = normalize(a);
    print_quaternion("Normalization", normed);

    double vec [3] = {1,0,0};
    double* rotated = rotate(a, vec);
    printf("Rotated: %f i + %f j + %f k\n", rotated[0], rotated[1], rotated[2]);
}



