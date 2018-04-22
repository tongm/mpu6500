#ifndef _MATRIX_H    
#define _MATRIX_H  
  
//ͷ�ļ�    
#include <stdio.h>    
#include <stdlib.h>    
    
//�������ݽṹ    
//��ά����    
typedef struct  
{     
    int m;    
    int n;    
    float *arr;    
} _Matrix ;   
  
//���󷽷�  
//����m    
void matrix_set_m(_Matrix *m,int mm);    
//����n    
void matrix_set_n(_Matrix *m,int nn);    
//��ʼ��    
void matrix_init(_Matrix *m);    

void matrix_set(_Matrix *m,int mm,int nn);
//�ͷ�    
void matrix_free(_Matrix *m);    
//��ȡi,j���������    
//ʧ�ܷ���-31415,�ɹ�����ֵ    
float matrix_read(_Matrix *m,int i,int j);    
//д��i,j���������    
//ʧ�ܷ���-1,�ɹ�����1    
int matrix_write(_Matrix *m,int i,int j,float val);   
  
//��������  
//�ɹ�����1,ʧ�ܷ���-1    
int matrix_add(_Matrix *A,_Matrix *B,_Matrix *C);    
//C = A - B    
//�ɹ�����1,ʧ�ܷ���-1    
int matrix_subtract(_Matrix *A,_Matrix *B,_Matrix *C);    
//C = A * B    
//�ɹ�����1,ʧ�ܷ���-1    
int matrix_multiply(_Matrix *A,_Matrix *B,_Matrix *C);    
//����ʽ��ֵ,ֻ�ܼ���2 * 2,3 * 3    
//ʧ�ܷ���-31415,�ɹ�����ֵ    
float matrix_det(_Matrix *A);    
//��ת�þ���,B = AT    
//�ɹ�����1,ʧ�ܷ���-1    
int matrix_transpos(_Matrix *A,_Matrix *B);    
//�������,B = A^(-1)    
//�ɹ�����1,ʧ�ܷ���-1    
int matrix_inverse(_Matrix *A,_Matrix *B);    
//�������Գ���
void matrix_multiply_const(_Matrix *A , float multiple);

#endif

