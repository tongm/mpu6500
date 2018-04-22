#include "imu.h"
#include "mpu6500.h"
#include <math.h>

//struct _Matrix imu_state;  
//struct _Matrix imu_measure;

float imu_state[6]={0};
float ium_measure[6]={0};
_Matrix imu_p;  
_Matrix imu_r;
_Matrix imu_q;



void init_matrix(void)
{
//	//the value get form mpu6500
//	matrix_set(&imu_measure,6,1);
//	//imu state
//	matrix_set(&imu_state,6,1);  
	/**state error covariance**/
	matrix_set(&imu_p,6,6);
	/**ium measurement noise**/
	matrix_set(&imu_r,6,6);
	/**system noise**/
	matrix_set(&imu_q,3,3);
}

void v2R(_Matrix * a ,_Matrix *b)
{
	double n,s,c;
	u8 i,j;
	float tem;
	_Matrix G,nv,temp,temp1,eye;
	n=matrix_det(a);
	s=sin(n);
	c=cos(n);
	
	matrix_set(&nv,1,3);
	matrix_set(&G,3,3);
	matrix_set(&temp,3,3);
	matrix_set(&temp1,3,3);
	matrix_set(&eye,3,3);
	
	matrix_write(&nv,0,0,matrix_read(a,0,0)/n);
	matrix_write(&nv,0,1,matrix_read(a,0,1)/n);
	matrix_write(&nv,0,2,matrix_read(a,0,2)/n);
	
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j=0)
		{
			matrix_write(&eye,i,j,0);
			matrix_write(&temp,i,j,0);
			matrix_write(&temp1,i,j,0);
		}
	}
	
	matrix_write(&eye,0,0,1);
	matrix_write(&eye,1,1,1);
	matrix_write(&eye,2,2,1);
	
	matrix_write(&G,0,0,0);
	matrix_write(&G,0,1,-matrix_read(&nv,0,2));
	matrix_write(&G,0,2,matrix_read(&nv,0,1));
	
	matrix_write(&G,1,0,matrix_read(&nv,0,2));
	matrix_write(&G,1,1,0);
	matrix_write(&G,1,2,-matrix_read(&nv,0,0));
	
	matrix_write(&G,1,0,-matrix_read(&nv,0,1));
	matrix_write(&G,1,1,matrix_read(&nv,0,0));
	matrix_write(&G,1,2,0);
	
	matrix_multiply(&temp,&G,&G);
	
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j=0)
		{
			tem=matrix_read(&temp,i,j);
			matrix_write(&temp,i,j,tem*(1-c));
		}
	}
	
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j=0)
		{
			tem=matrix_read(&G,i,j);
			matrix_write(&G,i,j,tem*s);
		}
	}
	matrix_add(&eye,&G,&temp1);
	matrix_add(&temp1,&temp,b);
}

void R2v(_Matrix *a, _Matrix *b)
{
	u8 i,j;
	float tr,s,qw,qx,qy,qz,temp[3][3],n;
	double angle;
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j=0)
		{
			temp[i][j]=matrix_read(a,i,j);
		}
	}
	tr=temp[0][0]+temp[1][1]+temp[2][2];
	if(tr>0)
	{
		s=sqrt(tr+1)*2;
		qw=0.25*s;
		qx=(temp[2][1]-temp[1][2])/s;
		qy=(temp[0][2]-temp[2][0])/s;
		qz=(temp[1][0]-temp[0][1])/s;
	}
	else if(temp[0][0]>temp[1][1]&&temp[0][0]>temp[2][2])
	{
		s=sqrt(1+temp[0][0]-temp[1][1]-temp[2][2])*2;
		qw=(temp[2][1]-temp[1][2])/s;
		qx=0.25*s;
		qy=(temp[0][1]+temp[1][0])/s;
		qz=(temp[0][2]+temp[2][0])/s;
	}
	else if(temp[1][1]>temp[2][2])
	{
		s=sqrt(1+temp[1][1]-temp[0][0]-temp[2][2])*2;
		qw=(temp[0][2]-temp[2][0])/s;
		qx=(temp[0][1]-temp[1][0])/s;
		qy=0.25*s;
		qz=(temp[1][2]+temp[2][1])/s;
	}
	else
	{
		s=sqrt(1+temp[2][2]-temp[0][0]-temp[1][1])*2;
		qw=(temp[1][0]-temp[0][1])/s;
		qx=(temp[0][2]-temp[2][0])/s;
		qy=(temp[1][2]-temp[2][1])/s;
		qz=0.25*s;
	}
	
	n=sqrt(1-qw*qw);
	
	angle=2*acos(qw);
	
	matrix_write(b,0,0,qx*angle);
	matrix_write(b,1,0,qy*angle);
	matrix_write(b,2,0,qz*angle);
}

void predict_process(float * state, float time )
{
	_Matrix w0,w1,w,Go,Gw,dw,df,eye,temp1,temp2,temp3,Qk,Q;
	u8 i,j;
	matrix_set(&w0,1,3);
	
	matrix_set(&w1,1,3);
	
	matrix_set(&w,1,3);
	
	matrix_set(&Go,3,3);
	
	matrix_set(&Gw,3,3);
	
	matrix_set(&dw,3,3);
	
	matrix_set(&df,6,6);
	
	matrix_write(&w0,0,0,*state);
	matrix_write(&w0,0,1,*(state+1));
	matrix_write(&w0,0,2,*(state+2));
	
	matrix_write(&w,0,0,*(state));
	matrix_write(&w,0,1,*(state+1));
	matrix_write(&w,0,2,*(state+2));
	
	matrix_write(&Gw,0,0,0);
	matrix_write(&Gw,0,1,*(state+5)/2);
	matrix_write(&Gw,0,2,-(*(state+4))/2);
	matrix_write(&Gw,1,0,-(*(state+5))/2);
	matrix_write(&Gw,1,1,0);
	matrix_write(&Gw,1,2,*(state+3)/2);
	matrix_write(&Gw,2,0,*(state+4)/2);
	matrix_write(&Gw,2,1,-(*(state+3))/2);
	matrix_write(&Gw,2,2,0);
	
	matrix_write(&Go,0,0,0);
	matrix_write(&Go,0,1,*(state+2)/2);
	matrix_write(&Go,0,2,-(*(state+1))/2);
	matrix_write(&Go,1,0,-(*(state+2))/2);
	matrix_write(&Go,1,1,0);
	matrix_write(&Go,1,2,*(state+0)/2);
	matrix_write(&Go,2,0,*(state+1)/2);
	matrix_write(&Go,2,1,-(*(state+0))/2);
	matrix_write(&Go,2,2,0);
	
	matrix_set(&eye,3,3);
	
	matrix_write(&eye,0,0,1);
	matrix_write(&eye,0,1,0);
	matrix_write(&eye,0,2,0);
	matrix_write(&eye,1,0,0);
	matrix_write(&eye,1,1,1);
	matrix_write(&eye,1,2,0);
	matrix_write(&eye,2,0,0);
	matrix_write(&eye,2,1,0);
	matrix_write(&eye,2,2,1);
	
	matrix_add(&eye,&Gw,&dw);
	
	matrix_write(&df,0,0,1+matrix_read(&Go,0,0)*time);
	matrix_write(&df,0,1,matrix_read(&Go,0,1)*time);
	matrix_write(&df,0,2,matrix_read(&Go,0,2)*time);
	matrix_write(&df,0,3,matrix_read(&dw,0,0)*time);
	matrix_write(&df,0,4,matrix_read(&dw,0,1)*time);
	matrix_write(&df,0,5,matrix_read(&dw,0,2)*time);
	
	matrix_write(&df,1,0,matrix_read(&Go,1,0)*time);
	matrix_write(&df,1,1,1+matrix_read(&Go,1,1)*time);
	matrix_write(&df,1,2,matrix_read(&Go,1,2)*time);
	matrix_write(&df,1,3,matrix_read(&dw,1,0)*time);
	matrix_write(&df,1,4,matrix_read(&dw,1,1)*time);
	matrix_write(&df,1,5,matrix_read(&dw,1,2)*time);
	
	matrix_write(&df,2,0,matrix_read(&Go,2,0)*time);
	matrix_write(&df,2,1,matrix_read(&Go,2,1)*time);
	matrix_write(&df,2,2,1+matrix_read(&Go,2,2)*time);
	matrix_write(&df,2,3,matrix_read(&dw,2,0)*time);
	matrix_write(&df,2,4,matrix_read(&dw,2,1)*time);
	matrix_write(&df,2,5,matrix_read(&dw,2,2)*time);
	
	for(i=3;i<6;i++)
	{
		for(j=0;j<6;j=0)
		{
			matrix_write(&Qk,i,j,0);
		}
	}
	matrix_write(&df,3,3,1);
	matrix_write(&df,4,4,1);
	matrix_write(&df,5,5,1);
	
	matrix_free(&w0);
	matrix_free(&w);
	matrix_free(&Go);
	matrix_free(&Gw);
	matrix_free(&dw);
	matrix_free(&eye);
	
	matrix_set(&Qk,6,6);
	for(i=0;i<6;i++)
	{
		for(j=0;j<6;j=0)
		{
			matrix_write(&Qk,i,j,0);
		}
	}
	matrix_write(&Qk,3,3,0.000001);
	matrix_write(&Qk,4,4,0.000001);
	matrix_write(&Qk,5,5,0.000001);

	matrix_set(&temp1,6,6); 
	matrix_set(&temp2,6,6); 
	matrix_set(&temp3,6,6);
	matrix_set(&Q,6,6);
	
	matrix_multiply(&df,&Qk,&temp1);
	matrix_transpos(&df,&temp2);
	matrix_multiply(&df,&temp2,&temp3);
	
	matrix_multiply_const(&temp1,time/2);
	matrix_multiply_const(&temp3,time/2);
	
	matrix_add(&temp1,&temp3,&Q);
	
	matrix_multiply(&df,&imu_p,&temp1);
	matrix_multiply(&temp1,&temp2,&temp3);
	
	matrix_add(&temp3,&Q,&imu_p);
	
	matrix_free(&df);
	matrix_free(&temp1);
	matrix_free(&temp1);
	matrix_free(&temp1);
	matrix_free(&Q);
	matrix_free(&Qk);
	
	*state=matrix_read(&w1,0,0);
	*(state+1)=matrix_read(&w1,0,1);
	*(state+2)=matrix_read(&w1,0,2);
	
	matrix_free(&w1);
	
}

void update_process(float * state, float time)
{
	_Matrix Cr,w,w_hat,w1,R,R_,a_hat,Ge,H,y,H_,S,K,KH,KH_,
		K_,temp1,temp2,temp3,eye,wplus;
	u8 i,j;
	matrix_set(&Cr,6,6);
	for(i=0;i<6;i++)
	{
		for(j=0;j<6;j=0)
		{
			matrix_write(&Cr,i,j,0);
		}
	}
	matrix_write(&Cr,0,0,0.8*0.8);
	matrix_write(&Cr,1,1,0.8*0.8);
	matrix_write(&Cr,2,2,0.8*0.8);
	matrix_write(&Cr,3,3,0.0039*0.0039*pi*pi/32400);
	matrix_write(&Cr,4,4,0.0039*0.0039*pi*pi/32400);
	matrix_write(&Cr,5,5,0.0039*0.0039*pi*pi/32400);
	
	matrix_set(&w,3,1);
	matrix_write(&w,0,0,*state);
	matrix_write(&w,1,0,*(state+1));
	matrix_write(&w,2,0,*(state+2));
	
	matrix_set(&w_hat,3,1);
	matrix_write(&w_hat,0,0,*(state+3));
	matrix_write(&w_hat,1,0,*(state+4));
	matrix_write(&w_hat,2,0,*(state+5));
	
	matrix_set(&w1,3,1);
	matrix_write(&w1,0,0,(*(state+3))*time);
	matrix_write(&w1,1,0,(*(state+4))*time);
	matrix_write(&w1,2,0,(*(state+5))*time);
	
	matrix_set(&R,3,3);
	
	matrix_set(&R_,3,3);
	
//	R=v2R(w1);
	
	matrix_transpos(&R,&R_);
	
	matrix_set(&Ge,3,1);
	matrix_write(&Ge,0,0,0);
	matrix_write(&Ge,1,0,0);
	matrix_write(&Ge,2,0,9.81);
	
	matrix_set(&a_hat,3,1);
	
	matrix_multiply(&R_,&Ge,&a_hat);
	
	matrix_set(&H,6,6);
	for(i=0;i<6;i++)
	{
		for(j=0;j<6;j=0)
		{
			matrix_write(&H,i,j,0);
		}
	}
	matrix_write(&H,0,1,-matrix_read(&a_hat,0,2));
	matrix_write(&H,0,2,matrix_read(&a_hat,0,1));
	matrix_write(&H,1,0,matrix_read(&a_hat,0,2));
	matrix_write(&H,1,2,-matrix_read(&a_hat,0,0));
	matrix_write(&H,2,0,-matrix_read(&a_hat,0,1));
	matrix_write(&H,2,1,matrix_read(&a_hat,0,0));
	matrix_write(&H,3,3,1);
	matrix_write(&H,4,4,1);
	matrix_write(&H,5,5,1);
	
	matrix_set(&y,6,1);
	matrix_write(&y,0,0,ium_measure[0]-matrix_read(&a_hat,0,0));
	matrix_write(&y,1,0,ium_measure[1]-matrix_read(&a_hat,0,1));
	matrix_write(&y,2,0,ium_measure[2]-matrix_read(&a_hat,0,2));
	matrix_write(&y,3,0,ium_measure[3]-matrix_read(&w_hat,0,0));
	matrix_write(&y,4,0,ium_measure[4]-matrix_read(&w_hat,0,1));
	matrix_write(&y,5,0,ium_measure[5]-matrix_read(&w_hat,0,0));
	
	matrix_set(&H_,6,6);
	matrix_transpos(&H,&H_);
	
	matrix_set(&S,6,6);
	matrix_set(&temp1,6,6);
	matrix_set(&temp2,6,6);
	matrix_set(&temp3,6,6);
	
	matrix_multiply(&H,&imu_p,&temp1);
	matrix_multiply(&temp1,&H_,&temp2);
	
	matrix_add(&temp2,&Cr,&S);
	
	matrix_multiply(&imu_p,&H_,&temp1);
	matrix_inverse(&S,&temp2);
	
	matrix_set(&K,6,6);
	matrix_multiply(&temp1,&temp2,&K);
	matrix_multiply(&K,&H,&temp1);
	matrix_set(&eye,6,6);
	for(i=0;i<6;i++)
	{
		for(j=0;j<6;j=0)
		{
			matrix_write(&eye,i,j,0);
		}
	}
	matrix_write(&eye,0,0,1);
	matrix_write(&eye,1,1,1);
	matrix_write(&eye,2,2,1);
	matrix_write(&eye,3,3,1);
	matrix_write(&eye,4,4,1);
	matrix_write(&eye,5,5,1);
	matrix_set(&KH,6,6);
	matrix_set(&KH_,6,6);
	matrix_set(&K_,6,6);
	matrix_transpos(&K,&K_);
	matrix_transpos(&KH,&KH_);
	matrix_subtract(&eye,&temp1,&KH);
	matrix_multiply(&K,&Cr,&temp1);
	matrix_multiply(&temp1,&K_,&temp3);
	matrix_multiply(&KH,&imu_p,&temp1);
	matrix_multiply(&temp1,&KH_,&temp2);
	matrix_add(&temp2,&temp3,&imu_p);

	matrix_set(&wplus,6,6);
	matrix_multiply(&K,&y,&wplus);
	
	*(state+3)+=matrix_read(&wplus,3,0);
	*(state+4)+=matrix_read(&wplus,4,0);
	*(state+5)+=matrix_read(&wplus,5,0);
	
	matrix_free(&Cr);
	matrix_free(&w);
	matrix_free(&w_hat);
	matrix_free(&w1);
	matrix_free(&R);
	matrix_free(&R_);
	
	matrix_free(&a_hat);
	matrix_free(&Ge);
	matrix_free(&H);
	matrix_free(&y);
	matrix_free(&H_);
	matrix_free(&S);
	
	matrix_free(&K);
	matrix_free(&KH);
	matrix_free(&KH_);
	matrix_free(&K_);
	matrix_free(&temp1);
	matrix_free(&temp2);
	
	matrix_free(&temp3);
	matrix_free(&eye);
	matrix_free(&wplus);
}




