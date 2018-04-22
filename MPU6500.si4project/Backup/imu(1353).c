#include "imu.h"
#include "mpu6500.h"

//struct _Matrix imu_state;  
//struct _Matrix imu_measure;

float imu_state[6]={0};
float ium_measure[6]={0};
_Matrix imu_p;  
_Matrix imu_r;
_Matrix imu_q;

_Matrix v2R(_Matrix a);
_Matrix R2v(_Matrix a);


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

_Matrix v2R(_Matrix a)
{
	_Matrix b;
	return b;
}

_Matrix R2v(_Matrix a)
{
	_Matrix b;
	return b;
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
	_Matrix Cr,w,w_hat,w1,R,R_,a_hat,Ge,H,y,H_,S,K,KH,KH_,K_,temp1,temp2,temp3,eye;
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
	
	matrix_set(&w,1,3);
	matrix_write(&w,0,0,*state);
	matrix_write(&w,0,1,*(state+1));
	matrix_write(&w,0,2,*(state+2));
	
	matrix_set(&w_hat,1,3);
	matrix_write(&w_hat,0,0,*(state+3));
	matrix_write(&w_hat,0,1,*(state+4));
	matrix_write(&w_hat,0,2,*(state+5));
	
	matrix_set(&w1,1,3);
	matrix_write(&w1,0,0,(*(state+3))*time);
	matrix_write(&w1,0,1,(*(state+4))*time);
	matrix_write(&w1,0,2,(*(state+5))*time);
	
	matrix_set(&R,3,3);
	
	matrix_set(&R_,3,3);
	
	R=v2R(w1);
	
	matrix_transpos(&R,&R_);
	
	matrix_set(&Ge,3,1);
	matrix_write(&Ge,0,0,0);
	matrix_write(&Ge,0,1,0);
	matrix_write(&Ge,0,2,9.81);
	
	matrix_set(&a_hat,1,3);
	
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
	
	matrix_set(&y,2,3);
	matrix_write(&y,0,0,ium_measure[0]-matrix_read(&a_hat,0,0));
	matrix_write(&y,0,1,ium_measure[1]-matrix_read(&a_hat,0,1));
	matrix_write(&y,0,2,ium_measure[2]-matrix_read(&a_hat,0,2));
	matrix_write(&y,1,0,ium_measure[3]-matrix_read(&w_hat,0,0));
	matrix_write(&y,1,1,ium_measure[4]-matrix_read(&w_hat,0,1));
	matrix_write(&y,1,2,ium_measure[5]-matrix_read(&w_hat,0,0));
	
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
	
}




