#include "arithmetic.h"
#include "PeripheralHeaderIncludes.h"
#include "math.h"

/*#define Q_w 1
#define R_w 0.001
#define Id_w 20
#define We_w 1
#define Id2_w 1
#define We2_w 1*/

float Rs = 2.35;    //2.35
float Ld = 0.0065;  //0.0065
float Lq = 0.0065;  //0.0065
float J = 0.00033; //0.00035 and 3e-5 and 0.00033
float Fai = 0.07876;

float Rsm = 2.35;    //2.35
float Ldm = 0.0065;  //0.0065
float Lqm = 0.0065;  //0.0065
float Jm = 0.00033; //0.00035 and 3e-5 and 0.00033
float Faim = 0.07876;

#define Bz  0.0
#define p   4.0

#define Tp  3
#define Tc  1

Matrix aa, bb, cc;
Matrix eye;
Matrix a, b, c;
Matrix A, B, C;
Matrix ca, cb;
Matrix u, X, Yr;
Matrix Omega, Gamma;
Matrix CA, CAA, CAB;
Matrix Q, R, H, f;
Matrix temp1, temp2, temp3, temp4;
Matrix temp5, temp6, temp7, temp8;
Matrix u_ol, delta_u;

int cnt = 0;

#define Temax 1.5   //1.27

float iqmax = Temax*2.11613340104;  //Temax*2/3/p/Fai  0.5*2.1161 = 1.0587
float iqmin = -Temax*2.11613340104; //-Temax*2/3/p/Fai

#if (BUILDLEVEL == LEVEL7)

#define COUNT 1

float Q_w = 0;
float R_w = 0;

float Id_w = 1;
float Id2_w = 1;
float Id3_w = 10;

float We_w = 1;
float We2_w = 1;
float We3_w = 1;

float vd_w = 0.1;   // 0.1
float vq_w = 0.1;   // 50

float qmax = 0.0;
float qmin = 0.0;

float qlim = udc*0.84/1.732;    //udc*0.84/1.732
float qlim2 = udc*0.84/1.732;
float dlim = udc*0.42/1.732;    //udc*0.42/1.732

float vd_old = 0.0;
float vq_old = 0.0;
float id_old = 0.0;
float iq_old = 0.0;
float we_old = 0.0;

int nx = 3;
int nu = 2;
int ny = 2;
int Nx, Nu, Ny;
int Ar, Ac, Br, Bc, Cr, Cc;

#endif

#if (BUILDLEVEL == LEVEL8)

#define COUNT 2

float Q_w = 0;
float R_w = 0;

float Id_w = 1;
float Id2_w = 1;
float Id3_w = 1;

float Iq_w = 1;
float Iq2_w = 1;
float Iq3_w = 1;

float vd_w = 0.004;
float vq_w = 0.004;

float qlim = udc*0.8/1.732;    //udc*0.84/1.732
float qlim2 = 60.0;   //udc*0.8/sqrt(3)
float dlim = 30.0;    //udc*0.42/1.732

float vd_old = 0.0;
float vq_old = 0.0;
float id_old = 0.0;
float iq_old = 0.0;

float usv = 0.0;

float id1 = 0.0;
float iq1 = 0.0;

float alpha = 0.0;
float beta = 0.0;

int nx = 2;
int nu = 2;
int ny = 2;
int Nx, Nu, Ny;
int Ar, Ac, Br, Bc, Cr, Cc;

#endif

#if (BUILDLEVEL == LEVEL7)
void PMSM_MPC_CCS(float id, float iq, float we, float idref, float weref, float *vd, float *vq, float Ts, float ParaMis)
{
    //Parameter Mismatch
//    Rs = ParaMis*Rsm;
//    Fai = ParaMis*Faim;
//    Ld = ParaMis*Ldm;

//    Lq = Ld;

	//int i, j, k, l, kr;
	/*--------------------------------------------------*/
	// Continuous Model
	/*** x = [id,iq,we] ***/
	/*** u = [ud,uq]    ***/
	/*** y = [id,we]    ***/
#ifndef opt
	set_matrix(aa,
		-Rs / Ld, Lq / Ld*we, Lq / Ld*iq,
		-Ld / Lq*we, -Rs / Lq, -Ld / Lq*id - Fai / Lq,
		0.0, 3.0 * p*p / (2.0 * J)* Fai, 0.0);
	set_matrix(bb,
		1.0 / Ld, 0.0,
		0.0, 1.0 / Lq,
		0.0, 0.0);
	set_matrix(cc,
		1.0, 0.0, 0.0,
		0.0, 0.0, 1.0);
#else
	aa.data[0][0] = -Rs / Ld;
	aa.data[0][1] = Lq / Ld*we;
	aa.data[0][2] = Lq / Ld*iq;
	aa.data[1][0] = -Ld / Lq*we;
	aa.data[1][1] = -Rs / Lq;
	aa.data[1][2] = -Ld / Lq*id - Fai / Lq;
	aa.data[2][0] = 0.0;
	aa.data[2][1] = 3.0 * p*p / (2.0 * J)*Fai;
	aa.data[2][2] = 0.0;
	bb.data[0][0] = 1.0 / Ld;
	bb.data[0][1] = 0.0;
	bb.data[1][0] = 0.0;
	bb.data[1][1] = 1.0 / Lq;
	bb.data[2][0] = 0.0;
	bb.data[2][1] = 0.0;
	cc.data[0][0] = 1.0;
	cc.data[0][1] = 0.0;
	cc.data[0][2] = 0.0;
	cc.data[1][0] = 0.0;
	cc.data[1][1] = 0.0;
	cc.data[1][2] = 1.0;
#endif
	//GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;
	/*--------------------------------------------------*/
	// Discrete Model
#ifndef opt
	set_identity_matrix(eye);
	scale_matrix(aa, Ts);
	add_matrix(aa, eye, a);

	scale_matrix(bb, Ts);
	copy_matrix(bb, b);

	copy_matrix(cc, c);
#else
	a.data[0][0] = aa.data[0][0] * Ts + 1;
	a.data[0][1] = aa.data[0][1] * Ts + 0;
	a.data[0][2] = aa.data[0][2] * Ts + 0;
	a.data[1][0] = aa.data[1][0] * Ts + 0;
	a.data[1][1] = aa.data[1][1] * Ts + 1;
	a.data[1][2] = aa.data[1][2] * Ts + 0;
	a.data[2][0] = aa.data[2][0] * Ts + 0;
	a.data[2][1] = aa.data[2][1] * Ts + 0;
	a.data[2][2] = aa.data[2][2] * Ts + 1;
	b.data[0][0] = bb.data[0][0] * Ts;
	b.data[0][1] = bb.data[0][1] * Ts;
	b.data[1][0] = bb.data[1][0] * Ts;
	b.data[1][1] = bb.data[1][1] * Ts;
	b.data[2][0] = bb.data[2][0] * Ts;
	b.data[2][1] = bb.data[2][1] * Ts;
	c.data[0][0] = cc.data[0][0];
	c.data[0][1] = cc.data[0][1];
	c.data[0][2] = cc.data[0][2];
	c.data[1][0] = cc.data[1][0];
	c.data[1][1] = cc.data[1][1];
	c.data[1][2] = cc.data[1][2];
#endif

	//GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;
	/*--------------------------------------------------*/
	// Extend the system    

	//A = [a zeros(nx,ny);c*a eye(ny)];
#ifndef opt
	for (i = 0; i<a.rows; i++)
	{
		for (j = 0; j<a.cols; j++)
		{
			A.data[i][j] = a.data[i][j];
		}
	}
	for (i = 0; i<nx; i++)
	{
		for (j = 0; j<ny; j++)
		{
			A.data[i][j + a.cols] = 0;
		}
	}
	multiply_matrix(c, a, ca);
	for (i = 0; i<c.rows; i++)
	{
		for (j = 0; j<a.cols; j++)
		{
			A.data[i + a.rows][j] = ca.data[i][j];
		}
	}
	for (i = 0; i<ny; i++)
	{
		for (j = 0; j<ny; j++)
		{
			if (i == j)
				A.data[i + a.rows][j + a.cols] = 1;
			else
				A.data[i + a.rows][j + a.cols] = 0;
		}
	}
#else
	A.data[0][0] = a.data[0][0];
	A.data[0][1] = a.data[0][1];
	A.data[0][2] = a.data[0][2];
	A.data[1][0] = a.data[1][0];
	A.data[1][1] = a.data[1][1];
	A.data[1][2] = a.data[1][2];
	A.data[2][0] = a.data[2][0];
	A.data[2][1] = a.data[2][1];
	A.data[2][2] = a.data[2][2];
	A.data[0][3] = 0;
	A.data[0][4] = 0;
	A.data[1][3] = 0;
	A.data[1][4] = 0;

	//multiply_matrix(c, a, ca);
	ca.data[0][0] = c.data[0][0] * a.data[0][0] + c.data[0][1] * a.data[1][0] + c.data[0][2] * a.data[2][0];
	ca.data[0][1] = c.data[0][0] * a.data[0][1] + c.data[0][1] * a.data[1][1] + c.data[0][2] * a.data[2][1];
	ca.data[0][2] = c.data[0][0] * a.data[0][2] + c.data[0][1] * a.data[1][2] + c.data[0][2] * a.data[2][2];
	ca.data[1][0] = c.data[1][0] * a.data[0][0] + c.data[1][1] * a.data[1][0] + c.data[1][2] * a.data[2][0];
	ca.data[1][1] = c.data[1][0] * a.data[0][1] + c.data[1][1] * a.data[1][1] + c.data[1][2] * a.data[2][1];
	ca.data[1][2] = c.data[1][0] * a.data[0][2] + c.data[1][1] * a.data[1][2] + c.data[1][2] * a.data[2][2];

	A.data[3][0] = ca.data[0][0];
	A.data[3][1] = ca.data[0][1];
	A.data[3][2] = ca.data[0][2];
	A.data[4][0] = ca.data[1][0];
	A.data[4][1] = ca.data[1][1];
	A.data[4][2] = ca.data[1][2];

	A.data[3][3] = 1;
	A.data[3][4] = 0;
	A.data[4][3] = 0;
	A.data[4][4] = 1;
#endif

	//B = [b;c*b];
#ifndef opt
	for (i = 0; i<b.rows; i++)
	{
		for (j = 0; j<b.cols; j++)
		{
			B.data[i][j] = b.data[i][j];
		}
	}
	multiply_matrix(c, b, cb);
	for (i = 0; i<c.rows; i++)
	{
		for (j = 0; j<b.cols; j++)
		{
			B.data[i + b.rows][j] = cb.data[i][j];
		}
	}

	//C = [zeros(ny,nx) eye(ny)];
	for (i = 0; i<ny; i++)
	{
		for (j = 0; j<nx; j++)
		{
			C.data[i][j] = 0;
		}
	}
	for (i = 0; i<c.rows; i++)
	{
		for (j = 0; j<b.cols; j++)
		{
			if (i == j)
				C.data[i][j + nx] = 1;
			else
				C.data[i][j + nx] = 0;
		}
	}
#else
	B.data[0][0] = b.data[0][0];
	B.data[0][1] = b.data[0][1];
	B.data[1][0] = b.data[1][0];
	B.data[1][1] = b.data[1][1];
	B.data[2][0] = b.data[2][0];
	B.data[2][1] = b.data[2][1];
	//multiply_matrix(c, b, cb);
	cb.data[0][0] = c.data[0][0] * b.data[0][0] + c.data[0][1] * b.data[1][0] + c.data[0][2] * b.data[2][0];
	cb.data[0][1] = c.data[0][0] * b.data[0][1] + c.data[0][1] * b.data[1][1] + c.data[0][2] * b.data[2][1];
	cb.data[1][0] = c.data[1][0] * b.data[0][0] + c.data[1][1] * b.data[1][0] + c.data[1][2] * b.data[2][0];
	cb.data[1][1] = c.data[1][0] * b.data[0][1] + c.data[1][1] * b.data[1][1] + c.data[1][2] * b.data[2][1];

	B.data[3][0] = cb.data[0][0];
    B.data[3][1] = cb.data[0][1];
    B.data[4][0] = cb.data[1][0];
    B.data[4][1] = cb.data[1][1];
    C.data[0][0] = 0;
    C.data[0][1] = 0;
    C.data[0][2] = 0;
    C.data[1][0] = 0;
    C.data[1][1] = 0;
    C.data[1][2] = 0;
    C.data[0][3] = 1;
    C.data[0][4] = 0;
    C.data[1][3] = 0;
    C.data[1][4] = 1;
#endif
	//GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;

	/*--- Get inout ---*/
	// x = [id,iq,we]';
	// u = [vd_old,vq_old]';
	// X = [delta_x;y];
	// yr = [idref,weref]';
	// Yr = repmat(yr,Tp,1);
#ifndef opt
	set_matrix(u,
		vd_old,
		vq_old);
	set_matrix(X,
		id - id_old,
		iq - iq_old,
		we - we_old,
		id,
		we);
#else
    u.data[0][0] = vd_old;
    u.data[1][0] = vq_old;
    X.data[0][0] = id - id_old;
    X.data[1][0] = iq - iq_old;
    X.data[2][0] = we - we_old;
    X.data[3][0] = id;
    X.data[4][0] = we;
#endif

#ifndef opt
	for (k = 0; k<Tp; k++)
	{
		for (i = 0; i<ny; i++)
		{
			for (j = 0; j<1; j++)
			{
				if (i == 0)
					Yr.data[i + k * ny][j] = idref;
				else
					Yr.data[i + k * ny][j] = weref;
			}
		}
	}

	//GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;

	/*--- Build QP ---*/

	for (k = 0; k<Tp; k++)
	{
		if (k == 0)
			multiply_matrix(C, A, CAA);
		else
			multiply_matrix(CA, A, CAA);
		copy_matrix(CAA, CA);
		for (i = 0; i<C.rows; i++)
		{
			for (j = 0; j<A.cols; j++)
			{
				Omega.data[i + k*C.rows][j] = CAA.data[i][j];
			}
		}
	}

	//GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;

	for (k = 0; k<Tp; k++)
	{
		if (k == 0)
		{
			multiply_matrix(C, B, CAB);
		}
		else
		{
			if (k == 1)
				multiply_matrix(C, A, CAA);
			else
				multiply_matrix(CA, A, CAA);
			copy_matrix(CAA, CA);
			multiply_matrix(CAA, B, CAB);
		}
		kr = k;
		for (l = 0; l<Tc; l++)
		{
			for (i = 0; i<C.rows; i++)
			{
				for (j = 0; j<B.cols; j++)
				{
					Gamma.data[i + kr*C.rows][j + l*B.cols] = CAB.data[i][j];
				}
			}
			kr++;
			if (kr >= Tp)
				break;
		}
	}
#else
    Yr.data[0][0] = idref;
    Yr.data[1][0] = weref;
    Yr.data[2][0] = idref;
    Yr.data[3][0] = weref;
    Yr.data[4][0] = idref;
    Yr.data[5][0] = weref;

	CAA.data[0][0] = C.data[0][0] * A.data[0][0] + C.data[0][1] * A.data[1][0] + C.data[0][2] * A.data[2][0] + C.data[0][3] * A.data[3][0] + C.data[0][4] * A.data[4][0];
	CAA.data[0][1] = C.data[0][0] * A.data[0][1] + C.data[0][1] * A.data[1][1] + C.data[0][2] * A.data[2][1] + C.data[0][3] * A.data[3][1] + C.data[0][4] * A.data[4][1];
	CAA.data[0][2] = C.data[0][0] * A.data[0][2] + C.data[0][1] * A.data[1][2] + C.data[0][2] * A.data[2][2] + C.data[0][3] * A.data[3][2] + C.data[0][4] * A.data[4][2];
	CAA.data[0][3] = C.data[0][0] * A.data[0][3] + C.data[0][1] * A.data[1][3] + C.data[0][2] * A.data[2][3] + C.data[0][3] * A.data[3][3] + C.data[0][4] * A.data[4][3];
	CAA.data[0][4] = C.data[0][0] * A.data[0][4] + C.data[0][1] * A.data[1][4] + C.data[0][2] * A.data[2][4] + C.data[0][3] * A.data[3][4] + C.data[0][4] * A.data[4][4];
	CAA.data[1][0] = C.data[1][0] * A.data[0][0] + C.data[1][1] * A.data[1][0] + C.data[1][2] * A.data[2][0] + C.data[1][3] * A.data[3][0] + C.data[1][4] * A.data[4][0];
	CAA.data[1][1] = C.data[1][0] * A.data[0][1] + C.data[1][1] * A.data[1][1] + C.data[1][2] * A.data[2][1] + C.data[1][3] * A.data[3][1] + C.data[1][4] * A.data[4][1];
	CAA.data[1][2] = C.data[1][0] * A.data[0][2] + C.data[1][1] * A.data[1][2] + C.data[1][2] * A.data[2][2] + C.data[1][3] * A.data[3][2] + C.data[1][4] * A.data[4][2];
	CAA.data[1][3] = C.data[1][0] * A.data[0][3] + C.data[1][1] * A.data[1][3] + C.data[1][2] * A.data[2][3] + C.data[1][3] * A.data[3][3] + C.data[1][4] * A.data[4][3];
	CAA.data[1][4] = C.data[1][0] * A.data[0][4] + C.data[1][1] * A.data[1][4] + C.data[1][2] * A.data[2][4] + C.data[1][3] * A.data[3][4] + C.data[1][4] * A.data[4][4];

	Omega.data[0][0] = CAA.data[0][0];
	Omega.data[0][1] = CAA.data[0][1];
	Omega.data[0][2] = CAA.data[0][2];
	Omega.data[0][3] = CAA.data[0][3];
	Omega.data[0][4] = CAA.data[0][4];
	Omega.data[1][0] = CAA.data[1][0];
	Omega.data[1][1] = CAA.data[1][1];
	Omega.data[1][2] = CAA.data[1][2];
	Omega.data[1][3] = CAA.data[1][3];
	Omega.data[1][4] = CAA.data[1][4];

	/*Gamma??*/
	CAB.data[0][0] = CAA.data[0][0] * B.data[0][0] + CAA.data[0][1] * B.data[1][0] + CAA.data[0][2] * B.data[2][0] + CAA.data[0][3] * B.data[3][0] + CAA.data[0][4] * B.data[4][0];
	CAB.data[0][1] = CAA.data[0][0] * B.data[0][1] + CAA.data[0][1] * B.data[1][1] + CAA.data[0][2] * B.data[2][1] + CAA.data[0][3] * B.data[3][1] + CAA.data[0][4] * B.data[4][1];
	CAB.data[1][0] = CAA.data[1][0] * B.data[0][0] + CAA.data[1][1] * B.data[1][0] + CAA.data[1][2] * B.data[2][0] + CAA.data[1][3] * B.data[3][0] + CAA.data[1][4] * B.data[4][0];
	CAB.data[1][1] = CAA.data[1][0] * B.data[0][1] + CAA.data[1][1] * B.data[1][1] + CAA.data[1][2] * B.data[2][1] + CAA.data[1][3] * B.data[3][1] + CAA.data[1][4] * B.data[4][1];
	Gamma.data[2][0] = CAB.data[0][0];
	Gamma.data[2][1] = CAB.data[0][1];
	Gamma.data[3][0] = CAB.data[1][0];
	Gamma.data[3][1] = CAB.data[1][1];
	/***********/

	//multiply_matrix(CAA, A, CA);																					 
	CA.data[0][0] = CAA.data[0][0] * A.data[0][0] + CAA.data[0][1] * A.data[1][0] + CAA.data[0][2] * A.data[2][0] + CAA.data[0][3] * A.data[3][0] + CAA.data[0][4] * A.data[4][0];
	CA.data[0][1] = CAA.data[0][0] * A.data[0][1] + CAA.data[0][1] * A.data[1][1] + CAA.data[0][2] * A.data[2][1] + CAA.data[0][3] * A.data[3][1] + CAA.data[0][4] * A.data[4][1];
	CA.data[0][2] = CAA.data[0][0] * A.data[0][2] + CAA.data[0][1] * A.data[1][2] + CAA.data[0][2] * A.data[2][2] + CAA.data[0][3] * A.data[3][2] + CAA.data[0][4] * A.data[4][2];
	CA.data[0][3] = CAA.data[0][0] * A.data[0][3] + CAA.data[0][1] * A.data[1][3] + CAA.data[0][2] * A.data[2][3] + CAA.data[0][3] * A.data[3][3] + CAA.data[0][4] * A.data[4][3];
	CA.data[0][4] = CAA.data[0][0] * A.data[0][4] + CAA.data[0][1] * A.data[1][4] + CAA.data[0][2] * A.data[2][4] + CAA.data[0][3] * A.data[3][4] + CAA.data[0][4] * A.data[4][4];
	CA.data[1][0] = CAA.data[1][0] * A.data[0][0] + CAA.data[1][1] * A.data[1][0] + CAA.data[1][2] * A.data[2][0] + CAA.data[1][3] * A.data[3][0] + CAA.data[1][4] * A.data[4][0];
	CA.data[1][1] = CAA.data[1][0] * A.data[0][1] + CAA.data[1][1] * A.data[1][1] + CAA.data[1][2] * A.data[2][1] + CAA.data[1][3] * A.data[3][1] + CAA.data[1][4] * A.data[4][1];
	CA.data[1][2] = CAA.data[1][0] * A.data[0][2] + CAA.data[1][1] * A.data[1][2] + CAA.data[1][2] * A.data[2][2] + CAA.data[1][3] * A.data[3][2] + CAA.data[1][4] * A.data[4][2];
	CA.data[1][3] = CAA.data[1][0] * A.data[0][3] + CAA.data[1][1] * A.data[1][3] + CAA.data[1][2] * A.data[2][3] + CAA.data[1][3] * A.data[3][3] + CAA.data[1][4] * A.data[4][3];
	CA.data[1][4] = CAA.data[1][0] * A.data[0][4] + CAA.data[1][1] * A.data[1][4] + CAA.data[1][2] * A.data[2][4] + CAA.data[1][3] * A.data[3][4] + CAA.data[1][4] * A.data[4][4];

	Omega.data[2][0] = CA.data[0][0];
	Omega.data[2][1] = CA.data[0][1];
	Omega.data[2][2] = CA.data[0][2];
	Omega.data[2][3] = CA.data[0][3];
	Omega.data[2][4] = CA.data[0][4];
	Omega.data[3][0] = CA.data[1][0];
	Omega.data[3][1] = CA.data[1][1];
	Omega.data[3][2] = CA.data[1][2];
	Omega.data[3][3] = CA.data[1][3];
	Omega.data[3][4] = CA.data[1][4];
	/*Gamma??*/
	CAB.data[0][0] = CA.data[0][0] * B.data[0][0] + CA.data[0][1] * B.data[1][0] + CA.data[0][2] * B.data[2][0] + CA.data[0][3] * B.data[3][0] + CA.data[0][4] * B.data[4][0];
	CAB.data[0][1] = CA.data[0][0] * B.data[0][1] + CA.data[0][1] * B.data[1][1] + CA.data[0][2] * B.data[2][1] + CA.data[0][3] * B.data[3][1] + CA.data[0][4] * B.data[4][1];
	CAB.data[1][0] = CA.data[1][0] * B.data[0][0] + CA.data[1][1] * B.data[1][0] + CA.data[1][2] * B.data[2][0] + CA.data[1][3] * B.data[3][0] + CA.data[1][4] * B.data[4][0];
	CAB.data[1][1] = CA.data[1][0] * B.data[0][1] + CA.data[1][1] * B.data[1][1] + CA.data[1][2] * B.data[2][1] + CA.data[1][3] * B.data[3][1] + CA.data[1][4] * B.data[4][1];
	Gamma.data[4][0] = CAB.data[0][0];
	Gamma.data[4][1] = CAB.data[0][1];
	Gamma.data[5][0] = CAB.data[1][0];
	Gamma.data[5][1] = CAB.data[1][1];

	//multiply_matrix(CA, A, CAA);																				   
	CAA.data[0][0] = CA.data[0][0] * A.data[0][0] + CA.data[0][1] * A.data[1][0] + CA.data[0][2] * A.data[2][0] + CA.data[0][3] * A.data[3][0] + CA.data[0][4] * A.data[4][0];
	CAA.data[0][1] = CA.data[0][0] * A.data[0][1] + CA.data[0][1] * A.data[1][1] + CA.data[0][2] * A.data[2][1] + CA.data[0][3] * A.data[3][1] + CA.data[0][4] * A.data[4][1];
	CAA.data[0][2] = CA.data[0][0] * A.data[0][2] + CA.data[0][1] * A.data[1][2] + CA.data[0][2] * A.data[2][2] + CA.data[0][3] * A.data[3][2] + CA.data[0][4] * A.data[4][2];
	CAA.data[0][3] = CA.data[0][0] * A.data[0][3] + CA.data[0][1] * A.data[1][3] + CA.data[0][2] * A.data[2][3] + CA.data[0][3] * A.data[3][3] + CA.data[0][4] * A.data[4][3];
	CAA.data[0][4] = CA.data[0][0] * A.data[0][4] + CA.data[0][1] * A.data[1][4] + CA.data[0][2] * A.data[2][4] + CA.data[0][3] * A.data[3][4] + CA.data[0][4] * A.data[4][4];
	CAA.data[1][0] = CA.data[1][0] * A.data[0][0] + CA.data[1][1] * A.data[1][0] + CA.data[1][2] * A.data[2][0] + CA.data[1][3] * A.data[3][0] + CA.data[1][4] * A.data[4][0];
	CAA.data[1][1] = CA.data[1][0] * A.data[0][1] + CA.data[1][1] * A.data[1][1] + CA.data[1][2] * A.data[2][1] + CA.data[1][3] * A.data[3][1] + CA.data[1][4] * A.data[4][1];
	CAA.data[1][2] = CA.data[1][0] * A.data[0][2] + CA.data[1][1] * A.data[1][2] + CA.data[1][2] * A.data[2][2] + CA.data[1][3] * A.data[3][2] + CA.data[1][4] * A.data[4][2];
	CAA.data[1][3] = CA.data[1][0] * A.data[0][3] + CA.data[1][1] * A.data[1][3] + CA.data[1][2] * A.data[2][3] + CA.data[1][3] * A.data[3][3] + CA.data[1][4] * A.data[4][3];
	CAA.data[1][4] = CA.data[1][0] * A.data[0][4] + CA.data[1][1] * A.data[1][4] + CA.data[1][2] * A.data[2][4] + CA.data[1][3] * A.data[3][4] + CA.data[1][4] * A.data[4][4];

	Omega.data[4][0] = CAA.data[0][0];
	Omega.data[4][1] = CAA.data[0][1];
	Omega.data[4][2] = CAA.data[0][2];
	Omega.data[4][3] = CAA.data[0][3];
	Omega.data[4][4] = CAA.data[0][4];
	Omega.data[5][0] = CAA.data[1][0];
	Omega.data[5][1] = CAA.data[1][1];
	Omega.data[5][2] = CAA.data[1][2];
	Omega.data[5][3] = CAA.data[1][3];
	Omega.data[5][4] = CAA.data[1][4];

	CAB.data[0][0] = C.data[0][0] * B.data[0][0] + C.data[0][1] * B.data[1][0] + C.data[0][2] * B.data[2][0] + C.data[0][3] * B.data[3][0] + C.data[0][4] * B.data[4][0];
	CAB.data[0][1] = C.data[0][0] * B.data[0][1] + C.data[0][1] * B.data[1][1] + C.data[0][2] * B.data[2][1] + C.data[0][3] * B.data[3][1] + C.data[0][4] * B.data[4][1];
	CAB.data[1][0] = C.data[1][0] * B.data[0][0] + C.data[1][1] * B.data[1][0] + C.data[1][2] * B.data[2][0] + C.data[1][3] * B.data[3][0] + C.data[1][4] * B.data[4][0];
	CAB.data[1][1] = C.data[1][0] * B.data[0][1] + C.data[1][1] * B.data[1][1] + C.data[1][2] * B.data[2][1] + C.data[1][3] * B.data[3][1] + C.data[1][4] * B.data[4][1];

	Gamma.data[0][0] = CAB.data[0][0];
	Gamma.data[0][1] = CAB.data[0][1];
	Gamma.data[1][0] = CAB.data[1][0];
	Gamma.data[1][1] = CAB.data[1][1];
#endif
	//GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;

	/*--y = [idref,weref]'--*/

#ifndef opt
	set_identity_matrix(Q);
	set_identity_matrix(R);
	scale_matrix(Q, Q_w);
	scale_matrix(R, R_w);
#else
    /**/
    Q.data[0][0] = Q_w;
    Q.data[1][1] = Q_w;
    Q.data[2][2] = Q_w;
    Q.data[3][3] = Q_w;
    Q.data[4][4] = Q_w;
    Q.data[5][5] = Q_w;
    R.data[0][0] = R_w;
    R.data[1][1] = R_w;
#endif

	Q.data[0][0] = Id_w;
	Q.data[1][1] = We_w;
	Q.data[2][2] = Id2_w;
	Q.data[3][3] = We2_w;
    Q.data[4][4] = Id3_w;
    Q.data[5][5] = We3_w;
	R.data[0][0] = vd_w;
	R.data[1][1] = vq_w;
	//transpose_matrix(Gamma, temp1);		//temp1 = Gamma'*Q
	//multiply_matrix(temp1, Q, temp2);		//temp2 = Gamma'*Q
	//multiply_matrix(temp2, Gamma, temp3);	//temp3 = Gamma'*Q*Gamma
	//add_matrix(temp3, R, temp4);			//temp4 = Gamma'*Q*Gamma+R
	temp2.data[0][0] = Gamma.data[0][0] * Q.data[0][0];
	temp2.data[0][1] = Gamma.data[1][0] * Q.data[1][1];
	temp2.data[0][2] = Gamma.data[2][0] * Q.data[2][2];
	temp2.data[0][3] = Gamma.data[3][0] * Q.data[3][3];
	temp2.data[0][4] = Gamma.data[4][0] * Q.data[4][4];
	temp2.data[0][5] = Gamma.data[5][0] * Q.data[5][5];
	temp2.data[1][0] = Gamma.data[0][1] * Q.data[0][0];
	temp2.data[1][1] = Gamma.data[1][1] * Q.data[1][1];
	temp2.data[1][2] = Gamma.data[2][1] * Q.data[2][2];
	temp2.data[1][3] = Gamma.data[3][1] * Q.data[3][3];
	temp2.data[1][4] = Gamma.data[4][1] * Q.data[4][4];
	temp2.data[1][5] = Gamma.data[5][1] * Q.data[5][5];

	temp4.data[0][0] = temp2.data[0][0] * Gamma.data[0][0] + temp2.data[0][1] * Gamma.data[1][0] + temp2.data[0][2] * Gamma.data[2][0] + temp2.data[0][3] * Gamma.data[3][0] + temp2.data[0][4] * Gamma.data[4][0] + temp2.data[0][5] * Gamma.data[5][0] + R.data[0][0];
	temp4.data[0][1] = temp2.data[0][0] * Gamma.data[0][1] + temp2.data[0][1] * Gamma.data[1][1] + temp2.data[0][2] * Gamma.data[2][1] + temp2.data[0][3] * Gamma.data[3][1] + temp2.data[0][4] * Gamma.data[4][1] + temp2.data[0][5] * Gamma.data[5][1] + R.data[0][1];
	temp4.data[1][0] = temp2.data[1][0] * Gamma.data[0][0] + temp2.data[1][1] * Gamma.data[1][0] + temp2.data[1][2] * Gamma.data[2][0] + temp2.data[1][3] * Gamma.data[3][0] + temp2.data[1][4] * Gamma.data[4][0] + temp2.data[1][5] * Gamma.data[5][0] + R.data[1][0];
	temp4.data[1][1] = temp2.data[1][0] * Gamma.data[0][1] + temp2.data[1][1] * Gamma.data[1][1] + temp2.data[1][2] * Gamma.data[2][1] + temp2.data[1][3] * Gamma.data[3][1] + temp2.data[1][4] * Gamma.data[4][1] + temp2.data[1][5] * Gamma.data[5][1] + R.data[1][1];
	//scale_matrix(temp4, 2);				//2*(Gamma'*Q*Gamma+R)
	//copy_matrix(temp4, H);				//H = 2*(Gamma'*Q*Gamma+R)

	//multiply_matrix(Omega, X, temp5);		//temp5 = Omega*X
	//subtract_matrix(Yr, temp5, temp6);	//temp6 = Yr - Omega*X
	temp6.data[0][0] = Yr.data[0][0] - (Omega.data[0][0] * X.data[0][0] + Omega.data[0][1] * X.data[1][0] + Omega.data[0][2] * X.data[2][0] + Omega.data[0][3] * X.data[3][0] + Omega.data[0][4] * X.data[4][0]);
	temp6.data[1][0] = Yr.data[1][0] - (Omega.data[1][0] * X.data[0][0] + Omega.data[1][1] * X.data[1][0] + Omega.data[1][2] * X.data[2][0] + Omega.data[1][3] * X.data[3][0] + Omega.data[1][4] * X.data[4][0]);
	temp6.data[2][0] = Yr.data[2][0] - (Omega.data[2][0] * X.data[0][0] + Omega.data[2][1] * X.data[1][0] + Omega.data[2][2] * X.data[2][0] + Omega.data[2][3] * X.data[3][0] + Omega.data[2][4] * X.data[4][0]);
	temp6.data[3][0] = Yr.data[3][0] - (Omega.data[3][0] * X.data[0][0] + Omega.data[3][1] * X.data[1][0] + Omega.data[3][2] * X.data[2][0] + Omega.data[3][3] * X.data[3][0] + Omega.data[3][4] * X.data[4][0]);
	temp6.data[4][0] = Yr.data[4][0] - (Omega.data[4][0] * X.data[0][0] + Omega.data[4][1] * X.data[1][0] + Omega.data[4][2] * X.data[2][0] + Omega.data[4][3] * X.data[3][0] + Omega.data[4][4] * X.data[4][0]);
	temp6.data[5][0] = Yr.data[5][0] - (Omega.data[5][0] * X.data[0][0] + Omega.data[5][1] * X.data[1][0] + Omega.data[5][2] * X.data[2][0] + Omega.data[5][3] * X.data[3][0] + Omega.data[5][4] * X.data[4][0]);
	
	//multiply_matrix(temp2, temp6, temp7);	//temp7 = Gamma'*Q*(Yr - Omega*X)
	temp7.data[0][0] = temp2.data[0][0] * temp6.data[0][0] + temp2.data[0][1] * temp6.data[1][0] + temp2.data[0][2] * temp6.data[2][0] + temp2.data[0][3] * temp6.data[3][0] + temp2.data[0][4] * temp6.data[4][0] + temp2.data[0][5] * temp6.data[5][0];
	temp7.data[1][0] = temp2.data[1][0] * temp6.data[0][0] + temp2.data[1][1] * temp6.data[1][0] + temp2.data[1][2] * temp6.data[2][0] + temp2.data[1][3] * temp6.data[3][0] + temp2.data[1][4] * temp6.data[4][0] + temp2.data[1][5] * temp6.data[5][0];

	//scale_matrix(temp7, -2);				//-2*Gamma'*Q*(Yr - Omega*X)
	//copy_matrix(temp7, f);				//f = -2*Gamma'*Q*(Yr - Omega*X)

	//GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;
	if (destructive_invert_matrix(temp4, temp8) == 0)
		return;


	u_ol.data[0][0] = temp8.data[0][0] * temp7.data[0][0] + temp8.data[0][1] * temp7.data[1][0];
	u_ol.data[1][0] = temp8.data[1][0] * temp7.data[0][0] + temp8.data[1][1] * temp7.data[1][0];
	//scale_matrix(u_ol, -1);

	//delta_u.data[0][0] = u_ol.data[0][0];
	//delta_u.data[1][0] = u_ol.data[1][0];

	//add_matrix(u, delta_u, u);
	vd[0] = u.data[0][0] + u_ol.data[0][0];
	vq[0] = u.data[1][0] + u_ol.data[1][0];
	
	/* torque/current limit */

	qmax = (iqmax - iq)*Lq / Ts + Rs*iq + Ld*we*id + we*Fai + 0 * (iqmax - iq);

	qmin = (iqmin - iq)*Lq / Ts + Rs*iq + Ld*we*id + we*Fai + 0 * (iqmin - iq);

	if (vq[0] > qmax)
		vq[0] = qmax;
	if (vq[0] < qmin)
		vq[0] = qmin;

	/* voltage limit */
	//qlim2 = sqrt(weref/1256.6367)*qlim;
	qlim2 = sqrt(weref/2300)*qlim;

	if (vd[0] > dlim)
		vd[0] = dlim;
	if (vd[0] < -dlim)
		vd[0] = -dlim;

	if (vq[0] > qlim2)
		vq[0] = qlim2;
	if (vq[0] < -qlim2)
		vq[0] = -qlim2;

	/*--------------*/
    cnt++;
    if(cnt == COUNT)
    {
        cnt = 0;
        vd_old = vd[0];
        vq_old = vq[0];
        id_old = id;
        iq_old = iq;
        we_old = we;
    }
    //GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;
}
#endif

#if (BUILDLEVEL == LEVEL8)
void PMSM_MPC_CCS_Current(float id, float iq, float we, float idref, float iqref, float *vd, float *vq, float Ts)
{
    //Delay compensation
    // id(k) --> id(k+1)
    // iq(k) --> iq(k+1)
//    id1 = id;
//    iq1 = iq;
//    id = (1-Rs*Ts/Ld)*id1 + (Lq*we*Ts/Ld)*iq1 + (Ts/Ld)*vd_old;
//    iq = (-Ld*we*Ts/Lq)*id1 + (1-Rs*Ts/Lq)*iq1 + (Ts/Lq)*vq_old + (-we*Fai/Lq)*Ts;

    //int i, j, k, l, kr;
    /*--------------------------------------------------*/
    // Continuous Model
    /*** x = [id,iq]'    ***/
    /*** u = [ud,uq]'    ***/
    /*** y = [id,iq]'    ***/
#ifndef opt
    set_matrix(aa,
        -Rs / Ld, Lq / Ld*we,
        -Ld / Lq*we, -Rs / Lq);
    set_matrix(bb,
        1.0 / Ld, 0.0,
        0.0, 1.0 / Lq);
    set_matrix(cc,
        1.0, 0.0,
        0.0, 1.0);
#else
    aa.data[0][0] = -Rs / Ld;
    aa.data[0][1] = Lq / Ld*we;
    aa.data[1][0] = -Ld / Lq*we;
    aa.data[1][1] = -Rs / Lq;

    bb.data[0][0] = 1.0 / Ld;
    bb.data[0][1] = 0.0;
    bb.data[1][0] = 0.0;
    bb.data[1][1] = 1.0 / Lq;

    cc.data[0][0] = 1.0;
    cc.data[0][1] = 0.0;
    cc.data[1][0] = 0.0;
    cc.data[1][1] = 1.0;
#endif
    //GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;
    /*--------------------------------------------------*/
    // Discrete Model
#ifndef opt
    set_identity_matrix(eye);
    scale_matrix(aa, Ts);
    add_matrix(aa, eye, a);

    scale_matrix(bb, Ts);
    copy_matrix(bb, b);

    copy_matrix(cc, c);
#else
    a.data[0][0] = aa.data[0][0] * Ts + 1;
    a.data[0][1] = aa.data[0][1] * Ts + 0;
    a.data[1][0] = aa.data[1][0] * Ts + 0;
    a.data[1][1] = aa.data[1][1] * Ts + 1;

    b.data[0][0] = bb.data[0][0] * Ts;
    b.data[0][1] = bb.data[0][1] * Ts;
    b.data[1][0] = bb.data[1][0] * Ts;
    b.data[1][1] = bb.data[1][1] * Ts;

    c.data[0][0] = cc.data[0][0];
    c.data[0][1] = cc.data[0][1];
    c.data[1][0] = cc.data[1][0];
    c.data[1][1] = cc.data[1][1];

#endif

    //GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;
    /*--------------------------------------------------*/
    // Extend the system

    //A = [a zeros(nx,ny);c*a eye(ny)];
#ifndef opt
    for (i = 0; i<a.rows; i++)
    {
        for (j = 0; j<a.cols; j++)
        {
            A.data[i][j] = a.data[i][j];
        }
    }
    for (i = 0; i<nx; i++)
    {
        for (j = 0; j<ny; j++)
        {
            A.data[i][j + a.cols] = 0;
        }
    }
    multiply_matrix(c, a, ca);
    for (i = 0; i<c.rows; i++)
    {
        for (j = 0; j<a.cols; j++)
        {
            A.data[i + a.rows][j] = ca.data[i][j];
        }
    }
    for (i = 0; i<ny; i++)
    {
        for (j = 0; j<ny; j++)
        {
            if (i == j)
                A.data[i + a.rows][j + a.cols] = 1;
            else
                A.data[i + a.rows][j + a.cols] = 0;
        }
    }
#else
    A.data[0][0] = a.data[0][0];
    A.data[0][1] = a.data[0][1];
    A.data[1][0] = a.data[1][0];
    A.data[1][1] = a.data[1][1];

    A.data[0][2] = 0;
    A.data[0][3] = 0;
    A.data[1][2] = 0;
    A.data[1][3] = 0;

    //multiply_matrix(c, a, ca);
    ca.data[0][0] = c.data[0][0] * a.data[0][0] + c.data[0][1] * a.data[1][0];
    ca.data[0][1] = c.data[0][0] * a.data[0][1] + c.data[0][1] * a.data[1][1];
    ca.data[1][0] = c.data[1][0] * a.data[0][0] + c.data[1][1] * a.data[1][0];
    ca.data[1][1] = c.data[1][0] * a.data[0][1] + c.data[1][1] * a.data[1][1];

    A.data[2][0] = ca.data[0][0];
    A.data[2][1] = ca.data[0][1];
    A.data[3][0] = ca.data[1][0];
    A.data[3][1] = ca.data[1][1];

    A.data[2][2] = 1;
    A.data[2][3] = 0;
    A.data[3][2] = 0;
    A.data[3][3] = 1;
#endif

    //B = [b;c*b];
#ifndef opt
    for (i = 0; i<b.rows; i++)
    {
        for (j = 0; j<b.cols; j++)
        {
            B.data[i][j] = b.data[i][j];
        }
    }
    multiply_matrix(c, b, cb);
    for (i = 0; i<c.rows; i++)
    {
        for (j = 0; j<b.cols; j++)
        {
            B.data[i + b.rows][j] = cb.data[i][j];
        }
    }

    //C = [zeros(ny,nx) eye(ny)];
    for (i = 0; i<ny; i++)
    {
        for (j = 0; j<nx; j++)
        {
            C.data[i][j] = 0;
        }
    }
    for (i = 0; i<c.rows; i++)
    {
        for (j = 0; j<b.cols; j++)
        {
            if (i == j)
                C.data[i][j + nx] = 1;
            else
                C.data[i][j + nx] = 0;
        }
    }
#else
    B.data[0][0] = b.data[0][0];
    B.data[0][1] = b.data[0][1];
    B.data[1][0] = b.data[1][0];
    B.data[1][1] = b.data[1][1];
    //multiply_matrix(c, b, cb);
    cb.data[0][0] = c.data[0][0] * b.data[0][0] + c.data[0][1] * b.data[1][0];
    cb.data[0][1] = c.data[0][0] * b.data[0][1] + c.data[0][1] * b.data[1][1];
    cb.data[1][0] = c.data[1][0] * b.data[0][0] + c.data[1][1] * b.data[1][0];
    cb.data[1][1] = c.data[1][0] * b.data[0][1] + c.data[1][1] * b.data[1][1];

    B.data[2][0] = cb.data[0][0];
    B.data[2][1] = cb.data[0][1];
    B.data[3][0] = cb.data[1][0];
    B.data[3][1] = cb.data[1][1];

    C.data[0][0] = 0;
    C.data[0][1] = 0;
    C.data[1][0] = 0;
    C.data[1][1] = 0;
    C.data[0][2] = 1;
    C.data[0][3] = 0;
    C.data[1][2] = 0;
    C.data[1][3] = 1;
#endif
    //GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;

    /*--- Get inout ---*/
    // x = [id,iq]';
    // u = [vd_old,vq_old]';
    // X = [delta_x;y];
    // yr = [idref,iqref]';
    // Yr = repmat(yr,Tp,1);
#ifndef opt
    set_matrix(u,
        vd_old,
        vq_old);
    set_matrix(X,
        id - id_old,
        iq - iq_old,
        id,
        iq);
#else
    u.data[0][0] = vd_old;
    u.data[1][0] = vq_old;
    X.data[0][0] = id - id_old;
    X.data[1][0] = iq - iq_old;
    X.data[2][0] = id;
    X.data[3][0] = iq;
#endif

#ifndef opt
    for (k = 0; k<Tp; k++)
    {
        for (i = 0; i<ny; i++)
        {
            for (j = 0; j<1; j++)
            {
                if (i == 0)
                    Yr.data[i + k * ny][j] = idref;
                else
                    Yr.data[i + k * ny][j] = iqref;
            }
        }
    }

    //GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;

    /*--- Build QP ---*/

    for (k = 0; k<Tp; k++)
    {
        if (k == 0)
            multiply_matrix(C, A, CAA);
        else
            multiply_matrix(CA, A, CAA);
        copy_matrix(CAA, CA);
        for (i = 0; i<C.rows; i++)
        {
            for (j = 0; j<A.cols; j++)
            {
                Omega.data[i + k*C.rows][j] = CAA.data[i][j];
            }
        }
    }

    //GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;

    for (k = 0; k<Tp; k++)
    {
        if (k == 0)
        {
            multiply_matrix(C, B, CAB);
        }
        else
        {
            if (k == 1)
                multiply_matrix(C, A, CAA);
            else
                multiply_matrix(CA, A, CAA);
            copy_matrix(CAA, CA);
            multiply_matrix(CAA, B, CAB);
        }
        kr = k;
        for (l = 0; l<Tc; l++)
        {
            for (i = 0; i<C.rows; i++)
            {
                for (j = 0; j<B.cols; j++)
                {
                    Gamma.data[i + kr*C.rows][j + l*B.cols] = CAB.data[i][j];
                }
            }
            kr++;
            if (kr >= Tp)
                break;
        }
    }
#else
    Yr.data[0][0] = idref;
    Yr.data[1][0] = iqref;
    Yr.data[2][0] = idref;
    Yr.data[3][0] = iqref;
    Yr.data[4][0] = idref;
    Yr.data[5][0] = iqref;

    CAA.data[0][0] = C.data[0][0] * A.data[0][0] + C.data[0][1] * A.data[1][0] + C.data[0][2] * A.data[2][0] + C.data[0][3] * A.data[3][0];
    CAA.data[0][1] = C.data[0][0] * A.data[0][1] + C.data[0][1] * A.data[1][1] + C.data[0][2] * A.data[2][1] + C.data[0][3] * A.data[3][1];
    CAA.data[0][2] = C.data[0][0] * A.data[0][2] + C.data[0][1] * A.data[1][2] + C.data[0][2] * A.data[2][2] + C.data[0][3] * A.data[3][2];
    CAA.data[0][3] = C.data[0][0] * A.data[0][3] + C.data[0][1] * A.data[1][3] + C.data[0][2] * A.data[2][3] + C.data[0][3] * A.data[3][3];
    CAA.data[1][0] = C.data[1][0] * A.data[0][0] + C.data[1][1] * A.data[1][0] + C.data[1][2] * A.data[2][0] + C.data[1][3] * A.data[3][0];
    CAA.data[1][1] = C.data[1][0] * A.data[0][1] + C.data[1][1] * A.data[1][1] + C.data[1][2] * A.data[2][1] + C.data[1][3] * A.data[3][1];
    CAA.data[1][2] = C.data[1][0] * A.data[0][2] + C.data[1][1] * A.data[1][2] + C.data[1][2] * A.data[2][2] + C.data[1][3] * A.data[3][2];
    CAA.data[1][3] = C.data[1][0] * A.data[0][3] + C.data[1][1] * A.data[1][3] + C.data[1][2] * A.data[2][3] + C.data[1][3] * A.data[3][3];

    Omega.data[0][0] = CAA.data[0][0];
    Omega.data[0][1] = CAA.data[0][1];
    Omega.data[0][2] = CAA.data[0][2];
    Omega.data[0][3] = CAA.data[0][3];
    Omega.data[1][0] = CAA.data[1][0];
    Omega.data[1][1] = CAA.data[1][1];
    Omega.data[1][2] = CAA.data[1][2];
    Omega.data[1][3] = CAA.data[1][3];

    /*Gamma??*/
    CAB.data[0][0] = CAA.data[0][0] * B.data[0][0] + CAA.data[0][1] * B.data[1][0] + CAA.data[0][2] * B.data[2][0] + CAA.data[0][3] * B.data[3][0];
    CAB.data[0][1] = CAA.data[0][0] * B.data[0][1] + CAA.data[0][1] * B.data[1][1] + CAA.data[0][2] * B.data[2][1] + CAA.data[0][3] * B.data[3][1];
    CAB.data[1][0] = CAA.data[1][0] * B.data[0][0] + CAA.data[1][1] * B.data[1][0] + CAA.data[1][2] * B.data[2][0] + CAA.data[1][3] * B.data[3][0];
    CAB.data[1][1] = CAA.data[1][0] * B.data[0][1] + CAA.data[1][1] * B.data[1][1] + CAA.data[1][2] * B.data[2][1] + CAA.data[1][3] * B.data[3][1];
	
    Gamma.data[2][0] = CAB.data[0][0];
    Gamma.data[2][1] = CAB.data[0][1];
    Gamma.data[3][0] = CAB.data[1][0];
    Gamma.data[3][1] = CAB.data[1][1];
    /***********/

    //multiply_matrix(CAA, A, CA);
    CA.data[0][0] = CAA.data[0][0] * A.data[0][0] + CAA.data[0][1] * A.data[1][0] + CAA.data[0][2] * A.data[2][0] + CAA.data[0][3] * A.data[3][0];
    CA.data[0][1] = CAA.data[0][0] * A.data[0][1] + CAA.data[0][1] * A.data[1][1] + CAA.data[0][2] * A.data[2][1] + CAA.data[0][3] * A.data[3][1];
    CA.data[0][2] = CAA.data[0][0] * A.data[0][2] + CAA.data[0][1] * A.data[1][2] + CAA.data[0][2] * A.data[2][2] + CAA.data[0][3] * A.data[3][2];
    CA.data[0][3] = CAA.data[0][0] * A.data[0][3] + CAA.data[0][1] * A.data[1][3] + CAA.data[0][2] * A.data[2][3] + CAA.data[0][3] * A.data[3][3];
    CA.data[1][0] = CAA.data[1][0] * A.data[0][0] + CAA.data[1][1] * A.data[1][0] + CAA.data[1][2] * A.data[2][0] + CAA.data[1][3] * A.data[3][0];
    CA.data[1][1] = CAA.data[1][0] * A.data[0][1] + CAA.data[1][1] * A.data[1][1] + CAA.data[1][2] * A.data[2][1] + CAA.data[1][3] * A.data[3][1];
    CA.data[1][2] = CAA.data[1][0] * A.data[0][2] + CAA.data[1][1] * A.data[1][2] + CAA.data[1][2] * A.data[2][2] + CAA.data[1][3] * A.data[3][2];
    CA.data[1][3] = CAA.data[1][0] * A.data[0][3] + CAA.data[1][1] * A.data[1][3] + CAA.data[1][2] * A.data[2][3] + CAA.data[1][3] * A.data[3][3];
 
    Omega.data[2][0] = CA.data[0][0];
    Omega.data[2][1] = CA.data[0][1];
    Omega.data[2][2] = CA.data[0][2];
    Omega.data[2][3] = CA.data[0][3];
    Omega.data[3][0] = CA.data[1][0];
    Omega.data[3][1] = CA.data[1][1];
    Omega.data[3][2] = CA.data[1][2];
    Omega.data[3][3] = CA.data[1][3];

    /*Gamma??*/
    CAB.data[0][0] = CA.data[0][0] * B.data[0][0] + CA.data[0][1] * B.data[1][0] + CA.data[0][2] * B.data[2][0] + CA.data[0][3] * B.data[3][0];
    CAB.data[0][1] = CA.data[0][0] * B.data[0][1] + CA.data[0][1] * B.data[1][1] + CA.data[0][2] * B.data[2][1] + CA.data[0][3] * B.data[3][1];
    CAB.data[1][0] = CA.data[1][0] * B.data[0][0] + CA.data[1][1] * B.data[1][0] + CA.data[1][2] * B.data[2][0] + CA.data[1][3] * B.data[3][0];
    CAB.data[1][1] = CA.data[1][0] * B.data[0][1] + CA.data[1][1] * B.data[1][1] + CA.data[1][2] * B.data[2][1] + CA.data[1][3] * B.data[3][1];
	
    Gamma.data[4][0] = CAB.data[0][0];
    Gamma.data[4][1] = CAB.data[0][1];
    Gamma.data[5][0] = CAB.data[1][0];
    Gamma.data[5][1] = CAB.data[1][1];

    //multiply_matrix(CA, A, CAA);
    CAA.data[0][0] = CA.data[0][0] * A.data[0][0] + CA.data[0][1] * A.data[1][0] + CA.data[0][2] * A.data[2][0] + CA.data[0][3] * A.data[3][0];
    CAA.data[0][1] = CA.data[0][0] * A.data[0][1] + CA.data[0][1] * A.data[1][1] + CA.data[0][2] * A.data[2][1] + CA.data[0][3] * A.data[3][1];
    CAA.data[0][2] = CA.data[0][0] * A.data[0][2] + CA.data[0][1] * A.data[1][2] + CA.data[0][2] * A.data[2][2] + CA.data[0][3] * A.data[3][2];
    CAA.data[0][3] = CA.data[0][0] * A.data[0][3] + CA.data[0][1] * A.data[1][3] + CA.data[0][2] * A.data[2][3] + CA.data[0][3] * A.data[3][3];
    CAA.data[1][0] = CA.data[1][0] * A.data[0][0] + CA.data[1][1] * A.data[1][0] + CA.data[1][2] * A.data[2][0] + CA.data[1][3] * A.data[3][0];
    CAA.data[1][1] = CA.data[1][0] * A.data[0][1] + CA.data[1][1] * A.data[1][1] + CA.data[1][2] * A.data[2][1] + CA.data[1][3] * A.data[3][1];
    CAA.data[1][2] = CA.data[1][0] * A.data[0][2] + CA.data[1][1] * A.data[1][2] + CA.data[1][2] * A.data[2][2] + CA.data[1][3] * A.data[3][2];
    CAA.data[1][3] = CA.data[1][0] * A.data[0][3] + CA.data[1][1] * A.data[1][3] + CA.data[1][2] * A.data[2][3] + CA.data[1][3] * A.data[3][3];

    Omega.data[4][0] = CAA.data[0][0];
    Omega.data[4][1] = CAA.data[0][1];
    Omega.data[4][2] = CAA.data[0][2];
    Omega.data[4][3] = CAA.data[0][3];
    Omega.data[5][0] = CAA.data[1][0];
    Omega.data[5][1] = CAA.data[1][1];
    Omega.data[5][2] = CAA.data[1][2];
    Omega.data[5][3] = CAA.data[1][3];

    CAB.data[0][0] = C.data[0][0] * B.data[0][0] + C.data[0][1] * B.data[1][0] + C.data[0][2] * B.data[2][0] + C.data[0][3] * B.data[3][0];
    CAB.data[0][1] = C.data[0][0] * B.data[0][1] + C.data[0][1] * B.data[1][1] + C.data[0][2] * B.data[2][1] + C.data[0][3] * B.data[3][1];
    CAB.data[1][0] = C.data[1][0] * B.data[0][0] + C.data[1][1] * B.data[1][0] + C.data[1][2] * B.data[2][0] + C.data[1][3] * B.data[3][0];
    CAB.data[1][1] = C.data[1][0] * B.data[0][1] + C.data[1][1] * B.data[1][1] + C.data[1][2] * B.data[2][1] + C.data[1][3] * B.data[3][1];

    Gamma.data[0][0] = CAB.data[0][0];
    Gamma.data[0][1] = CAB.data[0][1];
    Gamma.data[1][0] = CAB.data[1][0];
    Gamma.data[1][1] = CAB.data[1][1];
#endif
    //GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;

    /*--y = [idref,weref]'--*/

#ifndef opt
    set_identity_matrix(Q);
    set_identity_matrix(R);
    scale_matrix(Q, Q_w);
    scale_matrix(R, R_w);
#else
    /**/
    Q.data[0][0] = Q_w;
    Q.data[1][1] = Q_w;
    Q.data[2][2] = Q_w;
    Q.data[3][3] = Q_w;
    Q.data[4][4] = Q_w;
    Q.data[5][5] = Q_w;
    R.data[0][0] = R_w;
    R.data[1][1] = R_w;
#endif

    Q.data[0][0] = Id_w;
    Q.data[1][1] = Iq_w;
    Q.data[2][2] = Id2_w;
    Q.data[3][3] = Iq2_w;
    Q.data[4][4] = Id3_w;
    Q.data[5][5] = Iq3_w;
    R.data[0][0] = vd_w;
    R.data[1][1] = vq_w;
    //transpose_matrix(Gamma, temp1);       //temp1 = Gamma'*Q
    //multiply_matrix(temp1, Q, temp2);     //temp2 = Gamma'*Q
    //multiply_matrix(temp2, Gamma, temp3); //temp3 = Gamma'*Q*Gamma
    //add_matrix(temp3, R, temp4);          //temp4 = Gamma'*Q*Gamma+R
    temp2.data[0][0] = Gamma.data[0][0] * Q.data[0][0];
    temp2.data[0][1] = Gamma.data[1][0] * Q.data[1][1];
    temp2.data[0][2] = Gamma.data[2][0] * Q.data[2][2];
    temp2.data[0][3] = Gamma.data[3][0] * Q.data[3][3];
    temp2.data[0][4] = Gamma.data[4][0] * Q.data[4][4];
    temp2.data[0][5] = Gamma.data[5][0] * Q.data[5][5];
    temp2.data[1][0] = Gamma.data[0][1] * Q.data[0][0];
    temp2.data[1][1] = Gamma.data[1][1] * Q.data[1][1];
    temp2.data[1][2] = Gamma.data[2][1] * Q.data[2][2];
    temp2.data[1][3] = Gamma.data[3][1] * Q.data[3][3];
    temp2.data[1][4] = Gamma.data[4][1] * Q.data[4][4];
    temp2.data[1][5] = Gamma.data[5][1] * Q.data[5][5];

    temp4.data[0][0] = temp2.data[0][0] * Gamma.data[0][0] + temp2.data[0][1] * Gamma.data[1][0] + temp2.data[0][2] * Gamma.data[2][0] + temp2.data[0][3] * Gamma.data[3][0] + temp2.data[0][4] * Gamma.data[4][0] + temp2.data[0][5] * Gamma.data[5][0] + R.data[0][0];
    temp4.data[0][1] = temp2.data[0][0] * Gamma.data[0][1] + temp2.data[0][1] * Gamma.data[1][1] + temp2.data[0][2] * Gamma.data[2][1] + temp2.data[0][3] * Gamma.data[3][1] + temp2.data[0][4] * Gamma.data[4][1] + temp2.data[0][5] * Gamma.data[5][1] + R.data[0][1];
    temp4.data[1][0] = temp2.data[1][0] * Gamma.data[0][0] + temp2.data[1][1] * Gamma.data[1][0] + temp2.data[1][2] * Gamma.data[2][0] + temp2.data[1][3] * Gamma.data[3][0] + temp2.data[1][4] * Gamma.data[4][0] + temp2.data[1][5] * Gamma.data[5][0] + R.data[1][0];
    temp4.data[1][1] = temp2.data[1][0] * Gamma.data[0][1] + temp2.data[1][1] * Gamma.data[1][1] + temp2.data[1][2] * Gamma.data[2][1] + temp2.data[1][3] * Gamma.data[3][1] + temp2.data[1][4] * Gamma.data[4][1] + temp2.data[1][5] * Gamma.data[5][1] + R.data[1][1];
    //scale_matrix(temp4, 2);               //2*(Gamma'*Q*Gamma+R)
    //copy_matrix(temp4, H);                //H = 2*(Gamma'*Q*Gamma+R)

    //multiply_matrix(Omega, X, temp5);     //temp5 = Omega*X
    //subtract_matrix(Yr, temp5, temp6);    //temp6 = Yr - Omega*X
    temp6.data[0][0] = Yr.data[0][0] - (Omega.data[0][0] * X.data[0][0] + Omega.data[0][1] * X.data[1][0] + Omega.data[0][2] * X.data[2][0] + Omega.data[0][3] * X.data[3][0]);
    temp6.data[1][0] = Yr.data[1][0] - (Omega.data[1][0] * X.data[0][0] + Omega.data[1][1] * X.data[1][0] + Omega.data[1][2] * X.data[2][0] + Omega.data[1][3] * X.data[3][0]);
    temp6.data[2][0] = Yr.data[2][0] - (Omega.data[2][0] * X.data[0][0] + Omega.data[2][1] * X.data[1][0] + Omega.data[2][2] * X.data[2][0] + Omega.data[2][3] * X.data[3][0]);
    temp6.data[3][0] = Yr.data[3][0] - (Omega.data[3][0] * X.data[0][0] + Omega.data[3][1] * X.data[1][0] + Omega.data[3][2] * X.data[2][0] + Omega.data[3][3] * X.data[3][0]);
    temp6.data[4][0] = Yr.data[4][0] - (Omega.data[4][0] * X.data[0][0] + Omega.data[4][1] * X.data[1][0] + Omega.data[4][2] * X.data[2][0] + Omega.data[4][3] * X.data[3][0]);
    temp6.data[5][0] = Yr.data[5][0] - (Omega.data[5][0] * X.data[0][0] + Omega.data[5][1] * X.data[1][0] + Omega.data[5][2] * X.data[2][0] + Omega.data[5][3] * X.data[3][0]);

    //multiply_matrix(temp2, temp6, temp7); //temp7 = Gamma'*Q*(Yr - Omega*X)
    temp7.data[0][0] = temp2.data[0][0] * temp6.data[0][0] + temp2.data[0][1] * temp6.data[1][0] + temp2.data[0][2] * temp6.data[2][0] + temp2.data[0][3] * temp6.data[3][0] + temp2.data[0][4] * temp6.data[4][0] + temp2.data[0][5] * temp6.data[5][0];
    temp7.data[1][0] = temp2.data[1][0] * temp6.data[0][0] + temp2.data[1][1] * temp6.data[1][0] + temp2.data[1][2] * temp6.data[2][0] + temp2.data[1][3] * temp6.data[3][0] + temp2.data[1][4] * temp6.data[4][0] + temp2.data[1][5] * temp6.data[5][0];

    //scale_matrix(temp7, -2);              //-2*Gamma'*Q*(Yr - Omega*X)
    //copy_matrix(temp7, f);                //f = -2*Gamma'*Q*(Yr - Omega*X)

    //GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;
    if (destructive_invert_matrix(temp4, temp8) == 0)
        return;


    u_ol.data[0][0] = temp8.data[0][0] * temp7.data[0][0] + temp8.data[0][1] * temp7.data[1][0];
    u_ol.data[1][0] = temp8.data[1][0] * temp7.data[0][0] + temp8.data[1][1] * temp7.data[1][0];
    //scale_matrix(u_ol, -1);

    //delta_u.data[0][0] = u_ol.data[0][0];
    //delta_u.data[1][0] = u_ol.data[1][0];

    //add_matrix(u, delta_u, u);
    vd[0] = u.data[0][0] + u_ol.data[0][0];
    vq[0] = u.data[1][0] + u_ol.data[1][0];

    /* voltage limit */
    //qlim2 = sqrt(weref/1256.6367)*qlim;
    //qlim2 = sqrt(weref/1900)*qlim;

    /* voltage limit (separate)*/
//    if (vd[0] > dlim)
//        vd[0] = dlim;
//    if (vd[0] < -dlim)
//        vd[0] = -dlim;
//
//    if (vq[0] > qlim2)
//        vq[0] = qlim2;
//    if (vq[0] < -qlim2)
//        vq[0] = -qlim2;

    /* voltage limit (whole)*/
    usv = sqrt(vq[0]*vq[0] + vd[0]*vd[0]);
    if ( usv > (udc/sqrt(3.0)) )
    {
//        vd[0] = vd[0]/usv*(udc/sqrt(3.0));
//        vq[0] = vq[0]/usv*(udc/sqrt(3.0));
        alpha = (1-Rs*Ts/Ld)*id + Lq*we*Ts*iq/Ld - idref;
        beta = -Ld*we*Ts*id/Lq + (1-Rs*Ts/Lq)*iq - Ts*we*Fai/Lq - iqref;
        vd[0] = -alpha/sqrt(alpha*alpha+beta*beta)*udc/sqrt(3);
        vq[0] = -beta/sqrt(alpha*alpha+beta*beta)*udc/sqrt(3);
    }

    /*--------------*/
    cnt++;
    if(cnt == COUNT)
    {
        cnt = 0;
        vd_old = vd[0];
        vq_old = vq[0];
        id_old = id;
        iq_old = iq;
    }
    //GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;
}
#endif

#if (BUILDLEVEL == LEVEL7||BUILDLEVEL == LEVEL8)
void MPC_CCS_Init()
{
	aa = alloc_matrix(nx, nx);
	bb = alloc_matrix(nx, nu);
	cc = alloc_matrix(ny, nx);
	eye = alloc_matrix(nx, nx);
	a = alloc_matrix(aa.rows, aa.cols);
	b = alloc_matrix(bb.rows, bb.cols);
	c = alloc_matrix(cc.rows, cc.cols);
	Nx = a.cols + ny;
	Nu = b.cols;
	Ny = ny;

	Ar = a.rows + c.rows;
	Ac = Nx;
	Br = b.rows + c.rows;
	Bc = Nu;
	Cr = ny;
	Cc = nx + ny;
	A = alloc_matrix(Ar, Ac);
	B = alloc_matrix(Br, Bc);
	C = alloc_matrix(Cr, Cc);
	ca = alloc_matrix(c.rows, a.cols);
	cb = alloc_matrix(c.rows, b.cols);

	u = alloc_matrix(nu, 1);
	X = alloc_matrix(nx + ny, 1);
	Yr = alloc_matrix(2 * Tp, 1);

	Omega = alloc_matrix(Tp*Ny, Nx);
	Gamma = alloc_matrix(Tp*Ny, Tc*Nu);
	CA = alloc_matrix(C.rows, A.cols);
	CAA = alloc_matrix(C.rows, A.cols);
	CAB = alloc_matrix(C.rows, B.cols);

	Q = alloc_matrix(Tp*Ny, Tp*Ny);
	R = alloc_matrix(Tc*Nu, Tc*Nu);

	H = alloc_matrix(Tc*Nu, Tc*Nu);
	f = alloc_matrix(Tc*Nu, Yr.cols);

	temp1 = alloc_matrix(Gamma.cols, Gamma.rows);    //Gamma'
	temp2 = alloc_matrix(Gamma.cols, Q.cols);        //Gamma'*Q
	temp3 = alloc_matrix(Gamma.cols, Gamma.cols);    //Gamma'*a*Gamma
	temp4 = alloc_matrix(Gamma.cols, Gamma.cols);    //2*(Gamma'*a*Gamma+R)

	temp5 = alloc_matrix(Omega.rows, X.cols);    //Omega*X
	temp6 = alloc_matrix(Yr.rows, Yr.cols);      //(Yr-Omega*X)
	temp7 = alloc_matrix(Gamma.cols, Yr.cols);   //-2*Gamma'*Q*(Yr-Omega*X)
	temp8 = alloc_matrix(Gamma.cols, Gamma.cols);//HÄæ

	u_ol = alloc_matrix(H.rows, f.cols);
	delta_u = alloc_matrix(nu, 1);
}
#endif
// void MPC_CCS_Release()
// {
	// free_matrix(aa);
	// free_matrix(bb);
	// free_matrix(cc);
	// free_matrix(eye);
	// free_matrix(a);
	// free_matrix(b);
	// free_matrix(c);
	// free_matrix(A);
	// free_matrix(B);
	// free_matrix(C);
	// free_matrix(ca);
	// free_matrix(cb);
	// free_matrix(u);
	// free_matrix(X);
	// free_matrix(Yr);
	// free_matrix(Omega);
	// free_matrix(Gamma);
	// free_matrix(CA);
	// free_matrix(CAA);
	// free_matrix(CAB);
	// free_matrix(Q);
	// free_matrix(R);
	// free_matrix(H);
	// free_matrix(f);
	// free_matrix(temp1);
	// free_matrix(temp2);
	// free_matrix(temp3);
	// free_matrix(temp4);
	// free_matrix(temp5);
	// free_matrix(temp6);
	// free_matrix(temp7);
	// free_matrix(temp8);
	// free_matrix(u_ol);
	// free_matrix(delta_u);
// }


