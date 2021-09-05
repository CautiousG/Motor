#include "arithmetic.h"
#define qlim 1000
#define dlim 1000
#define delta_dlim 10
#define delta_qlim 1000
#define Q_w 1
#define R_w 0.01
#define Id_w 20
#define We_w 1
#define Id2_w 1
#define We2_w 1

// #define Q_w 1
// #define R_w 0.01
#define COUNT 8
int Tp = 3;        // Prediction horizon
int Tc = 1;        // Control horizon
int cnt = 0;
double Rs = 1.65;
double Ld = 0.01;
double Lq = 0.01;
double J = 5e-4;
double Bz = 0;
double Fai = 0.28;
double p = 4;
//double Ts = 1e-4;     // Sample time
//double qlim = 100;

double vd_old = 0;
double vq_old = 0;
double id_old = 1;
double iq_old = 2;
double we_old = 3;

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
int nx = 3;
int nu = 2;
int ny = 2;
int Nx, Nu, Ny;
int Ar, Ac, Br, Bc, Cr, Cc;


void PMSM_MPC_CCS(double id, double iq, double we, double idref, double weref, double *vd, double *vq, double Ts)
{
	int i, j, k, l, kr;
	/*--------------------------------------------------*/
	// Continuous Model
	/*** x = [id,iq,we] ***/
	/*** u = [ud,uq]    ***/
	/*** y = [id,we]    ***/
	set_matrix(aa,
		-Rs / Ld, Lq / Ld*we, Lq / Ld*iq,
		-Ld / Lq*we, -Rs / Lq, -Ld / Lq*id - Fai / Lq,
		0.0, 3 * p*p / (2 * J)*Fai, 0.0);
	set_matrix(bb,
		1 / Ld, 0.0,
		0.0, 1 / Lq,
		0.0, 0.0);
	set_matrix(cc,
		1.0, 0.0, 0.0,
		0.0, 0.0, 1.0);

	/*--------------------------------------------------*/
	// Discrete Model


	set_identity_matrix(eye);
	scale_matrix(aa, Ts);
	add_matrix(aa, eye, a);

	scale_matrix(bb, Ts);
	copy_matrix(bb, b);

	copy_matrix(cc, c);

	//print_matrix(cc);
	/*--------------------------------------------------*/
	// Extend the system    

	//A = [a zeros(nx,ny);c*a eye(ny)];
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

	//B = [b;c*b];
	for (i = 0; i < b.rows; i++)
	{
		for (j = 0; j < b.cols; j++)
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

	/*--- Get inout ---*/
	// x = [id,iq,we]';
	// u = [vd_old,vq_old]';
	// X = [x;u];
	// yr = [idref,weref]';
	// Yr = repmat(yr,Tp,1);

	set_matrix(u,
		vd_old,
		vq_old);
	set_matrix(X,
		id - id_old,
		iq - iq_old,
		we - we_old,
		id,
		we);

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

	/*--y = [idref,weref]'--*/

	set_identity_matrix(Q);
	set_identity_matrix(R);
	scale_matrix(Q, Q_w);
	scale_matrix(R, R_w);

	/**/
	//Q.data[0][0] = Id_w;
	//Q.data[1][1] = We_w;
	//Q.data[2][2] = Id2_w;
	//Q.data[3][3] = We2_w;

	transpose_matrix(Gamma, temp1);		//temp1 = Gamma'*Q
	multiply_matrix(temp1, Q, temp2);		//temp2 = Gamma'*Q
	multiply_matrix(temp2, Gamma, temp3);	//temp3 = Gamma'*Q*Gamma
	add_matrix(temp3, R, temp4);			//temp4 = Gamma'*Q*Gamma+R
	scale_matrix(temp4, 2);				//2*(Gamma'*Q*Gamma+R)
	copy_matrix(temp4, H);				//H = 2*(Gamma'*Q*Gamma+R)

	multiply_matrix(Omega, X, temp5);		//temp5 = Omega*X
	subtract_matrix(Yr, temp5, temp6);	//temp6 = Yr - Omega*X
	multiply_matrix(temp2, temp6, temp7);	//temp7 = Gamma'*Q*(Yr - Omega*X)
	scale_matrix(temp7, -2);				//-2*Gamma'*Q*(Yr - Omega*X)
	copy_matrix(temp7, f);				//f = -2*Gamma'*Q*(Yr - Omega*X)


	if (destructive_invert_matrix(temp4, temp8) == 0)
	{
		printf("error!\n");
		return;
	}

	multiply_matrix(temp8, temp7, u_ol);

	scale_matrix(u_ol, -1);


	//delta_u.data[0][0] = u_ol.data[0][0];
	//delta_u.data[1][0] = u_ol.data[1][0];

	//     if (delta_u.data[0][0] > delta_dlim)
	// 		delta_u.data[0][0] = delta_dlim;
	// 	if (delta_u.data[0][0] < -delta_dlim)
	// 		delta_u.data[0][0] = -delta_dlim;
	//     if (delta_u.data[1][0] > delta_qlim)
	// 		delta_u.data[1][0] = delta_qlim;
	// 	if (delta_u.data[1][0] < -delta_qlim)
	// 		delta_u.data[1][0] = -delta_qlim; 

	vd[0] = u.data[0][0] + u_ol.data[0][0];
	vq[0] = u.data[1][0] + u_ol.data[1][0];
	//     qlim = 100 - a.data[1][0]*id_old - a.data[1][1]*iq_old - a.data[1][2]*we_old - we_old*id_old*Ts;
	//     qlim = qlim/b.data[1][1];
	//    qlim = 100;
		if (vd[0]>dlim)
			vd[0] = dlim;
		if (vd[0]<-dlim)
			vd[0] = -dlim;
	//     qlim = sqrt(310*310/3 - vd[0]*vd[0]);
		if (vq[0]>qlim)
			vq[0] = qlim;
		if (vq[0]<-qlim)
			vq[0] = -qlim;

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

}

void PMSM_MPC_CCS_OPT(double id, double iq, double we, double idref, double weref, double *vd, double *vq, double Ts)
{ 
	/*--------------------------------------------------*/
	// Continuous Model
	/*** x = [id,iq,we] ***/
	/*** u = [ud,uq]    ***/
	/*** y = [id,we]    ***/
	//set_matrix(aa,
	//	-Rs / Ld, Lq / Ld*we, Lq / Ld*iq,
	//	-Ld / Lq*we, -Rs / Lq, -Ld / Lq*id - Fai / Lq,
	//	0.0, 3 * p*p / (2 * J)*Fai, 0.0);
	aa.data[0][0] = -Rs / Ld;
	aa.data[0][1] = Lq / Ld*we;
	aa.data[0][2] = Lq / Ld*iq;
	aa.data[1][0] = -Ld / Lq*we;
	aa.data[1][1] = -Rs / Lq;
	aa.data[1][2] = -Ld / Lq*id - Fai / Lq;
	aa.data[2][0] = 0.0;
	aa.data[2][1] = 3 * p*p / (2 * J)*Fai;
	aa.data[2][2] = 0.0;

	//set_matrix(bb,
	//	1 / Ld, 0.0,
	//	0.0, 1 / Lq,
	//	0.0, 0.0);
	bb.data[0][0] = 1 / Ld;
	bb.data[0][1] = 0.0;
	bb.data[1][0] = 0.0;
	bb.data[1][1] = 1 / Lq;
	bb.data[2][0] = 0.0;
	bb.data[2][1] = 0.0;

	//set_matrix(cc,
	//	1.0, 0.0, 0.0,
	//	0.0, 0.0, 1.0);
	cc.data[0][0] = 1.0;
	cc.data[0][1] = 0.0;
	cc.data[0][2] = 0.0;
	cc.data[1][0] = 0.0;
	cc.data[1][1] = 0.0;
	cc.data[1][2] = 1.0;

	/*--------------------------------------------------*/
	// Discrete Model


	//set_identity_matrix(eye);
	//scale_matrix(aa, Ts);
	//add_matrix(aa, eye, a);
	a.data[0][0] = aa.data[0][0] * Ts + 1;
	a.data[0][1] = aa.data[0][1] * Ts + 0;
	a.data[0][2] = aa.data[0][2] * Ts + 0;
	a.data[1][0] = aa.data[1][0] * Ts + 0;
	a.data[1][1] = aa.data[1][1] * Ts + 1;
	a.data[1][2] = aa.data[1][2] * Ts + 0;
	a.data[2][0] = aa.data[2][0] * Ts + 0;
	a.data[2][1] = aa.data[2][1] * Ts + 0;
	a.data[2][2] = aa.data[2][2] * Ts + 1;

	//scale_matrix(bb, Ts);
	//copy_matrix(bb, b);
	b.data[0][0] = bb.data[0][0] * Ts;
	b.data[0][1] = bb.data[0][1] * Ts;
	b.data[1][0] = bb.data[1][0] * Ts;
	b.data[1][1] = bb.data[1][1] * Ts;
	b.data[2][0] = bb.data[2][0] * Ts;
	b.data[2][1] = bb.data[2][1] * Ts;


	//copy_matrix(cc, c);
	c.data[0][0] = cc.data[0][0];
	c.data[0][1] = cc.data[0][1];
	c.data[0][2] = cc.data[0][2];
	c.data[1][0] = cc.data[1][0];
	c.data[1][1] = cc.data[1][1];
	c.data[1][2] = cc.data[1][2];



	//print_matrix(cc);
	/*--------------------------------------------------*/
	// Extend the system    

	//A = [a zeros(nx,ny);c*a eye(ny)];
	//for (i = 0; i<a.rows; i++)
	//{
	//	for (j = 0; j<a.cols; j++)
	//	{
	//		A.data[i][j] = a.data[i][j];
	//	}
	//}
	A.data[0][0] = a.data[0][0];
	A.data[0][1] = a.data[0][1];
	A.data[0][2] = a.data[0][2];
	A.data[1][0] = a.data[1][0];
	A.data[1][1] = a.data[1][1];
	A.data[1][2] = a.data[1][2];
	A.data[2][0] = a.data[2][0];
	A.data[2][1] = a.data[2][1];
	A.data[2][2] = a.data[2][2];

	//for (i = 0; i<nx; i++)
	//{
	//	for (j = 0; j<ny; j++)
	//	{
	//		A.data[i][j + a.cols] = 0;
	//	}
	//}
	A.data[0][3] = 0;
	A.data[0][4] = 0;
	A.data[1][3] = 0;
	A.data[1][4] = 0;

	//multiply_matrix(c, a, ca);
	//for (i = 0; i<c.rows; i++)
	//{
	//	for (j = 0; j<a.cols; j++)
	//	{
	//		A.data[i + a.rows][j] = ca.data[i][j];
	//	}
	//}
	ca.data[0][0] = cc.data[0][0] * a.data[0][0] + cc.data[0][1] * a.data[1][0] + cc.data[0][2] * a.data[2][0];
	ca.data[0][1] = cc.data[0][0] * a.data[0][1] + cc.data[0][1] * a.data[1][1] + cc.data[0][2] * a.data[2][1];
	ca.data[0][2] = cc.data[0][0] * a.data[0][2] + cc.data[0][1] * a.data[1][2] + cc.data[0][2] * a.data[2][2];
	ca.data[1][0] = cc.data[1][0] * a.data[0][0] + cc.data[1][1] * a.data[1][0] + cc.data[1][2] * a.data[2][0];
	ca.data[1][1] = cc.data[1][0] * a.data[0][1] + cc.data[1][1] * a.data[1][1] + cc.data[1][2] * a.data[2][1];
	ca.data[1][2] = cc.data[1][0] * a.data[0][2] + cc.data[1][1] * a.data[1][2] + cc.data[1][2] * a.data[2][2];
	A.data[3][0] = ca.data[0][0];
	A.data[3][1] = ca.data[0][1];
	A.data[3][2] = ca.data[0][2];
	A.data[4][0] = ca.data[1][0];
	A.data[4][1] = ca.data[1][1];
	A.data[4][2] = ca.data[1][2];


	//for (i = 0; i<ny; i++)
	//{
	//	for (j = 0; j<ny; j++)
	//	{
	//		if (i == j)
	//			A.data[i + a.rows][j + a.cols] = 1;
	//		else
	//			A.data[i + a.rows][j + a.cols] = 0;
	//	}
	//}
	A.data[3][3] = 1;
	A.data[3][4] = 0;
	A.data[4][3] = 0;
	A.data[4][4] = 1;


	//B = [b;c*b];
	//for (i = 0; i<b.rows; i++)
	//{
	//	for (j = 0; j<b.cols; j++)
	//	{
	//		B.data[i][j] = b.data[i][j];
	//	}
	//}
	B.data[0][0] = b.data[0][0];
	B.data[0][1] = b.data[0][1];
	B.data[1][0] = b.data[1][0];
	B.data[1][1] = b.data[1][1];
	B.data[2][0] = b.data[2][0];
	B.data[2][1] = b.data[2][1];


	//multiply_matrix(c, b, cb);
	//for (i = 0; i<c.rows; i++)
	//{
	//	for (j = 0; j<b.cols; j++)
	//	{
	//		B.data[i + b.rows][j] = cb.data[i][j];
	//	}
	//}
	cb.data[0][0] = cc.data[0][0] * b.data[0][0] + cc.data[0][1] * b.data[1][0] + cc.data[0][2] * b.data[2][0];
	cb.data[0][1] = cc.data[0][0] * b.data[0][1] + cc.data[0][1] * b.data[1][1] + cc.data[0][2] * b.data[2][1];
	cb.data[1][0] = cc.data[1][0] * b.data[0][0] + cc.data[1][1] * b.data[1][0] + cc.data[1][2] * b.data[2][0];
	cb.data[1][1] = cc.data[1][0] * b.data[0][1] + cc.data[1][1] * b.data[1][1] + cc.data[1][2] * b.data[2][1];
	B.data[3][0] = cb.data[0][0];
	B.data[3][1] = cb.data[0][1];
	B.data[4][0] = cb.data[1][0];
	B.data[4][1] = cb.data[1][1];


	//C = [zeros(ny,nx) eye(ny)];
	//for (i = 0; i<ny; i++)
	//{
	//	for (j = 0; j<nx; j++)
	//	{
	//		C.data[i][j] = 0;
	//	}
	//}
	C.data[0][0] = 0;
	C.data[0][1] = 0;
	C.data[0][2] = 0;
	C.data[1][0] = 0;
	C.data[1][1] = 0;
	C.data[1][2] = 0;


	//for (i = 0; i<c.rows; i++)
	//{
	//	for (j = 0; j<b.cols; j++)
	//	{
	//		if (i == j)
	//			C.data[i][j + nx] = 1;
	//		else
	//			C.data[i][j + nx] = 0;
	//	}
	//}
	C.data[0][3] = 1;
	C.data[0][4] = 0;
	C.data[1][3] = 0;
	C.data[1][4] = 1;


	/*--- Get inout ---*/
	// x = [id,iq,we]';
	// u = [vd_old,vq_old]';
	// X = [x;u];
	// yr = [idref,weref]';
	// Yr = repmat(yr,Tp,1);
    
	//set_matrix(u,
	//	vd_old,
	//	vq_old);
	//set_matrix(X,
	//	id - id_old,
	//	iq - iq_old,
	//	we - we_old,
	//	id,
	//	we);
	u.data[0][0] = vd_old;
	u.data[1][0] = vq_old;
	X.data[0][0] = id - id_old;
	X.data[1][0] = iq - iq_old;
	X.data[2][0] = we - we_old;
	X.data[3][0] = id;
	X.data[4][0] = we;

	//for (k = 0; k<Tp; k++)
	//{
	//	for (i = 0; i<ny; i++)
	//	{
	//		for (j = 0; j<1; j++)
	//		{
	//			if (i == 0)
	//				Yr.data[i + k * ny][j] = idref;
	//			else
	//				Yr.data[i + k * ny][j] = weref;
	//		}
	//	}
	//}
	Yr.data[0][0] = idref;
	Yr.data[1][0] = weref;
	Yr.data[2][0] = idref;
	Yr.data[3][0] = weref;
	Yr.data[4][0] = idref;
	Yr.data[5][0] = weref;




	/*--- Build QP ---*/

	//for (k = 0; k<Tp; k++)
	//{
	//	if (k == 0)
	//		multiply_matrix(C, A, CAA);
	//	else
	//		multiply_matrix(CA, A, CAA);
	//	copy_matrix(CAA, CA);
	//	for (i = 0; i<C.rows; i++)
	//	{
	//		for (j = 0; j<A.cols; j++)
	//		{
	//			Omega.data[i + k*C.rows][j] = CAA.data[i][j];
	//		}
	//	}
	//}
	//multiply_matrix(C, A, CAA);
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

	/*Gamma¼ò»¯*/
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

	/*Gamma¼ò»¯*/
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


	//for (k = 0; k<Tp; k++)
	//{
	//	if (k == 0)
	//	{
	//		multiply_matrix(C, B, CAB);
	//	}
	//	else
	//	{
	//		if (k == 1)
	//			multiply_matrix(C, A, CAA);
	//		else
	//			multiply_matrix(CA, A, CAA);
	//		copy_matrix(CAA, CA);
	//		multiply_matrix(CAA, B, CAB);
	//	}
	//	kr = k;
	//	for (l = 0; l<Tc; l++)
	//	{
	//		for (i = 0; i<C.rows; i++)
	//		{
	//			for (j = 0; j<B.cols; j++)
	//			{
	//				Gamma.data[i + kr*C.rows][j + l*B.cols] = CAB.data[i][j];
	//			}
	//		}
	//		kr++;
	//		if (kr >= Tp)
	//			break;
	//	}
	//}

	//multiply_matrix(C, B, CAB);
	CAB.data[0][0] = C.data[0][0] * B.data[0][0] + C.data[0][1] * B.data[1][0] + C.data[0][2] * B.data[2][0] + C.data[0][3] * B.data[3][0] + C.data[0][4] * B.data[4][0];
	CAB.data[0][1] = C.data[0][0] * B.data[0][1] + C.data[0][1] * B.data[1][1] + C.data[0][2] * B.data[2][1] + C.data[0][3] * B.data[3][1] + C.data[0][4] * B.data[4][1];
	CAB.data[1][0] = C.data[1][0] * B.data[0][0] + C.data[1][1] * B.data[1][0] + C.data[1][2] * B.data[2][0] + C.data[1][3] * B.data[3][0] + C.data[1][4] * B.data[4][0];
	CAB.data[1][1] = C.data[1][0] * B.data[0][1] + C.data[1][1] * B.data[1][1] + C.data[1][2] * B.data[2][1] + C.data[1][3] * B.data[3][1] + C.data[1][4] * B.data[4][1];

	Gamma.data[0][0] = CAB.data[0][0];
	Gamma.data[0][1] = CAB.data[0][1];
	Gamma.data[1][0] = CAB.data[1][0];
	Gamma.data[1][1] = CAB.data[1][1];

	
	//multiply_matrix(C, A, CAA);
	//multiply_matrix(CAA, B, CAB);
	//Gamma.data[2][0] = CAB.data[0][0];
	//Gamma.data[2][1] = CAB.data[0][1];
	//Gamma.data[3][0] = CAB.data[1][0];
	//Gamma.data[3][1] = CAB.data[1][1];
	
	//multiply_matrix(CAA, A, CA);
	//multiply_matrix(CA, B, CAB);
	//Gamma.data[4][0] = CAB.data[0][0];
	//Gamma.data[4][1] = CAB.data[0][1];
	//Gamma.data[5][0] = CAB.data[1][0];
	//Gamma.data[5][1] = CAB.data[1][1];



	/*--y = [idref,weref]'--*/

	//set_identity_matrix(Q);
	//set_identity_matrix(R);
	//scale_matrix(Q, Q_w);
	//scale_matrix(R, R_w);
	Q.data[0][0] = Q_w;
	Q.data[1][1] = Q_w;
	Q.data[2][2] = Q_w;
	Q.data[3][3] = Q_w;
	Q.data[4][4] = Q_w;
	Q.data[5][5] = Q_w;
	R.data[0][0] = R_w;
	R.data[1][1] = R_w;


    /**/
    //Q.data[0][0] = Id_w;
    //Q.data[1][1] = We_w;
    //Q.data[2][2] = Id2_w;
    //Q.data[3][3] = We2_w;

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


	if (destructive_invert_matrix(temp4, temp8) == 0)
    {
		printf("error!\n");
        return;
    }

	//multiply_matrix(temp8, temp7, u_ol);
	u_ol.data[0][0] = temp8.data[0][0] * temp7.data[0][0] + temp8.data[0][1] * temp7.data[1][0];
	u_ol.data[1][0] = temp8.data[1][0] * temp7.data[0][0] + temp8.data[1][1] * temp7.data[1][0];

//	scale_matrix(u_ol, -1);


	//delta_u.data[0][0] = u_ol.data[0][0];
	//delta_u.data[1][0] = u_ol.data[1][0];

//     if (delta_u.data[0][0] > delta_dlim)
// 		delta_u.data[0][0] = delta_dlim;
// 	if (delta_u.data[0][0] < -delta_dlim)
// 		delta_u.data[0][0] = -delta_dlim;
//     if (delta_u.data[1][0] > delta_qlim)
// 		delta_u.data[1][0] = delta_qlim;
// 	if (delta_u.data[1][0] < -delta_qlim)
// 		delta_u.data[1][0] = -delta_qlim; 
    
	vd[0] = u.data[0][0] + u_ol.data[0][0];
	vq[0] = u.data[1][0] + u_ol.data[1][0];
//     qlim = 100 - a.data[1][0]*id_old - a.data[1][1]*iq_old - a.data[1][2]*we_old - we_old*id_old*Ts;
//     qlim = qlim/b.data[1][1];
//    qlim = 100;
	if (vd[0]>dlim)
		vd[0] = dlim;
	if (vd[0]<-dlim)
		vd[0] = -dlim;
//     qlim = sqrt(310*310/3 - vd[0]*vd[0]);
	if (vq[0]>qlim)
		vq[0] = qlim;
	if (vq[0]<-qlim)
		vq[0] = -qlim;

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

}

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

void MPC_CCS_Release()
{
	free_matrix(aa);
	free_matrix(bb);
	free_matrix(cc);
	free_matrix(eye);
	free_matrix(a);
	free_matrix(b);
	free_matrix(c);
	free_matrix(A);
	free_matrix(B);
	free_matrix(C);
	free_matrix(ca);
	free_matrix(cb);
	free_matrix(u);
	free_matrix(X);
	free_matrix(Yr);
	free_matrix(Omega);
	free_matrix(Gamma);
	free_matrix(CA);
	free_matrix(CAA);
	free_matrix(CAB);
	free_matrix(Q);
	free_matrix(R);
	free_matrix(H);
	free_matrix(f);
	free_matrix(temp1);
	free_matrix(temp2);
	free_matrix(temp3);
	free_matrix(temp4);
	free_matrix(temp5);
	free_matrix(temp6);
	free_matrix(temp7);
	free_matrix(temp8);
	free_matrix(u_ol);
	free_matrix(delta_u);
}