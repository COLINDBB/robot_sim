#include <iostream>
#include <iomanip>
#include <conio.h>
#include <math.h>
#include <fstream>


using namespace std;

// Kinematic State Variables
double x,x_d;
double y,y_d;
double psi, psi_d;


// Right Wheel State Variables
double th_r, th_r_d;
double w_r, w_r_d;


// Left Wheel State Variables
double th_l, th_l_d;
double w_l, w_l_d;


// Input Variables
double V_l, V_r; //motor voltages.
double Tl_l, Tl_r;



// Motor Variables
double Kt;
double Ke;
double Ra;
double f;
double Jm;

double K1, K2, K3;

// Cart variables
double r;
double L;


//System & Simulation Variables
const int N = 7;  // # of state v
const int M = 4;        // # of inputs 
double X[N+1];
double Xd[N+1];
double R[M+1]; 
double t;

// X[1] = x;
// X[2] = y;
// X[3] = psi;
// X[4] = th_r;
// X[5] = th_l;
// X[6] = w_r;
// X[7] = w_l;


// Xd[1] = x_d;
// Xd[2] = y_d;
// Xd[3] = psi_d;
// Xd[4] = th_r_d;
// Xd[5] = th_l_d;
// Xd[6] = w_r_d;
// Xd[7] = w_l_d;

// R[0] = V_r;
// R[1] = V_l;
// R[2] = Tl_r;
// R[3] = Tl_l;






void initialize_X(double[], double[], int N);

void calculate_Xd(double[],double,int,double[], double[]);

void update_X(double[], double, int, double[]);

// We use Euler's method to simulate the system. 




void main(){

	initialize_X(X,R,N);
	
	cout << setprecision(3);




	while (1){

		calculate_Xd(X, 0, 0, Xd, R);

		update_X(X, 0.1, N, Xd);

		for (int i = 1; i <= N; i++){
			cout << X[i] << " ";
		}
		cout << "\n";
		_getch();
	}
	
	return;
}


void initialize_X(double X[], double R[], int N){
	
	//Initialize all variables to zero. 


	X[1] = 0; // x
	X[2] = 0; // y
	X[3] = -3.14159; // psi (radians)
	X[4] = 0; // th_r;
	X[5] = 0; // th_l;
	X[6] = 0; // w_r;
	X[7] = 0; // w_l;

	R[0] = 5; // V_r
	R[1] = 5; // V_l
	R[2] = 0; // Tl_r
	R[3] = 0; // Tl_l

	Kt = 0.2;
	Ke = 0.2;
	Ra = 0.07;
	f = 0.07;
	Jm = 0.5;
	
	K1 = (Kt*Ke + Ra*f) / (Ra*Jm);
	K2 = (1 / Jm);
	K3 = (Kt) / (Ra*Jm);

	r = 0.5;
	L = 6;



}


void calculate_Xd(double X[], double t, int N, double Xd[], double R[]){
	/*
	 x[1] = x;
	 x[2] = y;
	 x[3] = psi;
	 x[4] = th_r;
	 x[5] = th_l;
	 x[6] = w_r;
	 x[7] = w_l;
*/

	 //Xd[1] = x_d;
	 //Xd[2] = y_d;
	 //Xd[3] = psi_d;
	 //Xd[4] = th_r_d;
	 //Xd[5] = th_l_d;
	 //Xd[6] = w_r_d;
	 //Xd[7] = w_l_d;

	// Unpack. 

	x = X[1];
	y = X[2];
	psi = X[3];
	th_r = X[4]; 
	th_l = X[5]; 
	w_r = X[6]; 
	w_l = X[7];

	V_r = R[0];
	V_l = R[1]; 
	Tl_r = R[2]; 
	Tl_l = R[3]; 


	// Calculate the Derivative terms

	x_d = (r / 2)*(w_l + w_r)*cos(psi);
	y_d = (r / 2)*(w_l + w_r)*sin(psi);
	psi_d = (r / L)*(w_r - w_l);
	th_r_d = w_r;
	th_l_d = w_l;
	w_r_d = -K1*w_r - K2*Tl_r + K3*V_r;
	w_l_d = -K1*w_l - K2*Tl_l + K3*V_l;

	// Update the array (pack up)

	Xd[1] = x_d;
	Xd[2] = y_d;
	Xd[3] = psi_d;
	Xd[4] = th_r_d;
	Xd[5] = th_l_d;
	Xd[6] = w_r_d;
	Xd[7] = w_l_d;


	return;
}


void update_X(double X[], double t, int N, double Xd[]){

	for (int i = 1; i <= N; i++){
		
		X[i] = X[i] + t*Xd[i];
	

	}
	return;

}