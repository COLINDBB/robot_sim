#include <iostream>
#include <iomanip>
#include <conio.h>
#include <math.h>
#include <fstream>


using namespace std;

// Kinematic State Variables
double x,x_d; // x = 2D x position
double y,y_d; // y = 2D y position
double psi, psi_d; // psi = orientation of the car wrt to (eg) North. 


// Right Wheel State Variables
double th_r, th_r_d; // right shaft angle
double w_r, w_r_d;  // left shaft angle. 


// Left Wheel State Variables
double th_l, th_l_d;
double w_l, w_l_d;


// Input Variables
double V_l, V_r; //left and right motor voltages.
double Tl_l, Tl_r; // left and right external torques applied to shafts. 


// Motor Variables

double Kt; // KT*voltage = torque produced
double Ke; // Ke*w = back emf
double Ra; // Armature resistance
double f; // motor friction
double Jm; // Inertia attached to shaft. 

double K1, K2, K3; // combinations of motor variables to simplify calcs. 

// Cart variables
double r; // radius of wheels
double L; // distance between wheels (base)


//System & Simulation Variables
const int N = 7;        // # of state v
const int U = 4;        // # of inputs 
double X[N+1];          // array of state variables
double Xd[N+1];			// array of state variable derivatives
double M[U+1];			// array of input variables
double t;				// time 
double t_end;
double dt;

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






void initialize_X(double[], double[], int N); //set initial conditions of state variables and inputs

void calculate_inputs(double[], double, int, int, double[]); 

void calculate_Xd(double[],double,int,int, double[], double[]);

void update_X(double[], double, int, double[]);

void print_state_var(double[], int);
// We use Euler's method to simulate the; system. 

void send_to_file(double[], double, int, int, double[], double[], ofstream&);

void main(){

	initialize_X(X,M,N);
	char output_file[] = "motor_sim.dat";
	ofstream fout(output_file);

	cout << setprecision(3);

	

	while (t<t_end){

		calculate_inputs(X, t, N, U, M);
		calculate_Xd(X, t, N, U, M, Xd);
		update_X(X, dt, N, Xd);
		send_to_file(X, t, N, U, M, Xd, fout);
		//print_state_var(X, N);

		t += dt;	
	}
	fout.close();
	return;
}


void initialize_X(double X[], double M[], int N){
	
	//Initialize all variables to zero. 


	X[1] = 2; // x
	X[2] = 2; // y
	X[3] = 0; // psi (radians)
	X[4] = 0; // th_r;
	X[5] = 0; // th_l;
	X[6] = 0; // w_r;
	X[7] = 0; // w_l;

	M[1] = 10; // V_r
	M[2] = 1; // V_l
	M[3] = 0; // Tl_r
	M[4] = 0; // Tl_l

	Kt = 0.2;
	Ke = 0.2;
	Ra = 0.07;
	f = 0.05;
	Jm = 100;
	
	K1 = (Kt*Ke + Ra*f) / (Ra*Jm);
	K2 = (1 / Jm);
	K3 = (Kt) / (Ra*Jm);

	r = 0.5;
	L = 6;      

	t = 0;
	dt = 0.0005;
	t_end = 5;



}

void calculate_inputs(double x[], double t, int N, int U, double m[]){

	// Calculate the state of the inputs. 
	
	m[1] = m[1];
	m[2] = m[2];
	m[3] = 0; // sin(t);
	m[4] = 0; // sin(t);

	return;


}


void calculate_Xd(double X[], double t, int N, int U, double M[], double Xd[] ){
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

	V_r = M[1];
	V_l = M[2]; 
	Tl_r = M[3]; 
	Tl_l = M[4]; 


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


void update_X(double X[], double dt, int N, double Xd[]){

	for (int i = 1; i <= N; i++){
		
		X[i] = X[i] + dt*Xd[i];
	

	}
	return;

}

void print_state_var(double X[], int N){

	for (int i = 1; i <= N; i++){
		cout << X[i] << " ";
	}
	cout << endl;
}

void send_to_file(double X[], double t, int N, int U, double M[], double Xd[], ofstream &fout){

	// Send all the data to a file. 
	// make it CSV (comma delimited)
	// format: 
	// time,X[1],...,X[N],Xd[1],...,Xd[N],M[1],...,M[U],  

	fout << t << ",";
	for (int i = 1; i <= N; i++){
		fout << X[i] << ",";
	}
	for (int i = 1; i <= N; i++){
		fout << Xd[i] << ",";
	}
	for (int i = 1; i <= U-1; i++){
		fout << M[i] << ",";
	}fout << M[U] << "\n";



}