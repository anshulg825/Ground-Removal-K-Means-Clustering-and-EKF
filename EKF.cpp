#include<iostream>
#include<math.h>
#include"Eigen/Dense"

using namespace Eigen;
using namespace std;

class KalmanFilter{

private:
	double dt=0.1;
	VectorXd x_hat(4,1);
	VectorXd x_hat_new(4,1);
	MatrixXd I(4,4) 
	I=setIdentity();

public:
	const MatrixXd F(4,4);
	const MatrixXd W(4,1);
	const MatrixXd Q(4,4);
	const MatrixXd H(2,4);
	const MatrixXd P(4,4);
	const MatrixXd P_new(4,4);
	const MatrixXd R(2,2);
	void update(x_hat_new,P_new,Z);
	void predict(x_hat,P);
	VectorXd state() { return x_hat; }
};

KalmanFilter::KalmanFilter() {
	F << 1,0,dt,0,
		 0,1,0,dt,
		 0,0,1, 0,
		 0,0,0, 1;

	H << 1,0,0,0,
		 0,1,0,0;

}

void KalmanFilter::predict(MatrixXd x_hat, MatrixXd P){
	x_hat_new = F*(x_hat);
	P_new=F*P*(F.transpose()) + Q;
	update(x_hat_new,P_new,Z);
}

void KalmanFilter::update(MatrixXd x_hat_new,MatrixXd P_new,MatrixXd Z){
	MatrixXd Y(2,1),S(2,2), K(4,2);
	Y = Z - H*(x_hat_new);
	S = H*P_new*(H.transpose()) + R;
	K = P_new*(H.transpose())*(S.inverse());
	x_hat = x_hat_new + K*Y;
	P = (I - K*H)*P_new;
}

void main()
{

}