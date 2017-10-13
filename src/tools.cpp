#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

	VectorXd rmse(4);
		rmse << 0,0,0,0;

	    // TODO: YOUR CODE HERE

		// check the validity of the following inputs:
		//  * the estimation vector size should not be zero
		//  * the estimation vector size should equal ground truth vector size
		// ... your code here
		if (estimations.size()!=ground_truth.size())
		{
		   	return rmse;

		}


		    //accumulate squared residuals
		for(int i=0; i < estimations.size(); ++i){

		    VectorXd resdual=estimations[i]-ground_truth[i];
		    resdual=resdual.array()*resdual.array();
		    rmse+=resdual;

	        // ... your code here

		}
		rmse=rmse/estimations.size();
		rmse=rmse.array().sqrt();



		//calculate the mean
		// ... your code here

		//calculate the squared root
		// ... your code here

		//return the result
		return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);

	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	float c1= std::pow(px,2)+std::pow(py,2);
	float c2= std::sqrt(c1);
	float c3= c1*c2;


	if( std::fabs(c1)==0){
		cout<<"Error Hj Divided by zero"<<endl;
		return Hj;
	}
	else {
		Hj << px/c2, py/c2, 0, 0,
		     -py/c1, px/c1, 0, 0,
		     py*(vx*py-vy*px)/c3, px*(vy*px-vx*py)/c3, px/c2, py/c2;


	}
	return Hj;

}
