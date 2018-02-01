#include <iostream>
#include "glog\logging.h"
#include "ceres\ceres.h"

using namespace std;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct CostFunctor {
	template<typename T> bool operator()(const T* const x,  T* residual) const {
		residual[0] = T(10) - x[0];
		return true;
	}
};

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);

	double initial_x = 0.5;
	double x = initial_x;

	// BUild the problem
	Problem problem;

	// Set up the only cost function (also knowns as residual). This uses auto-diff to obtain the derivative (jacobian)
	CostFunction* cost_function = new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
	problem.AddResidualBlock(cost_function, NULL, &x);

	// Run the solver
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	Solver::Summary summary;
	Solve(options, &problem, &summary);

	cout << summary.BriefReport() << endl;
	cout << "x: " << initial_x << " -> " << x << endl;

	system("pause");
}