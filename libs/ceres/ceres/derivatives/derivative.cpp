#include <iostream>
#include "glog\logging.h"
#include "ceres\ceres.h"

using namespace std;
using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::SizedCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

struct NumericDiffCostFunctor {
	bool operator()(const double* const x, double* residual) const {
		residual[0] = 10.0 - x[0];
		return true;
	}
};

// unless you have a good reason to manage the jacobian computation yourself, 
// you use 'AutoDiffCostFunction or NUmericDiffCostFunction' to construct your residual blocks
class QuadraticCostFunction : public SizedCostFunction<1, 1> {
public:
	virtual ~QuadraticCostFunction() {}
	virtual bool Evaluate(double const* const * parameters,
		double * residuals, double ** jacobians) const {
		const double x = parameters[0][0];
		residuals[0] = 10 - x;
		if (jacobians != NULL && jacobians[0] != NULL) {
			jacobians[0][0] = -1;
		}
		return true;
	}
};

int main(int argc, char** argv) {

	google::InitGoogleLogging(argv[0]);

	double initial_x = 0.5;
	double x = initial_x;

	// BUild the problem
	Problem problem;

	CostFunction* cost_function = new NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL, 1, 1>(new NumericDiffCostFunctor);
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