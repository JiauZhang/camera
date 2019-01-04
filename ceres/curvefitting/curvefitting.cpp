#include <iostream>
#include <vector>
#include <random>
#include <ceres/ceres.h>

using namespace std;
using ceres::AutoDiffCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::CostFunction;
using ceres::SizedCostFunction;

struct CostFunctor {
  CostFunctor(double x, double y): _x(x), _y(y) {}
  template<typename T>
  bool operator()(const T* const m, const T* const c, T* residual) const {
    residual[0] = T(_y) - exp(m[0]*T(_x) + c[0]);
    return true;
  }  
private:
  double _x;
  double _y;
};

class QuadraticCostFunctor: public SizedCostFunction<1,1,1> {
public:
  QuadraticCostFunctor(double x, double y): _x(x), _y(y) {}
  virtual ~QuadraticCostFunctor() {}
  bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {//const cant't be ignore!
    residuals[0] = _y - exp(parameters[0][0]*_x + parameters[1][0]);
    
    if(jacobians && jacobians[0] && jacobians[1]) {
      //we should provide Solver negtive gradient!
      //so that costfunction can decrese. 
      jacobians[0][0] = -_x*exp(parameters[0][0]*_x + parameters[1][0]);
      jacobians[1][0] = -exp(parameters[0][0]*_x + parameters[1][0]);
    }
    
    return true;
  }
  
private:
  double _x, _y;
};

struct point_data {
  point_data(): x(0), y(0) {}
  double x;
  double y;
};

int main(int argc, char **argv) 
{
  //create data with Gaussian noise
  const int data_size = 50;
  struct point_data data[data_size];
  struct point_data point;
  default_random_engine generator;
  normal_distribution<double> distribution(0.0,0.5);
  //y = exp(0.3*x + 0.1)
  for(int i=0; i<data_size; i++) {
    point.x = 0.1 * i;
    point.y = exp(0.3*point.x + 0.1) + distribution(generator);
    data[i] = point;
  }
  
  double m = 0.0, c = 0.1;
  Problem problem;
  CostFunction* costfunction;
  for(int i=0; i<data_size; i++) {
    costfunction = new AutoDiffCostFunction<CostFunctor,1,1,1>(
      new CostFunctor(data[i].x, data[i].y));
    problem.AddResidualBlock(costfunction, NULL, &m, &c);
  }
  
  Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Initial m: " << 0.0 << " c: " << 0.1 << "\n";
  std::cout << "Final   m: " << m << " c: " << c << "\n";
  
  //Analytic
  m = 3.0, c = 1.1;
  for(int i=0; i<data_size; i++) {
    costfunction = new QuadraticCostFunctor(data[i].x, data[i].y);
    problem.AddResidualBlock(costfunction, NULL, &m, &c);
  }
  
  options.max_num_iterations = 100;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Initial m: " << 3.0 << " c: " << 1.1 << "\n";
  std::cout << "Final   m: " << m << " c: " << c << "\n";
  
  return 0;
}
