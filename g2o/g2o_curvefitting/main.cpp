#include <iostream>
#include <vector>
#include <random>
#include <boost/concept_check.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


using namespace std;
using namespace g2o;

class g2o_vertex: public BaseVertex<3, Eigen::Matrix<double, 1, 3>> {
	void oplusImpl(const double* v) {
		_estimate += Eigen::Matrix<double, 1, 3>(v);
	}
	void setToOriginImpl() {
		_estimate.setZero();
	}
	
	bool read(istream& is) {}
	bool write(ostream& os) const {}
};

class g2o_edge: public BaseUnaryEdge<1, double, g2o_vertex> {
public:
	explicit g2o_edge(double x): _x(x) {}
	void computeError() {
		g2o_vertex* v = static_cast<g2o_vertex*>(_vertices[0]);
		const Eigen::Matrix<double, 1, 3> est = v->estimate();
		_error(0, 0) = _measurement - exp(est(0,0)*_x*_x + est(0,1)*_x + est(0,2));
		//cout << "_error = " << _error(0,0) << endl;
	}
	
	bool read(istream& is) {}
	bool write(ostream& os) const {}
private:
	double _x;
};

//y = exp(3*x^2 + 2*x + 1)
int main(int argc, char **argv) 
{
	typedef BlockSolver<BlockSolverTraits<3, 1>> block_solver;
	block_solver::LinearSolverType* linear_solver = new LinearSolverDense<block_solver::PoseMatrixType>;
	block_solver* blk_slv = new block_solver(linear_solver);
	
	OptimizationAlgorithmLevenberg* algorithm = new OptimizationAlgorithmLevenberg(blk_slv);
	SparseOptimizer optimizer;
	optimizer.setAlgorithm(algorithm);
	optimizer.setVerbose(true);
	
	//加顶点
	g2o_vertex* vertex = new g2o_vertex;
	vertex->setEstimate(Eigen::Matrix<double, 1, 3>(0,0,0));
	vertex->setId(0);
	optimizer.addVertex(vertex);
	
	//生成观测值
	vector<double> _x,_y;
	double x_temp;
	default_random_engine generator;
	normal_distribution<double> distribution(0.0,0.5);
	for(int i=0;i<100;i++) {
		//100 * 0.005 = 0.5,此值不能太大,过大时exp(3*x^2 + 2*x + 1)就溢出了
		x_temp = i*0.005;
		_x.push_back(x_temp);
		_y.push_back(exp(3*x_temp*x_temp + 2*x_temp + 1) + distribution(generator));
	}
	//加边
	for(int i=0;i<100;i++) {
		g2o_edge* edge = new g2o_edge(_x[i]);
		edge->setId(i);
		edge->setVertex(0, vertex);
		edge->setInformation(Eigen::Matrix<double,1,1>(1/0.25));
		edge->setMeasurement(_y[i]);
		optimizer.addEdge(edge);
	}
	//开始进行优化估计
	optimizer.initializeOptimization();
	optimizer.optimize(100);
	
	cout << "optimized variable: " << vertex->estimate().transpose() << endl;
	
	return 0;
}
