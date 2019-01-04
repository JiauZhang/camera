#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>
#include <boost/concept_check.hpp>

using namespace std; 
using namespace g2o;
//Vertex数量n,这n个Vertex存储的数据类型
class powell_vertex: public BaseVertex<4, Eigen::Matrix<double,1,4>> {
	virtual void oplusImpl(const double* v) {
		//base_vertex.h: EstimateType _estimate;
		//cout << "oplus: "  << endl;
		//cout << Eigen::Matrix<double,1,4>(v) << endl;
		_estimate += Eigen::Matrix<double,1,4>(v);
		//cout << "_estimate: " << _estimate << endl;
	}
        virtual void setToOriginImpl() {
		//cout << "initial _estimate" << endl;
		_estimate << 0.5,1.5,2.5,3.5;
	}
	
	virtual bool read(std::istream& is) {}
        virtual bool write(std::ostream& os) const {}	//const 不能忽略
};
//e(xi)的维度、类型; vertex的类型
class powell_edge: public BaseUnaryEdge<4, Eigen::Matrix<double,1,4>, powell_vertex> {
	virtual void computeError() {
		//vertex* vtx = new vertex;
		const powell_vertex* v = static_cast<const powell_vertex*> (_vertices[0]);
		Eigen::Matrix<double,1,4> est =  v->estimate();
		//cout << "est: " << est(0,0)  << "---" << est(0,1)  <<"---" << est(0,2) <<  "---"<< est(0,3) << endl;
// 		_error(0,0) = (&_measurement)[0] - (est(0,0) + 10*est(0,1));
// 		_error(0,1) = (&_measurement)[1] - sqrt(5)*(est(0,2) - est(0,3));
// 		_error(0,2) = (&_measurement)[2] - (est(0,1) - 2*est(0,2))*(est(0,1) - 2*est(0,2));
// 		_error(0,3) = (&_measurement)[3] - sqrt(10)*(est(0,0) - est(0,2))*(est(0,0) - est(0,2));
		_error(0,0) = _measurement(0,0)- (est(0,0) + 10*est(0,1));
		_error(1,0) = _measurement(0,1) - sqrt(5)*(est(0,2) - est(0,3));
		_error(2,0) = _measurement(0,2) - (est(0,1) - 2*est(0,2))*(est(0,1) - 2*est(0,2));
		_error(3,0) = _measurement(0,3) - sqrt(10)*(est(0,0) - est(0,2))*(est(0,0) - est(0,2));
		cout << "_error: " << _error.transpose() << endl;
	}
	
	virtual bool read(std::istream& is) {}
        virtual bool write(std::ostream& os) const {}	//const 不能忽略
};

int main(int argc, char **argv) {    
	// 构建图优化，先设定g2o
	//注意维度对应--边--顶点---
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<4,4> > Block;  // int _PoseDim, int _LandmarkDim
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // 线性方程求解器
	Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
	// 梯度下降方法，从GN, LM, DogLeg 中选
	cout << "config blocksolver" << endl;
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
	// g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
	// g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
	g2o::SparseOptimizer optimizer;     // 图模型
	optimizer.setAlgorithm( solver );   // 设置求解器
	optimizer.setVerbose( true );       // 打开调试输出
	cout << "add Vertex" << endl;
	// 往图中增加顶点
	powell_vertex* v = new powell_vertex;
	Eigen::Matrix<double,1,4> estimate_initial;
	estimate_initial <<0.5,1.5,2.5,3.5;
	v->setEstimate(estimate_initial);
	v->setId(0);
	optimizer.addVertex( v );
	cout << "add Edge" << endl;
	//加边
	powell_edge* eg = new powell_edge;
	//double measurement_initial[4] = {21,-sqrt(5),16,9*sqrt(10)};
	//for (int i=0;i<4;i++) {
		eg->setId(0);
		eg->setVertex( 0, v );                // 设置连接的顶点
		Eigen::Matrix<double,1,4> measurement_initial;
		measurement_initial << 21,-sqrt(5),16,9*sqrt(10);
		eg->setMeasurement(measurement_initial);      // 观测数值
		//这个'4'对应于powell_edge第一个参数
		eg->setInformation(Eigen::Matrix<double,4,4>::Identity()*1/0.25); // 信息矩阵：协方差矩阵之逆
		optimizer.addEdge( eg );
	//}
	// 执行优化
	cout<<"start optimization"<<endl;
	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	cout<<"initial optimization"<<endl;
	optimizer.initializeOptimization();
	cout<<"start"<<endl;
	optimizer.optimize(200);
	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
	cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

	// 输出优化值
	Eigen::Matrix<double,1,4> abc_estimate = v->estimate();
	cout<<"estimated value: "<<abc_estimate.transpose()<<endl;
    
    return 0;
}
