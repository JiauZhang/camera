#include <iostream>
#include <boost/concept_check.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <random>
#include <chrono>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>

using namespace std;
using namespace g2o;

class hello_vertex: public BaseVertex<1, Eigen::Matrix<double,1,1>> {//数据类型要以Eigen::Matrix给出,因为需要转置操作
          /**
         * update the position of the node from the parameters in v.
         * Implement in your class!
         */
	//optimizable_graph.h  
        virtual void oplusImpl(const double* v) {
		cout << "v: " << *v << endl;
		_estimate(0,0) += *v;				//_estimate的类型即为继承模板时传入的类型
		cout << "_estimate: " << _estimate(0,0) << endl;
	}
	
        //! sets the node to the origin (used in the multilevel stuff)
        //optimizable_graph.h
        virtual void setToOriginImpl() {
		cout << "setToOriginImpl " << _estimate(0,0)  << endl;
		_estimate(0,0) = 0;
	}
	
	//! read the vertex from a stream, i.e., the internal state of the vertex
        virtual bool read(std::istream& is) {}
        //! write the vertex to a stream
        virtual bool write(std::ostream& os) const {}
};

class hello_edge: public BaseUnaryEdge<1, double, hello_vertex> {
	// computes the error of the edge and stores it in an internal structure
	//optimizable_graph.h
        virtual void computeError() {
		const hello_vertex* v = static_cast<const hello_vertex*> (_vertices[0]);
		const Eigen::Matrix<double,1,1> est = v->estimate();
		//typedef Eigen::Matrix<double, D, 1, Eigen::ColMajor> ErrorVector;
		//_error = est - (Eigen::Matrix<double, 1, 1> )(10);
		cout << "computeError: " ;
		//_error(0,0) = _measurement - est(0,0);
		//此处误差的定义影响后续解析求导时Jacob矩阵前边是否添加负号
		_error(0,0) = -_measurement + (est(0,0) + 1)*(est(0,0) + 1);
		cout << _error(0,0) << endl;
	}
	
	virtual void linearizeOplus() {
		const hello_vertex* v = static_cast<const hello_vertex*> (_vertices[0]);
		const Eigen::Matrix<double,1,1> est = v->estimate();
		//需要提供负梯度,与Ceres相同
		//之所以是负梯度,因为err = _measurement - f(_estimate)
		//因此为负的
		_jacobianOplusXi(0,0) = 2 * (est(0,0) + 1);
		cout << "_jacobianOplusXi: " << _jacobianOplusXi << endl;
	}
	
	//! read the vertex from a stream, i.e., the internal state of the vertex
        virtual bool read(std::istream& is) {}
        //! write the vertex to a stream
        virtual bool write(std::ostream& os) const {}
};
/*
 *  default_random_engine generator;  
 *  normal_distribution<double> distribution(0.0,0.5);
 *  distribution(generator);
 */



int main(int argc, char** argv)
{

	// 构建图优化，先设定g2o
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<1,1> > Block;  // 每个误差项优化变量维度为3，误差值维度为1
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // 线性方程求解器
	Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
	// 梯度下降方法，从GN, LM, DogLeg 中选
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
	// g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
	// g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
	g2o::SparseOptimizer optimizer;     // 图模型
	optimizer.setAlgorithm( solver );   // 设置求解器
	optimizer.setVerbose( true );       // 打开调试输出

	// 往图中增加顶点
	hello_vertex* v = new hello_vertex();
	v->setEstimate((Eigen::Matrix<double,1,1>) 100);
	v->setId(0);
	optimizer.addVertex( v );

	//加边
	hello_edge* edge = new hello_edge;
	edge->setId(0);
	edge->setVertex( 0, v );                // 设置连接的顶点
	edge->setMeasurement(66.6);      // 观测数值
	edge->setInformation(Eigen::Matrix<double,1,1>::Identity()*0.25); // 信息矩阵：协方差矩阵之逆
	optimizer.addEdge( edge );

	// 执行优化
	cout<<"start optimization"<<endl;
	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	optimizer.initializeOptimization();
	optimizer.optimize(100);
	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
	cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

	// 输出优化值
	Eigen::Matrix<double,1,1> abc_estimate = v->estimate();
	cout<<"estimated value: "<<abc_estimate.transpose()<<endl;
	
	
	
	return 0;
}



