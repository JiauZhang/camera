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

class hello_vertex: public BaseVertex<1, double> {
          /**
         * update the position of the node from the parameters in v.
         * Implement in your class!
         */
	//optimizable_graph.h  
        virtual void oplusImpl(const double* v) {
		_estimate += *v;
	}
	
        //! sets the node to the origin (used in the multilevel stuff)
        //optimizable_graph.h
        virtual void setToOriginImpl() {
		_estimate = 0;
	}
	
	//! read the vertex from a stream, i.e., the internal state of the vertex
        virtual bool read(std::istream& is) {}
        //! write the vertex to a stream
        virtual bool write(std::ostream& os) const {}
};

class hello_edge: public BaseUnaryEdge<1, double, double> {
	// computes the error of the edge and stores it in an internal structure
	//optimizable_graph.h
        virtual void computeError() {
		_error = double(10) - _measurement;
	}
	
	//! read the vertex from a stream, i.e., the internal state of the vertex
//         virtual bool read(std::istream& is) {}
        //! write the vertex to a stream
//         virtual bool write(std::ostream& os) const {}
};
/*
 *  default_random_engine generator;  
 *  normal_distribution<double> distribution(0.0,0.5);
 *  distribution(generator);
 */



int main(int argc, char** argv)
{
	double a=1.0, b=2.0, c=1.0;         // 真实参数值
	int N=100;                          // 数据点
	double w_sigma=1.0;                 // 噪声Sigma值
	cv::RNG rng;                        // OpenCV随机数产生器
	double abc[3] = {0,0,0};            // abc参数的估计值

	vector<double> x_data, y_data;      // 数据

	cout<<"generating data: "<<endl;


	// 构建图优化，先设定g2o
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;  // 每个误差项优化变量维度为3，误差值维度为1
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
	CurveFittingVertex* v = new hello_vertex();
	v->setEstimate( 0);
	v->setId(0);
	optimizer.addVertex( v );

	//加边
	CurveFittingEdge* edge = new hello_edge();
	edge->setId(0);
	edge->setVertex( 0, v );                // 设置连接的顶点
	edge->setMeasurement(0);      // 观测数值
	edge->setInformation(1); // 信息矩阵：协方差矩阵之逆
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
	Eigen::Vector3d abc_estimate = v->estimate();
	cout<<"estimated model: "<<abc_estimate.transpose()<<endl;
	
	
	
	return 0;
}



