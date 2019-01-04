#include <iostream>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <chrono>

using namespace std;
using namespace g2o;
//'2'表示顶点中待估计参数个数,数据类型表示这几个待估计变量放在一个顶点中的存储类型
//class g2o_vertex: public BaseVertex<2, Eigen::Matrix<double,1,2>> {
class g2o_vertex: public BaseVertex<4, Eigen::Matrix<double,1,4>> {
	void oplusImpl(const double* v) {
		cout << "oplus: " << Eigen::Matrix<double,1,4>(v) << endl;
		_estimate += Eigen::Matrix<double,1,4>(v);
		cout << "updated _estimate: " << _estimate << endl;
	}
	void setToOriginImpl() {
		_estimate = Eigen::Matrix<double,1,4>(6,6,6,6);
	}
	
	bool read(std::istream& is) {}
        bool write(std::ostream& os) const {}
};
//double定义的是_measurement;_error是Eigen::Matrix类型,维度与_measurement相同,但类型可能不同
// class g2o_edge: public BaseUnaryEdge<4,double,g2o_vertex> {
class g2o_edge1: public BaseUnaryEdge<1,double,g2o_vertex> {
	void computeError() {
		g2o_vertex* v = static_cast<g2o_vertex*>(_vertices[0]);
		Eigen::Matrix<double,1,4> est = v->estimate();
		cout << "est: " << est << endl;
		double f1 = (est(0,0) + 10*est(0,1)), 
			   f2 = sqrt(5)*(est(0,2) - est(0,3)), 
			   f3 = (est(0,1) - 2*est(0,2))*(est(0,1) - 2*est(0,2)),
			   f4 = sqrt(10)*(est(0,0) - est(0,3))*(est(0,0) - est(0,3));
		//cout << "f: " << f1 << "=====" << f2 << "=====" << f3 << "=====" << f4 << endl;
		_error(0,0) = _measurement - f1;
/*		_error(0,1) = _measurement(0,1) - f2;
		_error(0,2) = _measurement(0,2) - f3;
		_error(0,3) = _measurement(0,3) - f4;	*/	
		cout << "_error1: " << _error << endl;
	}
	
	bool read(std::istream& is) {}
        bool write(std::ostream& os) const {}
};

class g2o_edge2: public BaseUnaryEdge<1,double,g2o_vertex> {
	void computeError() {
		g2o_vertex* v = static_cast<g2o_vertex*>(_vertices[0]);
		Eigen::Matrix<double,1,4> est = v->estimate();
		cout << "est: " << est << endl;
		double f1 = (est(0,0) + 10*est(0,1)), 
			   f2 = sqrt(5)*(est(0,2) - est(0,3)), 
			   f3 = (est(0,1) - 2*est(0,2))*(est(0,1) - 2*est(0,2)),
			   f4 = sqrt(10)*(est(0,0) - est(0,3))*(est(0,0) - est(0,3));
		//cout << "f: " << f1 << "=====" << f2 << "=====" << f3 << "=====" << f4 << endl;
		_error(0,0) = _measurement - f2;
		cout << "_error2: " << _error << endl;
	}
	
	bool read(std::istream& is) {}
        bool write(std::ostream& os) const {}
};

class g2o_edge3: public BaseUnaryEdge<1,double,g2o_vertex> {
	void computeError() {
		g2o_vertex* v = static_cast<g2o_vertex*>(_vertices[0]);
		Eigen::Matrix<double,1,4> est = v->estimate();
		cout << "est: " << est << endl;
		double f1 = (est(0,0) + 10*est(0,1)), 
			   f2 = sqrt(5)*(est(0,2) - est(0,3)), 
			   f3 = (est(0,1) - 2*est(0,2))*(est(0,1) - 2*est(0,2)),
			   f4 = sqrt(10)*(est(0,0) - est(0,3))*(est(0,0) - est(0,3));
		//cout << "f: " << f1 << "=====" << f2 << "=====" << f3 << "=====" << f4 << endl;
		_error(0,0) = _measurement - f3;
		cout << "_error3: " << _error << endl;
	}
	
	bool read(std::istream& is) {}
        bool write(std::ostream& os) const {}
};

class g2o_edge4: public BaseUnaryEdge<1,double,g2o_vertex> {
	void computeError() {
		g2o_vertex* v = static_cast<g2o_vertex*>(_vertices[0]);
		Eigen::Matrix<double,1,4> est = v->estimate();
		cout << "est: " << est << endl;
		double f1 = (est(0,0) + 10*est(0,1)), 
			   f2 = sqrt(5)*(est(0,2) - est(0,3)), 
			   f3 = (est(0,1) - 2*est(0,2))*(est(0,1) - 2*est(0,2)),
			   f4 = sqrt(10)*(est(0,0) - est(0,3))*(est(0,0) - est(0,3));
		//cout << "f: " << f1 << "=====" << f2 << "=====" << f3 << "=====" << f4 << endl;
		_error(0,0) = _measurement - f4;
		cout << "_error4: " << _error << endl;
	}
	
	bool read(std::istream& is) {}
        bool write(std::ostream& os) const {}
};

//z = x^2 + 3*y + 1-----x=3 y=4 z=22
int main(int argc, char **argv) 
{
	// 构建图优化，先设定g2o
	//导致出现段错误在维度的设置上
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<4,1> > Block;  // 每个误差项优化变量维度为3，误差值维度为1
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // 线性方程求解器
	Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
	// 梯度下降方法，从GN, LM, DogLeg 中选
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
// 	g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
// 	g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
	g2o::SparseOptimizer optimizer;     // 图模型
	optimizer.setAlgorithm( solver );   // 设置求解器
	optimizer.setVerbose( true );       // 打开调试输出

	// 往图中增加顶点
	g2o_vertex* v = new g2o_vertex();
// 	v->setEstimate((Eigen::Matrix<double,1,2>) (2.5,3.5));
// 	v->setEstimate((Eigen::Matrix<double,1,4>) (21,-sqrt(5),16,9*sqrt(10)));
	v->setEstimate((Eigen::Matrix<double,1,4>) (6,6,6,6));
	v->setId(0);
	optimizer.addVertex( v );

	//加边
	g2o_edge1* edge1 = new g2o_edge1;
	edge1->setId(0);
	edge1->setVertex( 0, v );                // 设置连接的顶点
// 	edge->setMeasurement((Eigen::Matrix<double,1,4>) (1.5+0.5*i,2.5+0.5*i,3.5+0.5*i,4.5+0.5*i));      // 观测数值
	edge1->setMeasurement(21);
	edge1->setInformation(Eigen::Matrix<double,1,1>::Identity()*0.25); // 信息矩阵：协方差矩阵之逆
	optimizer.addEdge( edge1);
	
		//加边
	g2o_edge2* edge2 = new g2o_edge2;
	edge2->setId(0);
	edge2->setVertex( 0, v );                // 设置连接的顶点
	edge2->setMeasurement(-sqrt(5));
	edge2->setInformation(Eigen::Matrix<double,1,1>::Identity()*0.25); // 信息矩阵：协方差矩阵之逆
	optimizer.addEdge( edge2 );
	
		//加边
	g2o_edge3* edge3 = new g2o_edge3;
	edge3->setId(0);
	edge3->setVertex( 0, v );                // 设置连接的顶点
	edge3->setMeasurement(16);
	edge3->setInformation(Eigen::Matrix<double,1,1>::Identity()*0.25); // 信息矩阵：协方差矩阵之逆
	optimizer.addEdge( edge3 );
	
	//加边
	g2o_edge4* edge4 = new g2o_edge4;
	edge4->setId(0);
	edge4->setVertex( 0, v );                // 设置连接的顶点
	edge4->setMeasurement(9*sqrt(10));
	edge4->setInformation(Eigen::Matrix<double,1,1>::Identity()*0.25); // 信息矩阵：协方差矩阵之逆
	optimizer.addEdge( edge4 );

	// 执行优化
	cout<<"start optimization"<<endl;
	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	optimizer.initializeOptimization();
	optimizer.optimize(100);
	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
	cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

	// 输出优化值
	Eigen::Matrix<double,1,4> abc_estimate = v->estimate();
	cout<<"estimated value: "<<abc_estimate.transpose()<<endl;
	
	
	return 0;
}
