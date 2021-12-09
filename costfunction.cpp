#include "costfunction.h"
#include "vectorhelper.h"

#include <math.h>

CostFunction::CostFunction(const Robot &robot, const std::vector<double> &target) :
    m_target(target),
    m_robot(robot),
    m_x0(std::vector<double>()),
    m_t0(0.0),
    m_T(0.0),
    m_N(0),
    m_lambda(0.0)
{

}

/**
 * @brief CostFunction::getStagecosts
 * @param x
 * @param u
 * @return
 */
double CostFunction::getStagecosts(const std::vector<double>& x, const std::vector<double>& u, const double& lambda) {
    std::vector<double> costs;
    costs.reserve(3);
    costs.push_back(std::pow(x.at(0) - m_target.at(0), 2.0));
    costs.push_back(5.0 * (x.at(1) - m_target.at(1)));
    costs.push_back(std::pow(x.at(2) - m_target.at(2), 2.0));
    //try exp costs
    //costs.push_back(std::exp(x.at(0) - m_target.at(0)/*, 2.0*/));
    //costs.push_back(std::exp(x.at(1) - m_target.at(1)));
    //costs.push_back(std::exp(x.at(2) - m_target.at(2)/*, 2.0*/));

    double costsSquared = 0.0;
    /*for (auto i = 0; i < costs.size(); ++i) {
        costsSquared += std::pow(costs.at(i), 2.0);
    }*/
    costsSquared = VectorHelper::norm2(costs);
    std::vector<double> uPow = u;
    for (auto i = 0; i < uPow.size(); ++i) {
        uPow[i] = std::pow(uPow.at(i), 2);
    }
    double costsControl = VectorHelper::norm2(uPow);
    costsSquared += lambda * costsControl;
    return costsSquared;
}

/**
 * @brief CostFunction::getObjective
 * @param x
 * @param u
 * @param t0
 * @param T
 * @param N
 * @return
 */
double CostFunction::getObjective(const std::vector<double>& x0, const std::vector<double>& u, const double& t0, const double& T, const size_t& N, const double &lambda) {
   std::vector<std::vector<double> > control2d = VectorHelper::reshapeXd(u, 2);
   std::vector<std::vector<double> > x = m_robot.getTrajectory(x0, control2d, t0, T, N);
   double costs = 0.0;
   for (size_t i = 0; i < N; ++i) {
       costs += getStagecosts(x.at(i), control2d.at(i), lambda);
   }
   return costs;
}

void CostFunction::setData(const std::vector<double>& x0, const double& t0, const double& T, const size_t& N) {
    m_x0 = x0;
    m_t0 = t0;
    m_T = T;
    m_N = N;
}

/**
 * @brief CostFunction::wrapCostFunctionObject encapsulate for NLOpt the function as function object
 * @param x
 * @param grad
 * @param data
 * @return
 */
double CostFunction::wrapCostFunctionObject(const std::vector<double>& u, std::vector<double>& grad, void* data) {
    //return (operator()(x, grad, data));
    return (*reinterpret_cast<CostFunction*>(data)) (u, grad);
}

double CostFunction::operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data) {
    return getObjective(m_x0, u, m_t0, m_T, m_N, m_lambda);
}


void CostFunction::setLambda(const double& lambda) {
    m_lambda = lambda;
}
