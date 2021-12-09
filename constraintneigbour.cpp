#include "constraintneigbour.h"

#include "vectorhelper.h"

#include <cmath>
#include <iostream>

ConstraintNeigbour::ConstraintNeigbour(const std::vector<double>& point, const size_t &pos, const double& cellSize, const double& lowerSpeed) :
    m_point(point),
    m_cellSize(lowerSpeed + cellSize / 2.0 + lowerSpeed),
    m_otherRobot(m_point),
    m_t0(0.0),
    m_T(0.0),
    m_N(0),
    m_pos(pos)
{

}


double ConstraintNeigbour::operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data) {
    std::vector<std::vector<double> > control2d = VectorHelper::reshapeXd(u, 2);
    std::vector<std::vector<double> > x = m_otherRobot.getTrajectory(m_otherRobot.getState(), control2d, m_t0, m_T, m_N);
    std::vector<double> x2d = x.at(m_pos);
    auto itx2d = x2d.end();
    --itx2d;
    x2d.erase(itx2d, x2d.end());
    double result = (-1.0) * VectorHelper::getInfinityNorm(VectorHelper::sub(x2d, m_point));
    result +=  m_cellSize;
    if (result > 0.01) {
        //std::cout << "Constraint violated: " << result << std::endl;
    }
    return result;
}

/**
 * @brief ConstraintNeigbour::setOtherRobot set the robot, which trajectory should be validated
 * @param other
 */
void ConstraintNeigbour::setData(const Robot& other, const double &t0, const double &T, const size_t &N) {
    m_otherRobot = other;
    m_t0 = t0;
    m_T = T;
    m_N = N;
}

/**
 * @brief ConstraintNeigbour::wrapCostFunctionObject encapsulate for NLOpt the function as function object
 * @param x
 * @param grad
 * @param data
 * @return
 */
double ConstraintNeigbour::wrapCostFunctionObject(const std::vector<double>& u, std::vector<double>& grad, void* data) {
    //return (operator()(x, grad, data));
    return (*reinterpret_cast<ConstraintNeigbour*>(data)) (u, grad);
}
