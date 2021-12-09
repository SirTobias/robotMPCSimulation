#ifndef COSTFUNCTION_H
#define COSTFUNCTION_H

#include "robot.h"

#include <vector>

class CostFunction
{
public:
    CostFunction(const Robot& robot, const std::vector<double>& target);
    double getStagecosts(const std::vector<double>& x, const std::vector<double>& u, const double &lambda);
    double getObjective(const std::vector<double>& x, const std::vector<double>& u, const double& t0, const double& T, const size_t& N, const double& lambda);
    double operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data = nullptr);
    static double wrapCostFunctionObject(const std::vector<double>& u, std::vector<double>& grad, void* data);
    void setData(const std::vector<double> &x0, const double& t0, const double &T, const size_t& N);
    void setLambda(const double& lambda);
private:
    std::vector<double> m_target;
    Robot m_robot;
    std::vector<double> m_x0;
    double m_t0;
    double m_T;
    size_t m_N;
    double m_lambda;
};

#endif // COSTFUNCTION_H
