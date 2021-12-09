#ifndef CONSTRAINTNEIGBOUR_H
#define CONSTRAINTNEIGBOUR_H

#include "robot.h"

#include <vector>

class ConstraintNeigbour
{
public:
    ConstraintNeigbour(const std::vector<double>& point, const size_t &pos, const double& cellSize, const double& lowerSpeed);
    double operator()(const std::vector<double> &u, std::vector<double> &grad, void* f_data = nullptr);
    static double wrapCostFunctionObject(const std::vector<double>& u, std::vector<double>& grad, void* data);
    void setData(const Robot& other, const double& t0, const double& T, const size_t& N);
private:
    std::vector<double> m_point;
    double m_cellSize;
    Robot m_otherRobot;
    double m_t0;
    double m_T;
    size_t m_N;
    size_t m_pos;
};

#endif // CONSTRAINTNEIGBOUR_H
