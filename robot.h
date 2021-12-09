#ifndef ROBOT_H
#define ROBOT_H

#include <vector>

class Robot
{
public:
    Robot(const std::vector<double>& x0);
    std::vector<double> systemStep(const std::vector<double>& x0, const std::vector<double>& u, const double& t0, const double&tEnd) const;
    std::vector<std::vector<double> > getTrajectory(const std::vector<double>& x0, const std::vector<std::vector<double> >& u, const double& t0, const double& T, const size_t& N) const;
    std::vector<std::vector<double> > shiftState(const std::vector<double>& x0, const std::vector<std::vector<double> > &u0, const double& t0, const double& T);
    std::vector<std::vector<double> > shiftControl(const std::vector<double>& x0, const std::vector<std::vector<double> > &u0, const double& t0, const double& T);
    std::vector<double> getState() const;
private:
    std::vector<double> m_x;
};

#endif // ROBOT_H
