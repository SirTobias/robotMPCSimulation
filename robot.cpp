#include "robot.h"
#include <math.h>
#include <vector>
#include <iterator>

Robot::Robot(const std::vector<double> &x0) :
    m_x(x0)
{

}

std::vector<double> Robot::getState() const {
    return m_x;
}

/**
 * @brief Robot::systemStep returns the system step for the robot
 * @param x
 * @param u
 * @param t0
 * @param tEnd
 * @return
 */
std::vector<double> Robot::systemStep(const std::vector<double>& x0, const std::vector<double>& u, const double& t0, const double&tEnd) const {
    std::vector<double> ret;
    ret.reserve(3);
    //non-holonomic
    ret.push_back(x0.at(0) + (tEnd - t0) * std::cos(x0.at(2)) * u.at(0));
    ret.push_back(x0.at(1) + (tEnd - t0) * std::sin(x0.at(2)) * u.at(0));
    ret.push_back(x0.at(2) + (tEnd - t0) * u.at(1));
    //holonomic
    /*ret.push_back(x0.at(0) + (tEnd - t0) * u.at(0));
    ret.push_back(x0.at(1) + (tEnd - t0) * u.at(1));
    ret.push_back(0.0);*/
    return ret;
}

/**
 * @brief Robot::getTrajectory
 * @param x0
 * @param u
 * @param t0
 * @param T
 * @param N
 * @return
 */
std::vector<std::vector<double> > Robot::getTrajectory(const std::vector<double>& x0, const std::vector<std::vector<double> >& u, const double& t0, const double& T, const size_t& N) const {
    std::vector<std::vector<double> > x;
    x.reserve(N+1);
    x.push_back(x0);
    for (auto i = 0; i < N; ++i) {
        x.push_back(systemStep(x.at(i), u.at(i), t0 + (double)(i-1)*T, t0 + (double)i*T));
    }
    return x;
}

/**
 * @brief Robot::shiftState
 * @param x0
 * @param u0
 * @param t0
 * @param T
 * @return
 */
std::vector<std::vector<double> > Robot::shiftState(const std::vector<double>& x0, const std::vector<std::vector<double> >& u0, const double& t0, const double& T) {
    std::vector<std::vector<double> > ret;
    m_x = systemStep(x0, u0.at(0), t0, t0 + T);

    return ret;
}

/**
 * @brief Robot::shiftControl
 * @param x0
 * @param u0
 * @param t0
 * @param T
 * @return
 */
std::vector<std::vector<double> > Robot::shiftControl(const std::vector<double>& x0, const std::vector<std::vector<double> > &u0, const double& t0, const double& T) {
    std::vector<std::vector<double> > u = u0;
    auto toErase = u.begin();
    ++toErase;
    u.erase(u.begin(), toErase);
    std::vector<double> last = u.back();
    std::fill_n(std::begin(last), last.size(), 0.0);
    u.push_back(last);
    return u;
}
