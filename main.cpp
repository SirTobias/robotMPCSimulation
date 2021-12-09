#include "robot.h"
#include "costfunction.h"
#include "constraintneigbour.h"

#include "vectorhelper.h"
#include <nlopt.hpp>

#include <QtCore/QTextStream>
#include <QtCore/QFile>

#include <cmath>
#include <vector>
#include <iostream>

int main(int argc, char *argv[])
{
    size_t N = 12;
    //std::string strInput;
    //std::cin >> strInput;
    if (argc > 1) {
        N = std::atoi(argv[1]);
        std::cout << "N: " << N << "\n";
        Q_ASSERT_X(N != 0, "ERROR: ", "horizon N should be > 0");
    }
    double t0 = 0.0;
    double T = 1.0;
    Robot robot(std::vector<double>{0.01, 0.0, M_PI / 4.0});
    std::vector<double> target = {-4.01, 0.0, 0.0};
    CostFunction costs(robot, target);
    costs.setData(robot.getState(), t0, T, N);
    std::vector<double> controlLowerBound;
    std::vector<double> controlUpperBound;
    for (size_t i = 0; i < N; ++i) {
        controlLowerBound.emplace_back(-0.5);
        controlLowerBound.emplace_back(-0.5);
        controlUpperBound.emplace_back(0.5);
        controlUpperBound.emplace_back(0.5);
    }

    nlopt::opt optProblem(nlopt::LN_COBYLA, N*2);
    optProblem.set_lower_bounds(controlLowerBound);
    optProblem.set_upper_bounds(controlUpperBound);
    std::vector<ConstraintNeigbour> constraints;
    for (size_t i = 0; i < N; ++i) {
        ConstraintNeigbour constraintQ({-2.0, 0.0}, i, 2.0, 0.5);
        constraintQ.setData(robot, t0, T, N);
        constraints.push_back(constraintQ);
    }
    for (size_t i = 0; i < N; ++i) {
        optProblem.add_inequality_constraint(ConstraintNeigbour::wrapCostFunctionObject, &constraints.at(i), 0.0001);
    }
    optProblem.set_min_objective(CostFunction::wrapCostFunctionObject, &costs);
    std::vector<double> initControl;
    /*for (size_t i = 0; i < N; ++i) {
        initControl.emplace_back(0.0);
        initControl.emplace_back(0.0);
    }*/
    double optVal = 0.0, optValWoLambda = 0.0;
    nlopt::result ret = nlopt::SUCCESS;
    //optProblem.set_ftol_rel(0.000001);
    //optProblem.set_xtol_rel(0.0001);
    optProblem.set_xtol_abs(0.001);
    //optProblem.set_ftol_abs(0.00001);

    //std::vector<double> initStep;
    //non-holonomic
    for (int i = 0; i < N; ++i) {
        if (i < 2) {
            initControl.push_back(0.4);
            initControl.push_back(0.4);
        }
        else if (i >= 3 && i < 6) {
            initControl.push_back(0.5);
            initControl.push_back(0.45);
        }
        else {
            initControl.push_back(0.5);
            initControl.push_back(0.05);
        }
    }
    //holonomic
    /*for (int i = 0; i < N; ++i) {
        if (i < 4) {
            initControl.push_back(0.0);
            initControl.push_back(0.5);
        }
        else if (i >= 4 && i < 14) {
            initControl.push_back(-0.5);
            initControl.push_back(0.0);
        }
        else {
            initControl.push_back(0.0);
            initControl.push_back(-0.5);
        }
    }*/
    //test direct distance
    std::vector<double> uZero;
    for (size_t i = 0; i < N*2; ++i) {
        uZero.emplace_back(0.0);
    }
    std::cout << "do nothing: " << costs.getObjective(robot.getState(), uZero, t0, T, N, 0.2) << "\n";
    std::cout << "do init (lambda = 0.2): " << costs.getObjective(robot.getState(), initControl, 0.0, T, N, 0.2) << std::endl;
    std::cout << "do init (lambda = 0): " << costs.getObjective(robot.getState(), initControl, 0.0, T, N, 0.0) << std::endl;
    std::vector<std::vector<double> > firstPred = robot.getTrajectory(robot.getState(), VectorHelper::reshapeXd(initControl, 2), t0, T, N);
    for (size_t i = 0; i < N; ++i) {
        std::cout << "time: " << i << " (" << firstPred.at(i).at(0) << ", " << firstPred.at(i).at(1) << ", " << firstPred.at(i).at(2) << ")\n";
    }
    std::cout << std::endl;
    //optProblem.set_initial_step(initStep);
    QFile robotTrajectFile("robotHolonomTrajectoryHoriz" + QString::number(N) + ".txt");
    QFile robotTrajectOpenLoop("robotOpenLoopTrajecHoriz" + QString::number(N) + ".txt");
    QFile robotOpenLoopCostsFile("robotOpenLoopCostsHoriz" + QString::number(N) + ".txt");
    QFile robotClosedLoopCostsFile("robotClosedLoopCostsHoriz" + QString::number(N) + ".txt");
    robotTrajectFile.open(QIODevice::WriteOnly);
    robotTrajectOpenLoop.open(QIODevice::WriteOnly);
    robotOpenLoopCostsFile.open(QIODevice::WriteOnly);
    robotClosedLoopCostsFile.open(QIODevice::WriteOnly);
    QTextStream robotStream(&robotTrajectFile);
    QTextStream robotOpenLoopStream(&robotTrajectOpenLoop);
    QTextStream openLoopStream(&robotOpenLoopCostsFile);
    QTextStream closedLoopStream(&robotClosedLoopCostsFile);
    //headlines
    robotStream << "time \t (x(0)) \t x(1) \t x(2) \t u(0) \t u(1)\n";
    robotOpenLoopStream << "time \t (x(0)) \t x(1) \t x(2) \t u(0) \t u(1)\n";
    openLoopStream << "time \t Open-Loop-Costs (w/o Control) \t Open-Loop-Costs (with Control, lambda = 0.2) \t Open-Loop-Costs (with Control, lambda = 0)\n";
    closedLoopStream << "time \t Closed-Loop-Costs (lambda = 0.2) \t Closed-Loop-Costs (lambda = 0.0) \n";
    //--headlines
    size_t count = 0;

    while (VectorHelper::norm2(VectorHelper::sub(robot.getState(), target)) > 0.001 && count < 30) {
        costs.setLambda(0.2);
        auto initControlWoLambda = initControl;
        try {ret = optProblem.optimize(initControl, optVal);}
        catch (nlopt::roundoff_limited) {
            std::cout << "nlopt::roundOff\n";
        }

        costs.setLambda(0.0);
        try {ret = optProblem.optimize(initControlWoLambda, optValWoLambda);}
        catch (nlopt::roundoff_limited) {
            std::cout << "nlopt::roundOff\n";
        }
        std::vector<std::vector<double> > u2d = VectorHelper::reshapeXd(initControl, 2);
        std::vector<std::vector<double> > xPred= robot.getTrajectory(robot.getState(), u2d, t0, T, N);
        for (size_t i = 0; i < N; ++i) {
            std::vector<double> grad;
            //validate constraints
            double ret = constraints.at(i).operator ()(initControl, grad);
            if (ret > 0.0001) {
                std::cout << "constraint violated: control: at pos " << i
                          << " with control (" << u2d.at(i).at(0) << "," << u2d.at(i).at(1) << ")"
                          << " results to " << ret << std::endl;
            }
        }
        //robotStream << count << "\t" << xPred.front().at(0) << "\t" << xPred.front().at(1) << "\t" << xPred.front().at(2) << "\t" << initControl.at(0) << "\t" << initControl.at(1) << "\n";
        openLoopStream << count << "\t" << costs.getObjective(robot.getState(), uZero, t0, T, N, 0.2) << "\t" << optVal << "\t" << optValWoLambda << "\n";
        closedLoopStream << count << "\t" << costs.getObjective(xPred.at(0), initControl, t0, T, N, 0.2) << "\t" << costs.getObjective(xPred.at(0), initControlWoLambda, t0, T, N, 0.0) << "\n";
        std::cout << "Trajectory: ";
        //size_t openLoopCount = count;
        robotStream << count << "\t" << xPred.at(0).at(0) << "\t " << xPred.at(0).at(1) << "\t " << xPred.at(0).at(2) << "\t"
                    <<  u2d.at(0).at(0) << "\t" << u2d.at(0).at(1) << "\n";
        for (size_t i = 0; i < N; ++i) {
            std::cout << "(" << xPred.at(i).at(0) << ", " << xPred.at(i).at(1) << ", " << xPred.at(i).at(2) << ")\n";
            robotOpenLoopStream << i << "\t" << xPred.at(i).at(0) << "\t " << xPred.at(i).at(1) << "\t " << xPred.at(i).at(2) << "\t"
                        <<  u2d.at(i).at(0) << "\t" << u2d.at(i).at(1) << "\n";
        }
        std::cout << std::endl;
        robot.shiftState(robot.getState(), u2d, t0, T);
        initControl = VectorHelper::reshapeXdTo1d(robot.shiftControl(robot.getState(), u2d, t0, T));
        t0 += T;
        costs.setData(robot.getState(), t0, T, N);
        for (size_t i = 0; i < N; ++i) {
            constraints.at(i).setData(robot, t0, T, N);
        }
        ++count;
        robotClosedLoopCostsFile.flush();
        robotOpenLoopCostsFile.flush();
        robotTrajectFile.flush();

        /*robotClosedLoopCostsFile.close();
        robotOpenLoopCostsFile.close();
        robotTrajectFile.close();
        return 0;*/
    }
    robotClosedLoopCostsFile.close();
    robotOpenLoopCostsFile.close();
    robotTrajectFile.close();
    robotTrajectOpenLoop.close();
    return 0;
}
