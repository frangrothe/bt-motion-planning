//
// Created by francesco on 12.05.21.
//

#include "Time1D.h"

namespace tb1d {

// (1a) State Validity Checker -> Is element feasible?
bool isStateValid(const ob::State *state) {
    return true;
}

// (1b) Motion Validator -> Is transition from element to element feasible.
class MyMotionValidator : public ob::MotionValidator
{
private:
    double _maxSpeed = 1.0; // 1 m/s

public:
    explicit MyMotionValidator(ompl::base::SpaceInformation *si) : MotionValidator(si) {}
    explicit MyMotionValidator(const ompl::base::SpaceInformationPtr &si) : MotionValidator(si) {}

    // implement checkMotion()
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override {

        auto deltaX = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0]
                          - s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];

        auto deltaT = s2->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position
                          - s1->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

        if (deltaT > 0 && deltaX > 0 && deltaX / deltaT <= _maxSpeed) return true;

        return false;
    }

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                     std::pair<ompl::base::State *, double> &lastValid) const override {
        return checkMotion(s1, s2);
    }
};

class MyGoal : public ompl::base::Goal
{
private:
    double _xGoal = 1.0; // default value for the goal
    double _epsilon = 1e-3; // error margin
public:
    explicit MyGoal(const ompl::base::SpaceInformationPtr &si) : Goal(si) {}
    MyGoal(const ompl::base::SpaceInformationPtr &si, double xGoal) : Goal(si), _xGoal(xGoal) {}

    bool isSatisfied(const ompl::base::State *st) const override {
        auto x = st->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];

//        if (abs(_xGoal - x) <= _epsilon) return true;
        if (x >= _xGoal) return true;
        return false;
    }


};

void plan(const std::string& filename) {
    /*
     * (1) OMPL: State space creation
     * State-Time Space
     *
     * space[0] : x-coordinate
     * space[0] : time
     */

    auto r1State(std::make_shared<ob::RealVectorStateSpace>(1));
    auto timeState(std::make_shared<ob::TimeStateSpace>());
    auto space = r1State + timeState;

    // Set the bounds for R1
    ob::RealVectorBounds bounds(1);
    bounds.setLow(0);
    bounds.setHigh(2);
    r1State->setBounds(bounds);

    // Set the bounds for Time
    timeState->setBounds(0.0, 2.0);

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);
//    auto si(std::make_shared<ob::SpaceInformation>(space));
    si->setStateValidityChecker(isStateValid);
    si->setMotionValidator(std::make_shared<MyMotionValidator>(si));

    //(2) Problem Definition Ptr
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    ob::ScopedState<> start(space);
    start[0] = 0.0; // r1State
//    ob::ScopedState<> goal(space);
//    goal[0] = 1.0; // r1State

    ob::GoalPtr goal = std::make_shared<MyGoal>(si, 1.0);

//    pdef->setStartAndGoalStates(start, goal);
    pdef->addStartState(start);
    pdef->setGoal(goal);

    //(3) Planner
    auto planner(std::make_shared<og::RRT>(si));
    planner->setGoalBias(0.05); // with 5 % probability the goal will be sampled
    planner->setProblemDefinition(pdef);
    planner->setup();

    //(4) Planner executes
    ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);

    if (solved)
    {
        ob::PathPtr path = pdef->getSolutionPath();

        ob::PlannerData data(si);
        planner->getPlannerData(data);

//        for (int i = 0; i < data.numVertices(); ++i) {
//            std::cout << "\nVertex " << i << " --  x: " <<
//            data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0] <<
//            ", t: " << data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position << std::endl;
//
//            // Get incoming edges for node
//            std::vector< unsigned int > vertexIndices{};
//            data.getIncomingEdges(i, vertexIndices);
//            std::stringstream ss;
//            for (auto v : vertexIndices) {
//                ss << v << ", ";
//            }
//            std::string s = ss.str();
//            std::cout << "Incoming edges: " << s << std::endl;
//
//        }


        std::cout << "\nFound solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);

        std::cout << "\nNumber of Samples: " << data.numVertices();
        std::cout << "\nExact Solution: " << (pdef->hasExactSolution()? "yes" : "no");
        std::cout << "\nApproximate Solution: " << (pdef->hasApproximateSolution()? "yes" : "no");

        writeToCSV(data, filename);
    }
    else
        std::cout << "No solution found" << std::endl;
}

//
void writeToCSV(const ob::PlannerData &data, const std::string& filename) {
    std::string delim = ",";

    std::ofstream outfile ("data/" + filename + ".csv");

    outfile << "x" << delim << "time" << delim << "incoming edge" << delim << "outgoing edges\n";

    for (int i = 0; i < data.numVertices(); ++i) {
        double x = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double t = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;


        // Get incoming edges for node
        std::vector< unsigned int > inEdgeIndexes{};
        data.getIncomingEdges(i, inEdgeIndexes);
        long int inEdge = inEdgeIndexes.empty()? -1 : (long int) inEdgeIndexes.at(0);

        // Get outgoing edges for node
        std::vector< unsigned int > outEdgeIndexes{};
        data.getEdges(i, outEdgeIndexes);
        std::stringstream ss;
        for (int j = 0; j < outEdgeIndexes.size(); ++j) {
            if (j < outEdgeIndexes.size() - 1) {
                ss << outEdgeIndexes.at(j) << "#";
            }
            else {
                ss << outEdgeIndexes.at(j);
            }
        }
        std::string outEdges = outEdgeIndexes.empty()? "-1" : ss.str();

        // write node data to csv
        outfile << x << delim << t << delim << inEdge << delim << outEdges << "\n";

    }

    outfile.close();
}

}


