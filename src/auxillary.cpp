//
// Created by francesco on 17.05.21.
//

#include <fstream>
#include "auxillary.h"

namespace auxillary {

std::string currentDateTime() {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m__%H-%M-%S");
    return oss.str();

}

void writeSamplesToCSV1D(const ob::PlannerData &data, const std::string& filename) {

    std::ofstream outfile ("data/" + filename + "/samples.csv");

    outfile << "x" << delim_ << "time" << delim_ << "incoming edge" << delim_ << "outgoing edges\n";

    for (int i = 0; i < data.numVertices(); ++i) {
        double x = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double t = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;


        // Get incoming edges for node
        std::vector< unsigned int > inEdgeIndexes{};
        data.getIncomingEdges(i, inEdgeIndexes);
        std::stringstream ssIn;
        for (int j = 0; j < inEdgeIndexes.size(); ++j) {
            if (j < inEdgeIndexes.size() - 1) {
                ssIn << inEdgeIndexes.at(j) << "#";
            }
            else {
                ssIn << inEdgeIndexes.at(j);
            }
        }
        std::string inEdges = inEdgeIndexes.empty()? "-1" : ssIn.str();

        // Get outgoing edges for node
        std::vector< unsigned int > outEdgeIndexes{};
        data.getEdges(i, outEdgeIndexes);
        std::stringstream ssOut;
        for (int j = 0; j < outEdgeIndexes.size(); ++j) {
            if (j < outEdgeIndexes.size() - 1) {
                ssOut << outEdgeIndexes.at(j) << "#";
            }
            else {
                ssOut << outEdgeIndexes.at(j);
            }
        }
        std::string outEdges = outEdgeIndexes.empty()? "-1" : ssOut.str();

        // write node data to csv
        outfile << x << delim_ << t << delim_ << inEdges << delim_ << outEdges << "\n";

    }

    outfile.close();
}

void writeSamplesToCSV2D(const ob::PlannerData &data, const std::string &filename) {
    std::ofstream outfile ("data/" + filename + "/samples.csv");

    outfile << "x" << delim_ << "y" << delim_ << "time" << delim_ << "incoming edge" << delim_ << "outgoing edges\n";

    for (int i = 0; i < data.numVertices(); ++i) {
        double x = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double y = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        double t = data.getVertex(i).getState()->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;


        // Get incoming edges for node
        std::vector< unsigned int > inEdgeIndexes{};
        data.getIncomingEdges(i, inEdgeIndexes);
        std::stringstream ssIn;
        for (auto edgeIndex : inEdgeIndexes) {
            ssIn << "#" << edgeIndex;
        }
        std::string inEdges = inEdgeIndexes.empty()? "#" : ssIn.str();

        // Get outgoing edges for node
        std::vector< unsigned int > outEdgeIndexes{};
        data.getEdges(i, outEdgeIndexes);
        std::stringstream ssOut;
        for (auto edgeIndex : outEdgeIndexes) {
            ssOut << "#" << edgeIndex;
        }
        std::string outEdges = outEdgeIndexes.empty()? "#" : ssOut.str();

        // write node data to csv
        outfile << x << delim_ << y << delim_ << t << delim_ << inEdges << delim_ << outEdges << "\n";

    }

    outfile.close();
}

void writePathToCSV1D(const ompl::base::PathPtr &pathPointer, const std::string &filename) {

    std::ofstream outfile ("data/" + filename + "/path.csv");

    outfile << "x" << delim_ << "time\n";

    auto path = pathPointer->as<og::PathGeometric>();
    for (auto state : path->getStates()) {
        double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        outfile << x << delim_ << t << "\n";
    }

    outfile.close();
}

void writePathToCSV2D(const ompl::base::PathPtr &pathPointer, const std::string &filename) {

    std::ofstream outfile ("data/" + filename + "/path.csv");

    outfile << "x" << delim_ << "y" << delim_ << "time\n";

    auto path = pathPointer->as<og::PathGeometric>();
    for (auto state : path->getStates()) {
        double x = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
        double y = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[1];
        double t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;
        outfile << x << delim_ << y << delim_ << t << "\n";
    }

    outfile.close();
}

void writeConstraintsToCSV(const std::vector<Constraint>& constraints, const std::string &filename) {

    std::ofstream outfile ("data/" + filename + "/constraints.csv");

    outfile << "x lb" << delim_ << "x ub" << delim_ << "t lb" << delim_ << "t ub\n";
    for (auto c : constraints) {
        outfile << c.x_lb << delim_ << c.x_ub << delim_ << c.t_lb << delim_ << c.t_ub << "\n";
    }

    outfile.close();
}

void writeGoalRegionToCSV1D(double xLb, double xUb, double tLb, double tUb, const std::string &filename) {

    std::ofstream outfile ("data/" + filename + "/goal.csv");

    outfile << "x lb" << delim_ << "x ub" << delim_ << "t lb" << delim_ << "t ub\n";
    outfile << xLb << delim_ << xUb << delim_ << tLb << delim_ << tUb << "\n";

    outfile.close();
}

void writeGoalToCSV2D(double x, double y, double tLow, double tHigh, const std::string &filename) {
    std::ofstream outfile ("data/" + filename + "/goal.csv");

    outfile << "x" << delim_ << "y" << delim_ << "tLow" << delim_ << "tHigh\n";
    outfile << x << delim_ << y << delim_ << tLow << delim_ << tHigh << "\n";

    outfile.close();
}

void writeConstraintsToJSON(const std::vector<Constraint2D> &constraints, const std::string &filename) {

    using json = nlohmann::json;
    json j;

    int n = 20; // number of time samples to generate planes for visualization
    double tLow = 0.0;
    double tHigh = 2.5;

    double tDiff = (tHigh - tLow) / (n - 1);
    std::vector<double> ts(n);
    std::generate(ts.begin(), ts.end(), [i = 0, &tLow, &tDiff] () mutable { return tLow + i++ * tDiff; });

    std::vector<std::vector<std::vector<std::vector<double>>>> jsonVector;
    for (const auto &c : constraints) {
        std::vector<std::vector<std::vector<double>>> cVector;
        for (auto t : ts) {
            double xlb, xub, ylb, yub;
            std::tie(xlb, xub, ylb, yub) = c.getBoundsForTime(t);
            std::vector<std::vector<double>> v {
                    {xlb, yub, t},
                    {xlb, ylb, t},
                    {xub, ylb, t},
                    {xub, yub, t}
            };
            cVector.push_back(v);
        }
        jsonVector.push_back(cVector);
    }

    j = jsonVector;
    // write prettified JSON to another file
    std::ofstream outfile("data/" + filename + "/constraints.json");
    outfile << std::setw(4) << j << std::endl;

}




}


