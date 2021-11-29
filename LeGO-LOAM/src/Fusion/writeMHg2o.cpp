//
// Created by yewei on 10/31/21.
//
#include "utility.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>
#include <Eigen/Dense>

using namespace gtsam;
using namespace Eigen;

struct MHEdgeStruct{
    int EdgeType;
    Symbol IDFrom;
    vector<Symbol> IDTo;
    bool IsAmbiguity;
    vector<Pose3> Pose;
    MatrixXd NoiseMat;

};

void write_MH_g2o(const string& filename, vector<MHEdgeStruct> edge_list) {
    fstream stream(filename.c_str(), fstream::out);//, std::ios::app);
    Symbol key1, key2;
    int num_ambiguity;
    Pose3 p;
    Rot3 R;
    for (auto& edge : edge_list){
        switch (edge.EdgeType) {
            case 1:
                key1 = edge.IDFrom;
                key2 = edge.IDTo[0];
                p = edge.Pose[0];
                R = p.rotation();
                stream << "EDGE_SE3:QUAT " << key1 << " " << 1 << " "<< key2 << " " << 0 << " " << 1
                       << " " << p.x() << " "  << p.y() << " " << p.z()  << " " << R.toQuaternion().x()
                       << " " << R.toQuaternion().y() << " " << R.toQuaternion().z()  << " " << R.toQuaternion().w();

                for (int i = 0; i < 6; i++){
//                    for (int j = i; j < 6; j++){
                        stream << " " << edge.NoiseMat(i, i);
//                    }
                }
                stream << endl;
                break;
            case 2:
                key1 = edge.IDFrom;
                key2 = edge.IDTo[0];
                num_ambiguity = edge.Pose.size();
                stream << "EDGE_SE3:QUAT " << key1 << " " << 1 << " "<< key2 << " " << 0 << " " << num_ambiguity<< " ";

                for (auto& ite : edge.Pose){
                    p = ite;
                    R = p.rotation();
                    stream << p.x() << " "  << p.y() << " " << p.z()  << " " << R.toQuaternion().x()
                          << " " << R.toQuaternion().y() << " "
                          << R.toQuaternion().z()  << " " << R.toQuaternion().w()<<" ";
                }

                for (int i = 0; i < 6; i++){
//                    for (int j = i; j < 6; j++){
                        stream << " " << edge.NoiseMat(i, i);
//                    }
                }
                stream << endl;
                break;
            case 3:
                key1 = edge.IDFrom;
                num_ambiguity = edge.IDTo.size();
                stream << "EDGE_SE3:QUAT " << key1 << " " << num_ambiguity;
                for (auto& ite : edge.IDTo){
                    key2 = ite;
                    stream << " " << key2 << " ";
                }
                p = edge.Pose[0];
                R = p.rotation();
                stream << edge.IsAmbiguity << " " << p.x() << " "  << p.y() << " " << p.z()  << " " << R.toQuaternion().x()
                       << " " << R.toQuaternion().y() << " " << R.toQuaternion().z()  << " " << R.toQuaternion().w();
                for (int i = 0; i < 6; i++){
//                    for (int j = i; j < 6; j++){
                        stream << " " << edge.NoiseMat(i, i);
//                    }
                }
                stream << endl;

                break;
            case 4:
                key1 = edge.IDFrom;
                key2 = edge.IDTo[0];
                p = edge.Pose[0];
                R = p.rotation();
                stream << "EDGE_SE3:QUAT " << key1 << " " << 1 << " "<< key2 << " " << 1 << " " << edge.IsAmbiguity
                        << " " <<p.x() << " "  << p.y() << " " << p.z()  << " " << R.toQuaternion().x()
                        << " " << R.toQuaternion().y() << " " << R.toQuaternion().z()  << " " << R.toQuaternion().w();

                for (int i = 0; i < 6; i++){
//                    for (int j = i; j < 6; j++){
                        stream << " " << edge.NoiseMat(i, i);
//                    }
                }
                stream << endl;
                break;
            default:
                throw invalid_argument("Only support Type 1, 2, 3 & 4!");
        }

    }
    stream.close();
}