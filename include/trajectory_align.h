#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <pangolin/pangolin.h>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <unistd.h>

class TrajectoryAlignment {
public:
    struct UmeyamaResult {
        Eigen::MatrixXd rotation;  // Rotation matrix
        Eigen::VectorXd translation;  // Translation vector
        double scale;  // Scale factor
    };

    TrajectoryAlignment() = default;

    // Load trajectory from file
    std::vector<Sophus::SE3d> loadTrajectory(const std::string& file_path);

    // Perform Umeyama alignment
    UmeyamaResult umeyamaAlignment(const Eigen::MatrixXd& x, const Eigen::MatrixXd& y, bool with_scale = false);

    // Convert vector of Eigen::Vector3d to Eigen::MatrixXd
    Eigen::MatrixXd vectorToMatrix(const std::vector<Eigen::Vector3d>& points);

    // Convert Umeyama result to SE3 transformation
    Sophus::SE3d umeyamaToSE3(const UmeyamaResult& result);

    // Compute RMSE between two sets of poses
    double computeRMSE(const std::vector<Sophus::SE3d>& poses1, const std::vector<Sophus::SE3d>& poses2);

    // Visualize the trajectory
    void drawTrajectory(const std::vector<Sophus::SE3d>& poses1, const std::vector<Sophus::SE3d>& poses2);

private:
    void checkMatrixDimensions(const Eigen::MatrixXd& x, const Eigen::MatrixXd& y);
};
