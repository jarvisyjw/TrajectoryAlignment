#include "trajectory_align.h"
// Main function
int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <groundtruth_file> <estimated_file>" << std::endl;
        return -1;
    }

    std::string groundtruth_file = argv[1];
    std::string estimated_file = argv[2];

    TrajectoryAlignment aligner;

    try {
        // Load trajectories
        auto poses1 = aligner.loadTrajectory(groundtruth_file);
        auto poses2 = aligner.loadTrajectory(estimated_file);

        // Extract translations
        std::vector<Eigen::Vector3d> points1, points2;
        for (const auto& pose : poses1) points1.push_back(pose.translation());
        for (const auto& pose : poses2) points2.push_back(pose.translation());

        // Perform Umeyama alignment
        Eigen::MatrixXd x = aligner.vectorToMatrix(points1);
        Eigen::MatrixXd y = aligner.vectorToMatrix(points2);
        auto result = aligner.umeyamaAlignment(x, y, true);

        // Transform poses2 to align with poses1
        Sophus::SE3d alignment_transform = aligner.umeyamaToSE3(result).inverse();
        std::vector<Sophus::SE3d> poses2_aligned;
        for (const auto& pose : poses2) {
            poses2_aligned.push_back(alignment_transform * pose);
        }

        // Compute RMSE
        double rmse = aligner.computeRMSE(poses1, poses2_aligned);
        std::cout << "RMSE: " << rmse << std::endl;

        // Visualize trajectories
        aligner.drawTrajectory(poses1, poses2_aligned);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
