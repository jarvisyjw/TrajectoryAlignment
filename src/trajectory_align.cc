#include "trajectory_align.h"
// Implementation of class methods

std::vector<Sophus::SE3d> TrajectoryAlignment::loadTrajectory(const std::string& file_path) {
    std::vector<Sophus::SE3d> poses;
    std::ifstream file(file_path);

    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + file_path);
    }

    while (!file.eof()) {
        double data[8] = {0};
        for (auto& d : data) {
            file >> d;
        }
        
        Eigen::Vector3d t(data[1], data[2], data[3]);
        Eigen::Quaterniond q(data[7], data[4], data[5], data[6]);
        Sophus::SE3d pose(q, t);
        poses.push_back(pose);
    }
    std::cout << "Loaded " << poses.size() << " poses from file: " << file_path << std::endl;

    return poses;
}

TrajectoryAlignment::UmeyamaResult TrajectoryAlignment::umeyamaAlignment(const Eigen::MatrixXd& x, const Eigen::MatrixXd& y, bool with_scale) {
    checkMatrixDimensions(x, y);

    // Means
    Eigen::VectorXd mean_x = x.rowwise().mean();
    Eigen::VectorXd mean_y = y.rowwise().mean();

    // Variance
    double sigma_x = (x.colwise() - mean_x).squaredNorm() / x.cols();

    // Covariance matrix
    Eigen::MatrixXd cov_xy = Eigen::MatrixXd::Zero(x.rows(), x.rows());
    for (int i = 0; i < x.cols(); ++i) {
        cov_xy += (y.col(i) - mean_y) * (x.col(i) - mean_x).transpose();
    }
    cov_xy /= x.cols();

    // Singular Value Decomposition (SVD)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov_xy, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd u = svd.matrixU();
    Eigen::MatrixXd v = svd.matrixV();

    // S matrix
    Eigen::MatrixXd s = Eigen::MatrixXd::Identity(x.rows(), x.rows());
    if (u.determinant() * v.determinant() < 0.0) {
        s(x.rows() - 1, x.rows() - 1) = -1.0;
    }

    // Rotation matrix
    Eigen::MatrixXd rotation = u * s * v.transpose();

    // Scale and translation
    double scale = with_scale ? (svd.singularValues().dot(s.diagonal()) / sigma_x) : 1.0;
    Eigen::VectorXd translation = mean_y - scale * rotation * mean_x;

    return {rotation, translation, scale};
}

Eigen::MatrixXd TrajectoryAlignment::vectorToMatrix(const std::vector<Eigen::Vector3d>& points) {
    if (points.empty()) {
        throw std::invalid_argument("Input vector is empty.");
    }

    Eigen::MatrixXd matrix(3, points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        matrix.col(i) = points[i];
    }

    return matrix;
}

Sophus::SE3d TrajectoryAlignment::umeyamaToSE3(const UmeyamaResult& result) {
    Eigen::Matrix3d scaled_rotation = result.rotation * result.scale;
    Eigen::Quaterniond q(scaled_rotation);
    return Sophus::SE3d(q, result.translation);
}

double TrajectoryAlignment::computeRMSE(const std::vector<Sophus::SE3d>& poses1, const std::vector<Sophus::SE3d>& poses2) {
    if (poses1.size() != poses2.size()) {
        throw std::invalid_argument("Pose vectors must have the same size.");
    }

    double error_sum = 0.0;
    for (size_t i = 0; i < poses1.size(); ++i) {
        Eigen::Vector3d diff = poses1[i].translation() - poses2[i].translation();
        error_sum += diff.squaredNorm();
    }

    return std::sqrt(error_sum / poses1.size());
}

void TrajectoryAlignment::drawTrajectory(const std::vector<Sophus::SE3d>& poses1, const std::vector<Sophus::SE3d>& poses2) {
    if (poses1.empty() || poses2.empty()) {
        throw std::invalid_argument("Trajectory is empty!");
    }

    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        glColor3f(1.0f, 0.0f, 0.0f); // Red for poses1
        for (size_t i = 0; i < poses1.size() - 1; ++i) {
            glBegin(GL_LINES);
            auto p1 = poses1[i].translation();
            auto p2 = poses1[i + 1].translation();
            glVertex3d(p1[0], p1[1], p1[2]);
            glVertex3d(p2[0], p2[1], p2[2]);
            glEnd();
        }

        glColor3f(0.0f, 0.0f, 1.0f); // Blue for poses2
        for (size_t i = 0; i < poses2.size() - 1; ++i) {
            glBegin(GL_LINES);
            auto p1 = poses2[i].translation();
            auto p2 = poses2[i + 1].translation();
            glVertex3d(p1[0], p1[1], p1[2]);
            glVertex3d(p2[0], p2[1], p2[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000); // Sleep for 5 ms
    }
}

void TrajectoryAlignment::checkMatrixDimensions(const Eigen::MatrixXd& x, const Eigen::MatrixXd& y) {
    if (x.rows() != y.rows() || x.cols() != y.cols()) {
        throw std::invalid_argument("Data matrices must have the same shape.");
    }
}