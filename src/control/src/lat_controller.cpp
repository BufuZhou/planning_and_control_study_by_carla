// copyright
#include "control/lat_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

#include "common_msgs/msg/pose.hpp"
#include "control/digital_filter_coefficients.hpp"
#include "rclcpp/logging.hpp"

namespace control {

using Matrix = Eigen::MatrixXd;
#define PI 3.141592653589793

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  strftime(name_buffer, 80, "/tmp/steer_log_simple_optimal_%F_%H%M%S.csv",
           localtime(&raw_time));
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream &file_stream) {
  file_stream << "ref_lateral_distance,"
              << "lateral_distance,"
              << "lateral_error,"
              << "ref_heading,"
              << "heading,"
              << "heading_error,"
              << "heading_error_rate,"
              << "lateral_error_rate,"
              << "curvature,"
              << "steer_angle,"
              << "steer_angle_feedforward,"
              << "steer_angle_lateral_contribution,"
              << "steer_angle_lateral_rate_contribution,"
              << "steer_angle_heading_contribution,"
              << "steer_angle_heading_rate_contribution,"
              << "steer_angle_feedback,"
              << "steer_position"
              << "v" << std::endl;
}
}  // namespace

LatController::LatController() : name_("LQR-based Lateral Controller") {
  std::cout << "lateral controller: " << name_ << std::endl;
  FLAGS_enable_csv_debug = true;
  if (FLAGS_enable_csv_debug) {
    steer_log_file_.open(GetLogFileName());
    steer_log_file_ << std::fixed;
    steer_log_file_ << std::setprecision(6);
    WriteHeaders(steer_log_file_);
  }
  std::cout << "Using " << name_ << std::endl;
}

LatController::~LatController() { CloseLogFile(); }

void LatController::LoadControlConfig() {
  std::cout << "load lateral controller config..." << std::endl;
  // vehicle parameters
  ts_ = 0.02;
  wheelbase_ = 2.85;
  cf_ = 166208;
  cr_ = 166208;
  double mass_fl = 424.0;
  double mass_fr = 424.0;
  double mass_rl = 424.0;
  double mass_rr = 424.0;
  double mass_front = mass_fl + mass_fr;
  double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;
  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;
  max_front_wheel_steer_angle_ = 70.0 * PI / 180.0;
  max_lat_acc_ = 5.0;
  min_speed_protection_ = 0.1;
  // lqr solver paramters
  lqr_eps_ = 0.01;
  lqr_max_iteration_ = 150;

  cutoff_freq_ = 10.0F;
  mean_filter_window_size_ = 10U;
  basic_state_size_ = 4U;
  // std::cout << "load lateral controller finished..." << std::endl;
}

void LatController::ProcessLogs(const LateralControlDebug *debug) {
  using std::to_string;
  const std::string log_str = to_string(debug->ref_lateral_distance) + "," +
                              to_string(debug->lateral_distance) + "," +
                              to_string(debug->lateral_distance) + "," +
                              to_string(debug->lateral_distance_error) + ",";
  to_string(debug->ref_heading) + "," + to_string(debug->heading) + "," +
      to_string(debug->heading_error) + ",";
  to_string(debug->heading_error) + "," + to_string(debug->heading_error_rate) +
      "," + to_string(debug->curvature) + ",";
  to_string(debug->steer_angle) + "," +
      to_string(debug->steer_angle_feedforward) + "," +
      to_string(debug->lateral_distance_contribution) + ",";
  to_string(debug->lateral_distance_rate_contribution) + "," +
      to_string(debug->heading_contribudtion) + "," +
      to_string(debug->heading_rate_contribution) + "," +
      to_string(debug->steer_angle_feedback) + "," +
      to_string(debug->steer_angle) + "," +  // todo: steering_percentage
      to_string(debug->steer_angle);         // todo: linear_velocity

  if (FLAGS_enable_csv_debug) {
    steer_log_file_ << log_str << std::endl;
  }
  std::cout << "Lateral_Controller_Detail: " << log_str << std::endl;
}

void LatController::LogInitParameters() {
  std::cout << name_ << " begin.";
  std::cout << "[LatController parameters]"
            << " mass_: " << mass_ << ","
            << " iz_: " << iz_ << ","
            << " lf_: " << lf_ << ","
            << " lr_: " << lr_;
}

void LatController::InitializeFilters() {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(ts_, cutoff_freq_, &den, &num);
  digital_filter_.set_coefficients(den, num);
  distance_error_filter_ = common::MeanFilter(mean_filter_window_size_);
  heading_error_filter_ = common::MeanFilter(mean_filter_window_size_);
}

bool LatController::Init() {
  const int matrix_size = basic_state_size_;
  matrix_a_ = Eigen::MatrixXd::Zero(basic_state_size_, basic_state_size_);
  matrix_ad_ = Eigen::MatrixXd::Zero(basic_state_size_, basic_state_size_);
  matrix_adc_ = Eigen::MatrixXd::Zero(basic_state_size_, basic_state_size_);

  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

  matrix_a_coeff_ = Eigen::MatrixXd::Zero(basic_state_size_, basic_state_size_);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  matrix_b_ = Eigen::MatrixXd::Zero(basic_state_size_, 1);
  matrix_bd_ = Eigen::MatrixXd::Zero(basic_state_size_, 1);
  matrix_bdc_ = Eigen::MatrixXd::Zero(basic_state_size_, 1);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;

  // update state matrix
  matrix_state_ = Eigen::MatrixXd::Zero(basic_state_size_, 1);
  matrix_k_ = Eigen::MatrixXd::Zero(1, matrix_size);
  matrix_r_ = Eigen::MatrixXd::Identity(1, 1);
  matrix_q_ = Matrix::Zero(matrix_size, matrix_size);
  matrix_q_ << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0;
  matrix_q_updated_ = matrix_q_;
  InitializeFilters();
  // LoadLatGainScheduler();
  LogInitParameters();
  return true;
}

void LatController::CloseLogFile() {
  if (FLAGS_enable_csv_debug && steer_log_file_.is_open()) {
    steer_log_file_.close();
  }
}

void LatController::LoadLatGainScheduler() {
  double distance_error_gain_speed[5] = {4.0, 8.0, 12.0, 20.0, 25.0};
  double distance_error_gain_ratio[5] = {1.0, 0.6, 0.20, 0.01, 0.05};
  double heading_error_gain_speed[5] = {4.0, 8.0, 12.0, 20.0, 25.0};
  double heading_error_gain_ratio[5] = {1.0, 0.6, 0.40, 0.20, 0.10};

  std::vector<std::pair<double, double>> xy1, xy2;
  for (int ii = 0; ii < 5; ++ii) {
    xy1.push_back(std::make_pair(distance_error_gain_speed[ii],
                                 distance_error_gain_ratio[ii]));
    xy2.push_back(std::make_pair(heading_error_gain_speed[ii],
                                 heading_error_gain_ratio[ii]));
  }

  distance_error_interpolation_.reset(new common::Interpolation1D);
  heading_error_interpolation_.reset(new common::Interpolation1D);
}

void LatController::Stop() { CloseLogFile(); }

std::string LatController::Name() const { return name_; }

void LatController::ComputeControlCommand(
    const common_msgs::msg::Pose *localization,
    const common_msgs::msg::Trajectory *planning_published_trajectory,
    ControlCommand *cmd) {
  std::cout << "lateral controller start......" << std::endl;
  auto target_tracking_trajectory = *planning_published_trajectory;

  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(&target_tracking_trajectory));
  LateralControlDebug debug = cmd->simple_lat_debug;



  // the trajectory point closest to the actual position of the vehicle
  common_msgs::msg::TrajectoryPoint target_point;
  LoadControlConfig();

  // std::cout << planning_trajectory->trajectory.front().x << std::endl;
  ComputeLateralErrors(localization->x, localization->y, localization->yaw,
                       planning_published_trajectory);

  // Calculate the speed of the vehicle along the vehicle's x-axis
  double vx = localization->vel_y * std::cos(localization->yaw) +
              localization->vel_x * std::sin(localization->yaw);
  updateStateSpaceModel(vx);
  solveLqrProblem(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_, lqr_eps_,
                  lqr_max_iteration_, &matrix_k_);
  // feedback = - K * state
  steer_angle_feedback_ = -(matrix_k_ * matrix_state_)(0, 0);
  computeFeedforward(vx);
  steering_angle_command_ = steer_angle_feedback_ + steering_angle_feedforward_;
  std::cout << "steering angle command: " << steering_angle_command_
            << std::endl;
}

TrajectoryPoint LatController::QueryNearestPointByPosition(
    const double x, const double y,
    const common_msgs::msg::Trajectory *planning_trajectory) {
  // std::cout << "start to query target point" << std::endl;
  // std::cout << planning_trajectory->trajectory[0].x << std::endl;
  auto trajectory = planning_trajectory->trajectory;

  // auto func_distance_square = [](const common_msgs::msg::TrajectoryPoint
  // &point,
  //                                const double x, const double y) {
  //   double dx = point.x - x;
  //   double dy = point.y - y;
  //   return dx * dx + dy * dy;
  // };

  // double d_min =
  //     func_distance_square(trajectory.front(), x, y);
  size_t index_min = 0;

  // for (size_t i = 1; i < trajectory.size(); ++i) {
  //   double d_temp = func_distance_square(trajectory[i], x, y);
  //   if (d_temp < d_min) {
  //     d_min = d_temp;
  //     index_min = i;
  //   }
  // }
  // std::cout << "nearest point........." << std::endl;
  // std::cout << index_min << std::endl;
  // std::cout << trajectory[index_min].x << std::endl;
  // std::cout << trajectory[index_min].y << std::endl;
  return trajectory[index_min];
}

void LatController::ComputeLateralErrors(
    const double x, const double y, const double theta,
    const common_msgs::msg::Trajectory *trajectory) {
  // std::cout << "start to compute lateral error..." << std::endl;
  // the trajectory point closest to the actual position of the vehicle
  common_msgs::msg::TrajectoryPoint target_point;

  // target_point = QueryNearestPointByPosition(x, y, trajectory);
  // double dx = target_point.x - x;
  // double dy = target_point.y - y;
  // std::cout << "reference heading: " << target_point.theta
  //           << std::endl;
  // std::cout << "ego vehicle heading: " << theta << std::endl;

  // double cos_ego_heading = std::cos(target_point.theta);
  // double sin_ego_heading = std::sin(target_point.theta);
  // lateral_error_ = dy * cos_ego_heading - dx * sin_ego_heading;
  // heading_error_ = target_point.theta - theta;

  // std::cout << "lateral error" << lateral_error_ << std::endl;
  // std::cout << "heading error" << heading_error_ << std::endl;

  // todo: how to calculate lateral_error_dot and heading_error_dot
  // lateral_error_dot_ = linear_v * std::sin(heading_error_);
  // heading_error_dot_ = angular_v - target_trajectory_point_.kappa;
  lateral_error_dot_ = 0.0;
  heading_error_ = 0.0;
}

void LatController::updateStateSpaceModel(double vx) {
  vx = std::max(std::abs(vx), 0.01);
  matrix_a_ = Eigen::MatrixXd::Zero(4, 4);
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 1) = -(cf_ + cr_) / mass_ / vx;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_ / vx;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_ / vx;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_(3, 3) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_ / vx;

  matrix_b_ = Eigen::MatrixXd::Zero(4, 1);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;

  // discretization of State Space model
  Eigen::MatrixXd matrix_i = Eigen::MatrixXd::Identity(4, 4);
  matrix_ad_ = (matrix_i + 0.5 * ts_ * matrix_a_) *
               (matrix_i - 0.5 * ts_ * matrix_a_).inverse();
  matrix_bd_ = matrix_b_ * ts_;

  // update state matrix
  matrix_state_ = Eigen::MatrixXd::Zero(4, 1);
  matrix_state_(0, 0) = lateral_error_;
  matrix_state_(1, 0) = 0.1;
  matrix_state_(2, 0) = heading_error_;
  matrix_state_(3, 0) = 0.1;
}

void LatController::solveLqrProblem(
    const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, const double tolerance,
    const uint max_num_iteration, Eigen::MatrixXd *ptr_K) {
  if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() ||
      Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols()) {
    std::cout << "One or more matrices have incompatible dimensions.";
    return;
  }

  Eigen::MatrixXd AT = A.transpose();
  Eigen::MatrixXd BT = B.transpose();
  // Solves a discrete-time Algebraic Riccati equation (DARE)
  // Calculate Matrix Difference Riccati Equation, initialize P and Q
  Eigen::MatrixXd P = Q;
  uint num_iteration = 0;
  double diff = 0.0;
  while (num_iteration++ < max_num_iteration) {
    Eigen::MatrixXd P_next =
        AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q;
    // check the difference between P and P_next
    diff = fabs((P_next - P).maxCoeff());
    P = P_next;

    if (diff < tolerance) {
      break;
    }
  }

  if (num_iteration >= max_num_iteration) {
    std::cout << "lqr_not_convergence, last_diff_is:" << diff;
  } else {
    std::cout << "Number of iterations until convergence: " << num_iteration
              << ", max difference: " << diff;
  }
  *ptr_K = (R + BT * P * B).inverse() * BT * P * A;
}


// void LatController::UpdateState(LateralControlDebug *debug) {
  
// }



void LatController::computeFeedforward(double vx) {
  // cf_ is the sum of lateral stiffness of two front wheels
  // cr_ is the sum of lateral stiffness of two rear wheels
  double kv = lr_ * mass_ / cf_ / wheelbase_ - lf_ * mass_ / cr_ / wheelbase_;

  double ref_curvature = 0.002;
  // then change it from rad to %
  steering_angle_feedforward_ =
      wheelbase_ * ref_curvature + kv * vx * vx * ref_curvature -
      matrix_k_(0, 2) *
          (lr_ * ref_curvature -
           lf_ * mass_ * vx * vx * ref_curvature / cr_ / wheelbase_);
}

double LatController::GetSteeringAngleCommand() {
  return steering_angle_command_;
}

}  // namespace control
