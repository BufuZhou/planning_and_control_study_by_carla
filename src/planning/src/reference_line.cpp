// copyright
#include "planning/reference_line.hpp"

namespace planning {

// 构造函数，将路径点存储到私有成员xy_points_中
ReferenceLine::ReferenceLine(
    const std::vector<std::pair<double, double>>& xy_points) {
  xy_points_ = xy_points;
}

// 根据路径点（x,y）计算得到轨迹
bool ReferenceLine::ComputePathProfile(std::vector<double>* headings,
                                       std::vector<double>* accumulated_s,
                                       std::vector<double>* kappas,
                                       std::vector<double>* dkappas) {
  headings->clear();
  kappas->clear();
  dkappas->clear();

  if (xy_points_.size() < 2) {
    return false;
  }
  std::vector<double> dxs;
  std::vector<double> dys;
  std::vector<double> y_over_s_first_derivatives;
  std::vector<double> x_over_s_first_derivatives;
  std::vector<double> y_over_s_second_derivatives;
  std::vector<double> x_over_s_second_derivatives;

  // Get finite difference approximated dx and dy for heading and kappa calculation
  // 航向角是曲线的切线与x轴的夹角，所以航向角的正切值等于曲线的一阶导数。
  // 这里利用中心差分的方法计算曲线的一阶导数
  // 计算某个点的中心差分需要三个点，该点前面一个点，该点，该点后面一个点，公式如下：
  // y_n' = (y_n+1 - y_n-1) / (x_n+1 - x_n-1)
  // 所以起点和终点不满足条件，需要做特殊处理，起点的导数是向后差分，终点的导数是向前差分
  std::size_t points_size = xy_points_.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (xy_points_[i + 1].first - xy_points_[i].first);
      y_delta = (xy_points_[i + 1].second - xy_points_[i].second);
    } else if (i == points_size - 1) {
      x_delta = (xy_points_[i].first - xy_points_[i - 1].first);
      y_delta = (xy_points_[i].second - xy_points_[i - 1].second);
    } else {
      x_delta = 0.5 * (xy_points_[i + 1].first - xy_points_[i - 1].first);
      y_delta = 0.5 * (xy_points_[i + 1].second - xy_points_[i - 1].second);
    }
    dxs.push_back(x_delta);
    dys.push_back(y_delta);
  }

  // Heading calculation
  // 计算出一阶导数，然后反正切，计算出航向角
  for (std::size_t i = 0; i < points_size; ++i) {
    headings->push_back(std::atan2(dys[i], dxs[i]));
  }

  // Get linear interpolated s for dkappa calculation
  // 计算距离起点的距离s
  // 特定点的距离s表示的是从起点到该点的累积弧长，计算方法如下
  // （1）计算出相邻点的距离，end_segment_s = sqrt((y_n - y_n-1)^2 + (x_n - x_n-1)^2)
  // （2）分别对ds进行累加，依次得到特定点的距离s
  double distance = 0.0;
  accumulated_s->push_back(distance);
  double fx = xy_points_[0].first;
  double fy = xy_points_[0].second;
  double nx = 0.0;
  double ny = 0.0;
  for (std::size_t i = 1; i < points_size; ++i) {
    nx = xy_points_[i].first;
    ny = xy_points_[i].second;
    double end_segment_s =
        std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
    accumulated_s->push_back(end_segment_s + distance);
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  // Get finite difference approximated first derivative of y and x respective
  // to s for kappa calculation
  // 曲率的定义就是曲线的弯曲程度，就是某段弧长切线夹角的变化率，数据定义如下：
  //  kappa = dalpha/ds, kappa是曲率，dalpha是切线与x轴的夹角（航向角），ds是弧长
  // 一般曲率的计算公式：
  // kappa = y''/(1 + y'^2)^(3/2)
  // 参数化的曲率计算公式：
  // kappa = (x'(t)y''(t) - x''(t)y'(t))/(x'(t)x'(t) + y'(t)y'(t))^(3/2)
  // 参考资料1：https://www.cnblogs.com/fujj/p/9704589.html
  // 参考资料2：https://blog.csdn.net/buaazyp/article/details/82622972
  // 参考资料3：https://zhuanlan.zhihu.com/p/427833623
  // 
  // 这里计算曲率采用的是参数化的曲率公式
  // 另外，令x = x(s), y = y(s)来进行计算一阶导数和二阶导数

  // 一阶导数dx/ds和dy/ds，都是利用中心差分的方法，具体可以参考前面航向角的计算
  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0) {
      xds = (xy_points_[i + 1].first - xy_points_[i].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
      yds = (xy_points_[i + 1].second - xy_points_[i].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xds = (xy_points_[i].first - xy_points_[i - 1].first) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
      yds = (xy_points_[i].second - xy_points_[i - 1].second) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xds = (xy_points_[i + 1].first - xy_points_[i - 1].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      yds = (xy_points_[i + 1].second - xy_points_[i - 1].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_first_derivatives.push_back(xds);
    y_over_s_first_derivatives.push_back(yds);
  }

  // Get finite difference approximated second derivative of y and x
  // respective to s for kappa calculation
  // 二阶导数d^2x/ds^2和d^2y/ds^2，都是利用中心差分的方法，具体可以参考前面航向角的计算
  for (std::size_t i = 0; i < points_size; ++i) {
    double xdds = 0.0;
    double ydds = 0.0;
    if (i == 0) {
      xdds =
          (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
      ydds =
          (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xdds =
          (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
      ydds =
          (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] -
              x_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      ydds = (y_over_s_first_derivatives[i + 1] -
              y_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_second_derivatives.push_back(xdds);
    y_over_s_second_derivatives.push_back(ydds);
  }

  // 计算曲率，利用参数化的曲率计算公式
  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    double kappa =
        (xds * ydds - yds * xdds) /
        (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
    kappas->push_back(kappa);
  }

  // Dkappa calculation
  // 曲率变化率，直接利用中心差分的计算方法，参考前面航向角中心差分的处理方法。
  for (std::size_t i = 0; i < points_size; ++i) {
    double dkappa = 0.0;
    if (i == 0) {
      dkappa = (kappas->at(i + 1) - kappas->at(i)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      dkappa = (kappas->at(i) - kappas->at(i - 1)) /
               (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    dkappas->push_back(dkappa);
  }
  return true;
}
}  // namespace planning
