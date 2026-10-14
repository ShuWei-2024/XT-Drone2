// #include <fstream>
#include <ego_planner/planner_manager.h>
#include <thread>
#include <bspline_opt/stationary_terminal.h>
#include <bspline_opt/retime_bspline.h>
#include <bspline_opt/grid_bspline.h>
#include "visualization_msgs/msg/marker.hpp" // zx-todo

namespace ego_planner
{

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis)
  {
    node_ = node;
    grid_seed_fallback_ = node->declare_parameter("manager/grid_seed_fallback", false);
    min_goal_distance_m_ = node->declare_parameter("manager/min_goal_distance_m", 0.2);
    if (!std::isfinite(min_goal_distance_m_) || min_goal_distance_m_ <= 0.0)
      throw std::invalid_argument("minimum goal distance must be positive and finite");
    node->declare_parameter("manager/max_vel", -1.0);
    node->declare_parameter("manager/max_acc", -1.0);
    node->declare_parameter("manager/max_jerk", -1.0);
    node->declare_parameter("manager/feasibility_tolerance", 0.0);
    node->declare_parameter("manager/control_points_distance", -1.0);
    node->declare_parameter("manager/planning_horizon", 5.0);
    node->declare_parameter("manager/use_distinctive_trajs", false);
    node->declare_parameter("manager/drone_id", -1);

    node->get_parameter("manager/max_vel", pp_.max_vel_);
    node->get_parameter("manager/max_acc", pp_.max_acc_);
    node->get_parameter("manager/max_jerk", pp_.max_jerk_);
    node->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
    node->get_parameter("manager/control_points_distance", pp_.ctrl_pt_dist);
    node->get_parameter("manager/planning_horizon", pp_.planning_horizen_);
    node->get_parameter("manager/use_distinctive_trajs", pp_.use_distinctive_trajs);
    node->get_parameter("manager/drone_id", pp_.drone_id);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    grid_map_->initMap(node);

    bspline_optimizer_.reset(new BsplineOptimizer);
    // bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setParam(node);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);

    if ((start_pt - local_target_pt).norm() < min_goal_distance_m_)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    if (planningBudgetExpired()) return false;
    bool envelope_detour = false;
    if (grid_map_->executionEnvelopeRequired()) {
      const int n = std::max(1, static_cast<int>(std::ceil((local_target_pt-start_pt).norm()/.1)));
      for (int i=0; i<=n; ++i)
        if (!grid_map_->executionEnvelopeAllows(start_pt+(local_target_pt-start_pt)*(double(i)/n))) {
          envelope_detour = true;
          break;
        }
    }
    // A vertical search cannot cross an XY safety exclusion. Try the bounded
    // sensor-grid detour first instead of exhausting the budget on fly-over seeds.
    if (grid_seed_fallback_ && (flag_randomPolyTraj || envelope_detour) &&
        gridFallbackReplan(start_pt, start_vel, start_acc, local_target_pt, local_target_vel))
      return true;
    if (planningBudgetExpired()) return false;
    bspline_optimizer_->setLocalTargetPt(local_target_pt);
    bspline_optimizer_->setStationaryTerminal(local_target_vel.norm() < 1e-6);

    rclcpp::Time t_start = node_->now();
    rclcpp::Duration t_init(0, 0), t_opt(0, 0), t_refine(0, 0);

    /*** STEP 1: INIT
    根据起始点和目标点的距离计算首个时间步长ts,向量的模大于0.1则用1.5倍否则用5倍
    ***/
    // Arc-resampled controls are spaced at ctrl_pt_dist. A fixed 1.5x
    // interval slowed every replan unnecessarily; final derivative hull
    // retiming already enforces the configured speed/acceleration limits.
    double ts = pp_.ctrl_pt_dist / pp_.max_vel_;
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      // 这里如果正常进入if（通常为初次生成），则do部分只进行一次，即只清空一次点集；若进入else则有可能对异常情况重置flag_regenerate并再do一次
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;
        // 用于存储生成的轨迹
        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        // 判断 速度的平方/加速度 是否大于dist，并决定如何计算时间
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;
        RCLCPP_INFO(node_->get_logger(), "Trajectory seed: distance=%.3f duration=%.3f dt=%.3f v=%.3f a=%.3f",
                    dist, time, ts, pp_.max_vel_, pp_.max_acc_);
        if (!start_pt.allFinite() || !start_vel.allFinite() || !start_acc.allFinite() ||
            !local_target_pt.allFinite() || !local_target_vel.allFinite() ||
            !std::isfinite(time) || time <= 0.0 || !std::isfinite(ts) || ts <= 0.0)
        {
          RCLCPP_ERROR(node_->get_logger(), "Invalid trajectory seed; rejecting planning attempt");
          continous_failures_count_++;
          return false;
        }

        if (!flag_randomPolyTraj)
        // false生成一段单一的多项式轨迹，true生成一个包含随机插入点的轨迹
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        int sampling_attempts = 0;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          if (++sampling_attempts > 32)
          {
            RCLCPP_ERROR(node_->get_logger(), "Trajectory seed sampling did not converge");
            continous_failures_count_++;
            return false;
          }
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = std::clamp((node_->now() - local_data_.start_time_).seconds(), 0.0, local_data_.duration_);

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        if (pseudo_arc_length.size() < 2 || segment_point.size() != pseudo_arc_length.size() ||
            !std::isfinite(pseudo_arc_length.back()) || pseudo_arc_length.back() < 1e-6)
        {
          RCLCPP_WARN(node_->get_logger(), "Degenerate previous trajectory seed; use polynomial retry");
          return false;
        }
        int resampling_attempts = 0;
        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          if (++resampling_attempts > 32)
          {
            RCLCPP_WARN(node_->get_logger(), "Previous trajectory resampling limit reached");
            return false;
          }
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id + 1 < pseudo_arc_length.size()) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // 将轨迹变为B样条轨迹
    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    if (bspline_optimizer_->stationaryTerminal())
      pinStationaryTerminal(ctrl_pts, local_target_pt);
    RCLCPP_INFO(node_->get_logger(), "Trajectory seed sampled: %zu points; starting optimization", point_set.size());

    vector<std::pair<int, int>> segments;
    if (planningBudgetExpired()) return false;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);
    if (planningBudgetExpired()) return false;
    // 计算时间差并更新时间
    auto now = node_->now();
    t_init = now - t_start;
    t_start = now;

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    if (pp_.use_distinctive_trajs)
    {
      // cout << "enter" << endl;
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = node_->now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2);
    }
    else
    {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = node_->now() - t_start;
      // static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }

    t_start = node_->now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    // Keep the optimized detour instead of geometrically refitting its tail.
    // Restore the new curve's head position/velocity after knot scaling so
    // repeated replans do not repeatedly reduce the commanded speed. The
    // changed head and complete tail must pass the occupancy check below.
    double time_scale;
    try
    {
      time_scale = retimeStartContinuousSpline(pos, start_pt, start_vel, pp_.max_vel_, pp_.max_acc_, start_acc);
    }
    catch (const std::invalid_argument &error)
    {
      // A highly curved handoff can make strict C2 continuity require more
      // than the bounded time scale. Rebuild a braking prefix with the same
      // position and velocity, then let the unchanged full occupancy check
      // decide whether this safer handoff is usable.
      if (start_acc.norm() <= 1e-8)
      {
        RCLCPP_ERROR(node_->get_logger(), "Rejecting trajectory: %s", error.what());
        continous_failures_count_++;
        return false;
      }
      try
      {
        time_scale = retimeStartContinuousSpline(pos, start_pt, start_vel,
                                                   pp_.max_vel_, pp_.max_acc_);
        RCLCPP_WARN(node_->get_logger(),
                    "Using bounded braking prefix after strict acceleration handoff was infeasible");
      }
      catch (const std::invalid_argument &fallback_error)
      {
        RCLCPP_ERROR(node_->get_logger(), "Rejecting trajectory: %s (fallback: %s)",
                     error.what(), fallback_error.what());
        continous_failures_count_++;
        return false;
      }
    }
    if (time_scale > 1.01)
      RCLCPP_WARN(node_->get_logger(), "Scaled trajectory time by %.3f to meet motion limits", time_scale);

    // An optimizer's nonuniform control spacing can make its derivative hull
    // much faster than the actual vehicle, forcing the whole detour to crawl.
    // Construct an arc-spaced candidate and accept it only after a fresh full
    // occupancy check; otherwise retain the original bounded curve.
    try
    {
      std::vector<Eigen::Vector3d> arc_path;
      const double original_duration=pos.getTimeSum();
      if (original_duration/0.05 < 20000)
      {
        for (double t=0.; ; t=std::min(original_duration,t+0.05))
        {
          arc_path.push_back(pos.evaluateDeBoorT(t));
          if (t>=original_duration || planningBudgetExpired()) break;
        }
        arc_path.front()=start_pt;
        // Keep construction bounded even for a long slow curve.
        if (arc_path.size()>2048)
        {
          std::vector<Eigen::Vector3d> compact;
          for (size_t i=0;i<arc_path.size();i+=2) compact.push_back(arc_path[i]);
          if ((compact.back()-arc_path.back()).norm()>1e-9) compact.push_back(arc_path.back());
          arc_path=std::move(compact);
        }
        auto candidate=gridPathSpline(arc_path,start_vel,start_acc,
                                     local_target_vel,pp_.ctrl_pt_dist,pp_.ctrl_pt_dist/pp_.max_vel_);
        retimeStartContinuousSpline(candidate,start_pt,start_vel,pp_.max_vel_,pp_.max_acc_,start_acc);
        bool free=true;
        for (double t=0.; free; t=std::min(candidate.getTimeSum(),t+0.05))
        {
          const Eigen::Vector3d point=candidate.evaluateDeBoorT(t);
          free=!planningBudgetExpired() && point.allFinite() && grid_map_->isInMap(point) &&
               !grid_map_->getInflateOccupancy(point);
          if (t>=candidate.getTimeSum()) break;
        }
        if (free && candidate.getTimeSum()<original_duration)
        {
          RCLCPP_INFO(node_->get_logger(), "Accepted arc-spaced detour: %.3f -> %.3f s",
                      original_duration,candidate.getTimeSum());
          pos=candidate;
        }
      }
    }
    catch (const std::invalid_argument &) { /* Retain the checked original below. */ }

    const double duration = pos.getTimeSum();
    const double sample_dt = std::min(0.05, grid_map_->getResolution() /
        (4.0 * std::max(pp_.max_vel_, 1e-3)));
    if (!std::isfinite(duration) || duration <= 0.0 || duration / sample_dt > 20000)
    {
      continous_failures_count_++;
      return false;
    }
    for (double t = 0.0; ; t = std::min(duration, t + sample_dt))
    {
      const Eigen::Vector3d point = pos.evaluateDeBoorT(t);
      if (!point.allFinite() || !grid_map_->isInMap(point) ||
          grid_map_->getInflateOccupancy(point))
      {
        RCLCPP_WARN(node_->get_logger(), "Rejecting colliding final trajectory at t=%.3f", t);
        continous_failures_count_++;
        return false;
      }
      if (t >= duration) break;
    }

    // t_refine = ros::Time::now() - t_start;
    t_refine = node_->now() - t_start;

    // save planned results
    if (planningBudgetExpired()) return false;
    updateTrajInfo(pos, node_->now());

    static double sum_time = 0;
    static int count_success = 0;

    sum_time += (t_init + t_opt + t_refine).seconds();

    count_success++;

    // cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).seconds() << "\033[0m,optimize:" << (t_init + t_opt).seconds() << ",refine:" << t_refine.seconds() << ",avg_time=" << sum_time / count_success << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::gridFallbackReplan(const Eigen::Vector3d &start,
      const Eigen::Vector3d &velocity, const Eigen::Vector3d &acceleration,
      const Eigen::Vector3d &goal, const Eigen::Vector3d &goal_velocity)
  {
    if (planningBudgetExpired()) return false;
    auto search = bspline_optimizer_->a_star_;
    // A cell of extra horizontal clearance protects spline corner rounding.
    // Live sensor occupancy plus the independent shared safety envelope.
    search->setSearchMargin(1, std::abs(start.z()-goal.z()) < 0.15);
    const double spacing=std::min(pp_.ctrl_pt_dist,grid_map_->getResolution()/2.0);
    const double dt=spacing/pp_.max_vel_;
    // A heading reversal needs a short forward prefix before the grid detour.
    // Starting its control polygon backwards made a C1-continuous retime
    // infeasible (or colliding) even when a safe turn exists.
    const Eigen::Vector3d search_start = velocity.norm()>1e-4
        ? Eigen::Vector3d(start+velocity.normalized()*(spacing+velocity.norm()*dt)) : start;
    const bool found = search->AstarSearch(grid_map_->getResolution(), search_start, goal);
    search->setSearchMargin(0, false);
    if (!found || planningBudgetExpired()) return false;
    auto path = search->getPath();
    if (path.size() < 2 || path.size() > 2048) return false;
    if ((path.front()-start).norm() > (path.back()-start).norm())
      std::reverse(path.begin(),path.end());
    path.front() = search_start;
    path.back() = goal;
    if ((search_start-start).norm()>1e-8) path.insert(path.begin(),start);
    UniformBspline position = gridPathSpline(path,velocity,acceleration,goal_velocity,
                                             spacing,dt);
    try { retimeStartContinuousSpline(position,start,velocity,pp_.max_vel_,pp_.max_acc_,acceleration); }
    catch (const std::invalid_argument &error) {
      RCLCPP_WARN(node_->get_logger(), "Grid fallback retiming rejected: %s",error.what());
      return false;
    }
    const double duration=position.getTimeSum();
    for (double t=0.; ; t=std::min(duration,t+0.05))
    {
      const Eigen::Vector3d point=position.evaluateDeBoorT(t);
      if (planningBudgetExpired() || !point.allFinite() || !grid_map_->isInMap(point) ||
          grid_map_->getInflateOccupancy(point)) {
        RCLCPP_WARN(node_->get_logger(), "Grid fallback rejected at t=%.3f XYZ=%.3f %.3f %.3f",t,point.x(),point.y(),point.z());
        return false;
      }
      if (t >= duration) break;
    }
    if ((position.evaluateDeBoorT(0.)-start).norm()>1e-6 ||
        (position.evaluateDeBoorT(duration)-goal).norm()>1e-6) return false;
    RCLCPP_WARN(node_->get_logger(), "Accepted sensor-grid B-spline fallback: %ld controls, %.3f s",position.getControlPoint().cols(),duration);
    updateTrajInfo(position,node_->now());
    continous_failures_count_=0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    // if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
    if (local_data_.start_time_.seconds() < 1e9)
      return false;

    // double my_traj_start_time = local_data_.start_time_.toSec();
    // double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();
    double my_traj_start_time = local_data_.start_time_.seconds();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.seconds();

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();

    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    /*挨个读取点并计算点距判断是否需要插点，随后计算插点并写入矩阵，最后根据插点数量生成全局轨迹
      最终返回值为是否规划成功的布尔值 */
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();

    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
    if (bspline_optimizer_->stationaryTerminal())
      pinStationaryTerminal(ctrl_pts, bspline.evaluateDeBoorT(duration));
  }

} // namespace ego_planner
