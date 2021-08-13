/**
 *  Author: Tianhua Zhao
 */
#ifndef PLANNER
#define PLANNER

#include <algorithm>
#include <array>
#include <vector>

#include "spline.h"

namespace planner {

enum BehaviorType {
  KEEP_LANE = 0,
  LANE_CHANGE_RIGHT = 1,
  LANE_CHANGE_LEFT = -1,
};

struct LocalizationData {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  int lane;  // 0 is left, 1 is mid, 2 is right
};

/**
 *  Motion Planning Class
 */
class Planner {
 public:
  /**
   * Planner
   *
   * @param  {bool} p_prefer_mid                : prefer mid lane if both lanes
   *                                              have slow vehicles?
   * @param  {double} p_speed_limit             : speed limit in miles per hour
   * @param  {int} p_prev_points_to_use         : number of previous path points
   *                                              to use for better smoothness
   * @param  {int} p_path_size                  : number of new path points to
   *                                              generate
   * @param  {double} p_accel                   : tangential acceleration when
   *                                              changing speed
   * @param  {double} p_control_points_distance : distance between spline 
   *                                              control points, small
   *                                              value could lead to large
   *                                              normal acceleration when
   *                                              changing lanes
   * @param  {double} p_delta_t                 : time gap between visiting two
   *                                              waypoints
   */
  Planner(bool p_prefer_mid = true, double p_speed_limit = 49.8,
          double p_lane_width = 4.0, int p_prev_points_to_use = 5,
          int p_path_size = 45, double p_accel = 5,
          double p_control_points_distance = 36, double p_delta_t = 0.02) {
    prefer_mid = p_prefer_mid;
    speed_limit = p_speed_limit * 0.446944;  // convert to m/s
    lane_width = p_lane_width;
    prev_points_to_use = p_prev_points_to_use;
    path_size = p_path_size;
    accel = p_accel;
    control_points_distance = p_control_points_distance;
    delta_t = p_delta_t;
  }

  /**
   * Load map data
   * @param  {std::vector<double>} map_waypoints_x  :
   * @param  {std::vector<double>} map_waypoints_y  :
   * @param  {std::vector<double>} map_waypoints_s  :
   * @param  {std::vector<double>} map_waypoints_dx :
   * @param  {std::vector<double>} map_waypoints_dy :
   */
  void LoadMap(const std::vector<double> &map_waypoints_x,
               const std::vector<double> &map_waypoints_y,
               const std::vector<double> &map_waypoints_s,
               const std::vector<double> &map_waypoints_dx,
               const std::vector<double> &map_waypoints_dy) {
    map_waypoints_x_ = map_waypoints_x;
    map_waypoints_y_ = map_waypoints_y;
    map_waypoints_s_ = map_waypoints_s;
    map_waypoints_dx_ = map_waypoints_dx;
    map_waypoints_dy_ = map_waypoints_dy;
  }

  /**
   * Compute the next path
   * @param  {LocalizationData} localization_data             :
   * @param  {std::vector<double>} previous_path_x            :
   * @param  {std::vector<double>} previous_path_y            :
   * @param  {double} end_path_s                              :
   * @param  {double} end_path_d                              :
   * @param  {std::vector<std::vector<double>>} sensor_fusion :
   */
  void ComputeNextPath(const LocalizationData &localization_data,
                       const std::vector<double> &previous_path_x,
                       const std::vector<double> &previous_path_y,
                       double end_path_s, double end_path_d,
                       const std::vector<std::vector<double>> &sensor_fusion) {
    ego_state_ = localization_data;
    ego_state_.speed *= 0.446944;  // comvert to m/s
    // compute ego lane id, 0 is left, 1 is mid, 2 is right
    ego_state_.lane = floor(ego_state_.d / lane_width);

    sensor_fusion_ = sensor_fusion;
    previous_path_x_ = previous_path_x;
    previous_path_y_ = previous_path_y;
    end_path_s_ = end_path_s;
    end_path_d_ = end_path_d;

    // behavior planning
    PlanBehavior();

    // trajectory generation
    GenerateTrajectory();

    // iteration counter
    iteration_++;
    // std::cout << "iteration :" << iteration_;
  };

  // some tunable parameters
  double speed_limit;      // speed limit in m per s
  double lane_width;       // lane width
  int prev_points_to_use;  // number of previous path points to use
  int path_size;           // new path size, not including previous points
  double accel;            // acceleration in m/s
  double control_points_distance;  // distance between control points in meters
  double delta_t;                  // time gap between waypoints
  bool prefer_mid;  // prefer mid lane if both lanes have slow vehicles ahead?

  // behavior planning range thresholds
  double check_range_ahead = 45;  // check range for vehicles ahead on ego lane
  double check_range_side_ahead =
      25;  // check range for vehicles ahead on side lanes
  double check_range_side_behind =
      10;  // check range for vehicles behind on side lanes
  double check_range_side_ahead_far =
      65;  // check range for slow vehicles ahead on side lanes
  double check_range_side_behind_far =
      20;  // check range for fast vehicle behind on side lanes
  double time_to_react =
      1.7;  // safe following distance = ego speed * time to react

  // path planning result
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

 private:
  /**
   * Determine the behavior base on localization and sensor fusion data.
   * Behavior planning policy:
   * - if ego is in the middle of a lane change, continue the lane change
   * - else if there is no slow vehicles ahead in range, keep lane
   * - else if the car ahead is too close, slow down, keep following distance
   * - else if can change left, lane change left
   * - else if can change right, lane change right
   * - else, keep lane, follow car ahead
   */
  void PlanBehavior() {
    // first check if ego is in the middle of a lane change
    // if so, should continue the lane change
    int ending_lane = floor(end_path_d_ / lane_width);
    if (previous_path_x_.size() and
        ego_state_.lane != ending_lane) {  // in the middle of a lane change
      // should continue the lane change
      optimal_behavior_ = (BehaviorType)(ending_lane - ego_state_.lane);
      std::cout << "Continue lane changing." << std::endl;
      target_speed_ = speed_limit;
      return;
    }

    bool car_ahead = false;
    double car_ahead_speed = 1000;  // large number
    bool car_ahead_too_close = false;
    bool can_change_left = true;
    bool can_change_right = true;
    bool slow_vehicle_on_left = false;
    bool slow_vehicle_on_right = false;
    bool merging_vehicle_on_left = false;
    bool merging_vehicle_on_right = false;

    if (ego_state_.lane == 0) {
      can_change_left = false;
    }

    if (ego_state_.lane == 2) {
      can_change_right = false;
    }

    // process sensor fusion data
    for (auto vehicle : sensor_fusion_) {
      double d = vehicle[6];

      // ignore cars on the other side
      if (d < 0 or d > 3 * lane_width) {
        continue;
      }

      // helper function to compute speed
      auto get_speed = [](const std::vector<double> &vehicle) -> double {
        double vx = vehicle[3];
        double vy = vehicle[4];
        return sqrt(vx * vx + vy * vy);
      };

      int lane = floor(d / lane_width);  // 0 is left, 1 is mid, 2 is right
      if (lane == ego_state_.lane) {
        // if the vehicle is in our lane, check if it is close and slow
        double s = vehicle[5];
        if (s > ego_state_.s and s < ego_state_.s + check_range_ahead) {
          double speed = get_speed(vehicle);
          if (speed < speed_limit) {
            car_ahead = true;
            // track the slowest vehicle
            car_ahead_speed = std::min(car_ahead_speed, speed);
            std::cout << "Slow vehicle ahead." << std::endl;
          }
          if (s < ego_state_.s + ego_state_.speed * time_to_react) {
            // check if it is too close, if so, should slow down
            // and keep lane
            std::cout << "Too close to the vehicle ahead!!!" << std::endl;
            car_ahead_too_close = true;
          }
        }
      } else if (lane == ego_state_.lane - 1) {
        // if the vehicle is to our left
        double s = vehicle[5];
        if (s > ego_state_.s - check_range_side_behind and
            s < ego_state_.s + check_range_side_ahead_far) {
          if (s > ego_state_.s + check_range_side_ahead) {
            if (get_speed(vehicle) < speed_limit) {
              std::cout << "Slow vehicle ahead on the left." << std::endl;
              slow_vehicle_on_left = true;
            }
          } else {
            std::cout << "Close vehicle on the left." << std::endl;
            can_change_left = false;
            if (d - lane * lane_width > lane_width * 0.75) {
              // predict the vehicle would merge to ego lane
              std::cout << "Possible merging." << std::endl;
              merging_vehicle_on_left = true;
            }
          }
        } else if (s > ego_state_.s - check_range_side_behind_far and
                   s <= ego_state_.s - check_range_side_behind and
                   get_speed(vehicle) > ego_state_.speed) {
          // check if the vehicle is within a further range behind and faster
          // than ego
          std::cout << "Fast vehicle behind on the left." << std::endl;
          can_change_left = false;
        }
      } else if (lane == ego_state_.lane + 1) {
        // if the vehicle is to our right
        double s = vehicle[5];
        if (s > ego_state_.s - check_range_side_behind and
            s < ego_state_.s + check_range_side_ahead_far) {
          if (s > ego_state_.s + check_range_side_ahead) {
            if (get_speed(vehicle) < speed_limit) {
              std::cout << "Slow vehicle ahead on the right." << std::endl;
              slow_vehicle_on_right = true;
            }
          } else {
            std::cout << "Close vehicle on the right." << std::endl;
            can_change_right = false;
            if (d - lane * lane_width < lane_width * 0.25) {
              // predict the vehicle would merge to ego lane
              std::cout << "Possible merging." << std::endl;
              merging_vehicle_on_right = true;
            }
          }
        } else if (s > ego_state_.s - check_range_side_behind_far and
                   s <= ego_state_.s - check_range_side_behind and
                   get_speed(vehicle) > ego_state_.speed) {
          // check if the vehicle is within a further range behind and faster
          // than ego
          std::cout << "Fast vehicle behind on the right." << std::endl;
          can_change_right = false;
        }
      }
    }
    // policy

    if (not car_ahead) {
      if (merging_vehicle_on_left and can_change_right) {
        optimal_behavior_ = LANE_CHANGE_RIGHT;
        target_speed_ = speed_limit;
        std::cout << "Changing lane right to avoid merging vehicle."
                  << std::endl;
        num_merging_++;
      } else if (merging_vehicle_on_right and can_change_left) {
        optimal_behavior_ = LANE_CHANGE_LEFT;
        target_speed_ = speed_limit;
        std::cout << "Changing lane left to avoid merging vehicle."
                  << std::endl;
        num_merging_++;
      }
      optimal_behavior_ = KEEP_LANE;
      target_speed_ = speed_limit;
      std::cout << "Keeping lane. " << std::endl;
    } else if (car_ahead_too_close) {
      optimal_behavior_ = KEEP_LANE;
      target_speed_ = 0;
      std::cout << "Slowing down." << std::endl;
    } else if (can_change_left and not slow_vehicle_on_left) {
      optimal_behavior_ = LANE_CHANGE_LEFT;
      target_speed_ = speed_limit;
      std::cout << "Changing lane left." << std::endl;
    } else if (can_change_right and not slow_vehicle_on_right) {
      optimal_behavior_ = LANE_CHANGE_RIGHT;
      target_speed_ = speed_limit;
      std::cout << "Changing lane right." << std::endl;
    } else {
      if (prefer_mid and ego_state_.lane != 1 and
          (can_change_left or can_change_right)) {
        // if both lanes have slow vehicles, and prefer mid
        std::cout << "Changing lane to mid (preferred)" << std::endl;
        optimal_behavior_ = (BehaviorType)(1 - ego_state_.lane);
        target_speed_ = speed_limit;
        num_change_mid_++;
      } else {
        std::cout << "Following the vehicle ahead." << std::endl;
        optimal_behavior_ = KEEP_LANE;
        target_speed_ = car_ahead_speed;
      }
    }
    // std::cout << "ego spd: " << ego_state_.speed << std::endl;
    // std::cout << "target spd: " << target_speed_ << std::endl;
  }

  /**
   * Generate a smooth path base on the optimal behavior and the target speed
   */
  void GenerateTrajectory() {
    next_x_vals.clear();
    next_y_vals.clear();

    vector<double> control_points_x;
    vector<double> control_points_y;

    double speed;  // current ego speed
    double yaw;    // current ego yaw

    int destination_lane = ego_state_.lane + (int)optimal_behavior_;
    double destination_d = 2 + 4 * destination_lane;

    int previous_path_size = previous_path_x_.size();

    if (previous_path_size < 2) {
      // add two points that are tangent to ego car as control points
      double prev_car_x = ego_state_.x - cos(ego_state_.yaw);
      double prev_car_y = ego_state_.y - sin(ego_state_.yaw);

      control_points_x.push_back(prev_car_x);
      control_points_y.push_back(prev_car_y);
      control_points_x.push_back(ego_state_.x);
      control_points_y.push_back(ego_state_.y);

      yaw = ego_state_.yaw;
      speed = ego_state_.speed;
    } else {
      // add some points from the previous path for smoothness
      for (int i = 0; i < prev_points_to_use; ++i) {
        next_x_vals.push_back(previous_path_x_[i]);
        next_y_vals.push_back(previous_path_y_[i]);
      }

      // add the last two points from the previous path to the control points
      control_points_x.push_back(next_x_vals[prev_points_to_use - 2]);
      control_points_x.push_back(next_x_vals[prev_points_to_use - 1]);
      control_points_y.push_back(next_y_vals[prev_points_to_use - 2]);
      control_points_y.push_back(next_y_vals[prev_points_to_use - 1]);

      // compute the current ego yaw and speed at the last previous point to
      // use
      yaw = atan2(next_y_vals[prev_points_to_use - 1] -
                      next_y_vals[prev_points_to_use - 2],
                  next_x_vals[prev_points_to_use - 1] -
                      next_x_vals[prev_points_to_use - 2]);

      speed = distance(next_x_vals[prev_points_to_use - 2],
                       next_y_vals[prev_points_to_use - 2],
                       next_x_vals[prev_points_to_use - 1],
                       next_y_vals[prev_points_to_use - 1]) *
              50;
    }

    // add more control points near the end of the path
    vector<double> control_point2 =
        getXY(ego_state_.s + control_points_distance, destination_d,
              map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    vector<double> control_point3 =
        getXY(ego_state_.s + control_points_distance * 2, destination_d,
              map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    vector<double> control_point4 =
        getXY(ego_state_.s + control_points_distance * 3, destination_d,
              map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);

    control_points_x.push_back(control_point2[0]);
    control_points_x.push_back(control_point3[0]);
    control_points_x.push_back(control_point4[0]);
    control_points_y.push_back(control_point2[1]);
    control_points_y.push_back(control_point3[1]);
    control_points_y.push_back(control_point4[1]);

    // transform to ego coordinate, where ego car is at the origin, ego
    // heading is 0
    double ref_x = control_points_x[1];
    double ref_y = control_points_y[1];
    for (int i = 0; i < control_points_x.size(); ++i) {
      double shift_x = control_points_x[i] - ref_x;
      double shift_y = control_points_y[i] - ref_y;

      control_points_x[i] = (shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw));
      control_points_y[i] = (shift_x * sin(0 - yaw) + shift_y * cos(0 - yaw));
    }

    // create a spline using control points
    tk::spline sp(control_points_x, control_points_y);

    double cur_x = 0;  // current ego location in ego coordinate
    double cur_y;

    for (int i = 0; i < path_size; ++i) {
      double delta_speed = accel * delta_t;
      // speed control
      if (speed + delta_speed < target_speed_) {
        // speed up if less than target speed
        speed += delta_speed;  // v = v + a * t
      } else if (speed - delta_speed > target_speed_) {
        // slow down if greater than target speed
        speed -= delta_speed;
      } else {
        // maintain target speed
        speed = target_speed_;
      }

      // compute the next waypoint in car coordinate
      cur_x += speed * delta_t;  // s = s + v * t
      cur_y = sp(cur_x);

      // transform back to map coordinate
      double cur_x_map = (cur_x * cos(yaw) - cur_y * sin(yaw));
      double cur_y_map = (cur_x * sin(yaw) + cur_y * cos(yaw));
      cur_x_map += ref_x;
      cur_y_map += ref_y;

      // add the waypoint to the result
      next_x_vals.push_back(cur_x_map);
      next_y_vals.push_back(cur_y_map);
    }
  }

  // map data
  vector<double> map_waypoints_x_;
  vector<double> map_waypoints_y_;
  vector<double> map_waypoints_s_;
  vector<double> map_waypoints_dx_;
  vector<double> map_waypoints_dy_;

  // localziation data
  LocalizationData ego_state_;

  // previous path data
  vector<double> previous_path_x_;
  vector<double> previous_path_y_;

  // previous path's end s and d values
  double end_path_s_;
  double end_path_d_;

  // sensor fusion data
  vector<vector<double>> sensor_fusion_;

  // behavior planning output
  BehaviorType optimal_behavior_;
  double target_speed_;

  // iteration counter
  int iteration_ = 0;

  // merging vehicle counter
  int num_merging_ = 0;

  // change mid counter
  int num_change_mid_ = 0;
};

}  // namespace planner

#endif  // PLANNER