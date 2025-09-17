/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file stage_stop.cc
 **/

#include "modules/planning/scenarios/stop_sign_unprotected/stage_stop.h"

#include <algorithm>
#include <utility>

#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/util.h"
#include "modules/planning/planning_base/common/vehicle_config_helper.h"
#include "modules/planning/planning_base/reference_line/reference_line.h"
#include "modules/planning/scenarios/stop_sign_unprotected/context.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::cyber::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneInfoConstPtr;
using apollo::hdmap::OverlapInfoConstPtr;
using apollo::hdmap::PathOverlap;
using apollo::perception::PerceptionObstacle;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

using StopSignLaneVehicles =
    std::unordered_map<std::string, std::vector<std::string>>;

StageResult StopSignUnprotectedStageStop::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Stop";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  auto context = GetContextAs<StopSignUnprotectedContext>();
  const ScenarioStopSignUnprotectedConfig& scenario_config =
      context->scenario_config;

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "StopSignUnprotectedPreStop planning error";
  }

  const auto& reference_line_info = frame->reference_line_info().front();
  std::string stop_sign_overlap_id = context->current_stop_sign_overlap_id;

  // refresh overlap along reference line
  PathOverlap* current_stop_sign_overlap =
      reference_line_info.GetOverlapOnReferenceLine(
          stop_sign_overlap_id, ReferenceLineInfo::STOP_SIGN);
  if (!current_stop_sign_overlap) {
    return FinishScenario();
  }

  // set right_of_way_status
  const double stop_sign_start_s = current_stop_sign_overlap->start_s;
  reference_line_info.SetJunctionRightOfWay(stop_sign_start_s, false);

  static constexpr double kPassStopLineBuffer = 1.0;  // unit: m
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double distance_adc_pass_stop_sign =
      adc_front_edge_s - stop_sign_start_s;
  // passed stop line too far
  if (distance_adc_pass_stop_sign > kPassStopLineBuffer) {
    return FinishStage();
  }

  // ISCC 2025 竞赛专用：智能"清场等待"决策逻辑
  // 移除基于时间的简单判断，转而使用时空预测的智能决策

  // 调用智能清场检测函数
  bool intersection_clear = IsIntersectionClear(*frame, reference_line_info);

  if (intersection_clear) {
    ADEBUG << "Intersection is clear - proceeding to next stage";
    return FinishStage();
  } else {
    ADEBUG << "Intersection is not clear - continuing to wait";
    // 保留原有的 watch_vehicles 可视化逻辑
    auto& watch_vehicles = context->watch_vehicles;

    // get all vehicles currently watched for visualization
    std::vector<std::string> watch_vehicle_ids;
    for (const auto& watch_vehicle : watch_vehicles) {
      std::copy(watch_vehicle.second.begin(), watch_vehicle.second.end(),
                std::back_inserter(watch_vehicle_ids));
      // for debug
      std::string s;
      for (const std::string& vehicle : watch_vehicle.second) {
        s = s.empty() ? vehicle : s + "," + vehicle;
      }
      const std::string& associated_lane_id = watch_vehicle.first;
      ADEBUG << "watch_vehicles: lane_id[" << associated_lane_id << "] vehicle["
             << s << "]";
    }

    // remove duplicates
    watch_vehicle_ids.erase(
        unique(watch_vehicle_ids.begin(), watch_vehicle_ids.end()),
        watch_vehicle_ids.end());

    // pass vehicles being watched to DECIDER_RULE_BASED_STOP task for visualization
    for (const auto& perception_obstacle_id : watch_vehicle_ids) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_stop_sign()
          ->add_wait_for_obstacle_id(perception_obstacle_id);
    }

    const PathDecision& path_decision = reference_line_info.path_decision();
    RemoveWatchVehicle(path_decision, &watch_vehicles);

    // 继续等待，确保停车虚拟墙仍然生效
    return result.SetStageStatus(StageStatusType::RUNNING);
  }
}

/**
 * @brief: remove a watch vehicle which not stopping at stop sign any more
 */
int StopSignUnprotectedStageStop::RemoveWatchVehicle(
    const PathDecision& path_decision, StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);
  auto context = GetContextAs<StopSignUnprotectedContext>();
  for (auto& vehicle : *watch_vehicles) {
    // associated_lane/stop_sign info
    std::string associated_lane_id = vehicle.first;
    auto assoc_lane_it = std::find_if(
        context->associated_lanes.begin(), context->associated_lanes.end(),
        [&associated_lane_id](
            std::pair<LaneInfoConstPtr, OverlapInfoConstPtr>& assc_lane) {
          return assc_lane.first.get()->id().id() == associated_lane_id;
        });
    if (assoc_lane_it == context->associated_lanes.end()) {
      continue;
    }
    auto stop_sign_over_lap_info =
        assoc_lane_it->second.get()->GetObjectOverlapInfo(
            hdmap::MakeMapId(associated_lane_id));
    if (stop_sign_over_lap_info == nullptr) {
      AERROR << "can't find stop_sign_over_lap_info for id: "
             << associated_lane_id;
      continue;
    }
    const double stop_line_end_s =
        stop_sign_over_lap_info->lane_overlap_info().end_s();

    const auto lane =
        HDMapUtil::BaseMap().GetLaneById(hdmap::MakeMapId(associated_lane_id));
    if (lane == nullptr) {
      continue;
    }
    auto stop_sign_point = lane.get()->GetSmoothPoint(stop_line_end_s);

    std::vector<std::string> remove_vehicles;
    auto& vehicles = vehicle.second;
    for (const auto& perception_obstacle_id : vehicles) {
      // watched-vehicle info
      const PerceptionObstacle* perception_obstacle =
          path_decision.FindPerceptionObstacle(perception_obstacle_id);
      if (!perception_obstacle) {
        ADEBUG << "mark ERASE obstacle_id[" << perception_obstacle_id
               << "] not exist";
        remove_vehicles.push_back(perception_obstacle_id);
        continue;
      }

      PerceptionObstacle::Type obstacle_type = perception_obstacle->type();
      std::string obstacle_type_name =
          PerceptionObstacle_Type_Name(obstacle_type);
      auto obstacle_point = common::util::PointFactory::ToPointENU(
          perception_obstacle->position());

      double distance =
          common::util::DistanceXY(stop_sign_point, obstacle_point);
      ADEBUG << "obstacle_id[" << perception_obstacle_id << "] distance["
             << distance << "]";

      // TODO(all): move 10.0 to conf
      if (distance > 10.0) {
        ADEBUG << "mark ERASE obstacle_id[" << perception_obstacle_id << "]";
        remove_vehicles.push_back(perception_obstacle_id);
      }
    }
    for (const auto& perception_obstacle_id : remove_vehicles) {
      ADEBUG << "ERASE obstacle_id[" << perception_obstacle_id << "]";
      vehicles.erase(
          std::remove(vehicles.begin(), vehicles.end(), perception_obstacle_id),
          vehicles.end());
    }
  }

  return 0;
}

StageResult StopSignUnprotectedStageStop::FinishStage() {
  auto context = GetContextAs<StopSignUnprotectedContext>();
  // update PlanningContext
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_stop_sign()
      ->set_done_stop_sign_overlap_id(context->current_stop_sign_overlap_id);
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_stop_sign()
      ->clear_wait_for_obstacle_id();

  context->creep_start_time = Clock::NowInSeconds();

  next_stage_ = "STOP_SIGN_UNPROTECTED_CREEP";
  return StageResult(StageStatusType::FINISHED);
}

bool StopSignUnprotectedStageStop::IsIntersectionClear(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) const {
  ADEBUG << "Checking if intersection is clear for ISCC 2025 competition";

  // 1. 定义主车冲突区域 (Define Ego's Conflict Zone)
  const auto& path_data = reference_line_info.path_data();
  const auto& discretized_path = path_data.discretized_path();

  if (discretized_path.empty()) {
    ADEBUG << "No discretized path available";
    return false;
  }

  // 获取车辆参数
  const auto& vehicle_config = VehicleConfigHelper::Instance()->GetConfig();
  const double vehicle_width = vehicle_config.vehicle_param().width();

  // 构建主车冲突区域的多边形
  std::vector<Vec2d> ego_conflict_zone_points;

  for (const auto& path_point : discretized_path) {
    // 在每个路径点处根据车辆宽度向两侧扩展
    const double half_width = vehicle_width / 2.0;
    const double theta = path_point.theta();

    // 计算垂直于前进方向的向量
    Vec2d lateral_dir(-std::sin(theta), std::cos(theta));

    // 生成冲突区域的四个角点（简化版，只考虑前后扩展）
    Vec2d center(path_point.x(), path_point.y());
    Vec2d left_extend = center + lateral_dir * half_width;
    Vec2d right_extend = center - lateral_dir * half_width;

    ego_conflict_zone_points.push_back(left_extend);
    ego_conflict_zone_points.push_back(right_extend);
  }

  // 简化版：使用最小外接矩形作为冲突区域
  if (ego_conflict_zone_points.empty()) {
    return false;
  }

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto& point : ego_conflict_zone_points) {
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x, point.x());
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  }

  Box2d ego_conflict_zone({(min_x + max_x) / 2.0, (min_y + max_y) / 2.0},
                         max_x - min_x, max_y - min_y, 0.0);

  // 2. 获取所有动态障碍物 (Get all dynamic obstacles)
  const auto& path_decision = reference_line_info.path_decision();
  const auto& obstacles = path_decision.obstacles();

  // 3. 时空碰撞检查 (Spatio-Temporal Collision Check)
  constexpr double kPredictionTimeHorizon = 5.0;  // 5秒预测时域
  constexpr double kTimeStep = 0.5;  // 0.5秒时间步长

  for (const auto& obstacle : obstacles) {
    // 只关注动态障碍物
    if (obstacle->IsStatic()) {
      continue;
    }

    // 检查障碍物类型
    const auto& perception_obstacle = obstacle->Perception();
    if (perception_obstacle.type() != PerceptionObstacle::VEHICLE &&
        perception_obstacle.type() != PerceptionObstacle::PEDESTRIAN &&
        perception_obstacle.type() != PerceptionObstacle::BICYCLE) {
      continue;
    }

    // 检查是否有预测轨迹
    if (!obstacle->HasTrajectory()) {
      ADEBUG << "Obstacle " << obstacle->Id() << " has no trajectory, assuming potential threat";
      return false;  // 如果没有轨迹，为了安全起见认为不安全
    }

    // 获取预测轨迹
    const auto& trajectory = obstacle->Trajectory();

    // 检查未来时间点是否有碰撞
    for (double t = 0.0; t <= kPredictionTimeHorizon; t += kTimeStep) {
      if (t >= trajectory.trajectory_point_size()) {
        break;  // 轨迹结束
      }

      const auto& trajectory_point = trajectory.trajectory_point(static_cast<int>(t / kTimeStep));

      // 获取障碍物的预测包围盒
      Box2d obstacle_box = obstacle->GetBoundingBox(trajectory_point);

      // 检查是否与主车冲突区域重叠
      if (ego_conflict_zone.HasOverlap(obstacle_box)) {
        ADEBUG << "Collision detected at time " << t << " with obstacle "
               << obstacle->Id() << " at position ("
               << trajectory_point.path_point().x() << ", "
               << trajectory_point.path_point().y() << ")";
        return false;  // 检测到碰撞，不安全
      }
    }
  }

  ADEBUG << "Intersection is clear - no collision threats detected";
  return true;  // 没有检测到任何威胁，交叉口安全
}

}  // namespace planning
}  // namespace apollo
