/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file
 **/

#include "modules/planning/scenarios/valet_parking/valet_parking_scenario.h"

#include "modules/planning/planning_base/common/frame.h"
#include "modules/common/math/polygon2d.h"
#include "modules/planning/scenarios/valet_parking/stage_approaching_parking_spot.h"
#include "modules/planning/scenarios/valet_parking/stage_parking.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleState;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

bool ValetParkingScenario::Init(std::shared_ptr<DependencyInjector> injector,
                                const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioValetParkingConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get config of scenario" << Name();
    return false;
  }
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  CHECK_NOTNULL(hdmap_);
  init_ = true;
  return true;
}

// [核心修改] 重写IsTransferable函数，实现动态车位搜索逻辑
bool ValetParkingScenario::IsTransferable(const Scenario* const other_scenario,
                                          const Frame& frame) {
  // 如果已经处于泊车场景中，则不再重复判断和切换
  if (other_scenario != nullptr && other_scenario->Name() == Name()) {
    return true;
  }

  // 确保至少有一条参考线用于定位和规划
  if (frame.reference_line_info().empty()) {
    return false;
  }

  // 检查是否已收到上游的泊车指令，这是进入泊车区域的标志
  // 注意：我们不再检查具体的parking_spot_id，只检查是否有泊车这个意图
  if (!frame.local_view().planning_command->has_parking_command()) {
    return false;
  }

  std::string target_spot_id;
  // 调用新函数执行搜索、过滤、排序逻辑
  if (FindClosestAvailableSpot(const_cast<Frame*>(&frame), &target_spot_id)) {
    // 如果成功找到一个车位，则将其ID存入场景上下文
    context_.target_parking_spot_id = target_spot_id;
    AINFO << "[ValetParking] Dynamic search successful. Selected spot ID: "
          << target_spot_id;
    // 返回true，通知ScenarioManager可以切换到本场景
    return true;
  }

  AINFO << "[ValetParking] No available parking spot found. "
        << "Cannot transfer to ValetParkingScenario.";
  // 未找到可用车位，不激活本场景
  return false;
}

bool ValetParkingScenario::SearchTargetParkingSpotOnPath(
    const Path& nearby_path, const std::string& target_parking_id,
    PathOverlap* parking_space_overlap) {
  const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
  for (const auto& parking_overlap : parking_space_overlaps) {
    if (parking_overlap.object_id == target_parking_id) {
      *parking_space_overlap = parking_overlap;
      return true;
    }
  }
  return false;
}

bool ValetParkingScenario::CheckDistanceToParkingSpot(
    const Frame& frame, const VehicleState& vehicle_state,
    const Path& nearby_path, const double parking_start_range,
    const PathOverlap& parking_space_overlap) {
  // TODO(Jinyun) parking overlap s are wrong on map, not usable
  const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
  hdmap::Id id;
  double center_point_s, center_point_l;
  id.set_id(parking_space_overlap.object_id);
  ParkingSpaceInfoConstPtr target_parking_spot_ptr =
      hdmap->GetParkingSpaceById(id);
  Vec2d left_bottom_point = target_parking_spot_ptr->polygon().points().at(0);
  Vec2d right_bottom_point = target_parking_spot_ptr->polygon().points().at(1);
  Vec2d right_top_point = target_parking_spot_ptr->polygon().points().at(2);
  Vec2d left_top_point = target_parking_spot_ptr->polygon().points().at(3);
  Vec2d center_point = (left_bottom_point + right_bottom_point +
                        right_top_point + left_top_point) /
                       4.0;
  nearby_path.GetNearestPoint(center_point, &center_point_s, &center_point_l);
  double vehicle_point_s = 0.0;
  double vehicle_point_l = 0.0;
  Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
  nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
  if (std::abs(center_point_s - vehicle_point_s) < parking_start_range) {
    return true;
  }
  return false;
}

// [新功能] 实现检查泊车位是否可用的函数
bool ValetParkingScenario::IsSpotAvailable(
    const ParkingSpaceInfoConstPtr& parking_spot, Frame* frame) {
  if (!parking_spot) {
    return false;
  }

  // 从高精地图获取车位的几何多边形
  const auto& spot_polygon = parking_spot->polygon();

  // 遍历当前帧感知到的所有障碍物
  for (const auto* obstacle : frame->obstacles()) {
    // 忽略虚拟障碍物或置信度低的障碍物
    if (obstacle->IsVirtual()) {
      continue;
    }

    // 获取障碍物的感知多边形
    const Polygon2d& obstacle_polygon = obstacle->PerceptionPolygon();

    // 检查车位多边形与障碍物多边形是否重叠
    // 增加一个小的安全缓冲（buffer）可以提高鲁棒性
    if (spot_polygon.HasOverlap(obstacle_polygon.ExpandByDistance(0.1))) {
      ADEBUG << "Parking spot " << parking_spot->id().id()
             << " is occupied by obstacle " << obstacle->Id();
      return false;  // 发现重叠，车位被占用
    }
  }

  return true;  // 未发现任何重叠，车位可用
}

// [新功能] 实现查找最近可用泊车位的核心逻辑
bool ValetParkingScenario::FindClosestAvailableSpot(Frame* frame,
                                                  std::string* target_spot_id) {
  const auto& reference_line_info = frame->reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  const auto& adc_sl_boundary = reference_line_info.AdcSlBoundary();

  // 1. 获取当前车道及后续车道上的所有泊车位ID
  std::vector<std::string> parking_spot_ids;
  const auto& segments = reference_line_info.Lanes();
  for (const auto& segment : segments.RouteSegments()) {
    for (const auto& overlap : segment.lane->parking_space_overlaps()) {
      parking_spot_ids.push_back(overlap.object_id);
    }
  }

  if (parking_spot_ids.empty()) {
    ADEBUG << "No parking spots found along the current route.";
    return false;
  }

  // 2. 定义入口点s值，这里简化为车辆当前位置的s值
  // 一个更精确的定义可以是第一个包含停车位的车道段的起点s值
  const double entrance_s = adc_sl_boundary.end_s();

  double min_distance = std::numeric_limits<double>::max();
  std::string best_spot_id = "";

  // 3. 遍历所有候选车位，进行过滤和排序
  for (const auto& spot_id_str : parking_spot_ids) {
    hdmap::Id spot_id;
    spot_id.set_id(spot_id_str);
    auto parking_spot_info = hdmap_->GetParkingSpaceById(spot_id);
    if (!parking_spot_info) {
      AWARN << "Failed to get parking spot info for ID: " << spot_id_str;
      continue;
    }

    // 过滤：检查车位是否可用
    if (IsSpotAvailable(parking_spot_info, frame)) {
      // 排序：计算距离
      const auto& spot_polygon = parking_spot_info->polygon();
      const Vec2d spot_center = spot_polygon.center();
      common::SLPoint spot_sl;
      if (!reference_line.XYToSL(spot_center, &spot_sl)) {
        AWARN << "Failed to project parking spot " << spot_id_str
              << " center to reference line.";
        continue;
      }

      // 确保我们只选择前方的车位
      double distance = spot_sl.s() - entrance_s;
      if (distance >= 0 && distance < min_distance) {
        min_distance = distance;
        best_spot_id = spot_id_str;
      }
    }
  }

  // 4. 最终选择
  if (!best_spot_id.empty()) {
    *target_spot_id = best_spot_id;
    return true;
  }

  return false;
}

}  // namespace planning
}  // namespace apollo
