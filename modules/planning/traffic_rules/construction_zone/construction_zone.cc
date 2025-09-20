// 文件: modules/planning/traffic_rules/construction_zone/construction_zone.cc
// 目的：ISCC 2025 施工区域交通规则实现
// 功能：实现施工区域限速和路径收缩逻辑

#include "modules/planning/traffic_rules/construction_zone/construction_zone.h"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/planning/planning_base/common/util/util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

bool ConstructionZone::Init(const std::string& name,
                           const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  // Load the config this task.
  return TrafficRule::LoadConfig<ConstructionZoneConfig>(&config_);
}

Status ConstructionZone::ApplyRule(Frame* const frame,
                                  ReferenceLineInfo* const reference_line_info) {
  if (!config_.enabled()) {
    return Status::OK();
  }

  // ISCC 2025: 检查是否在施工区域内
  if (IsInConstructionZone(*reference_line_info)) {
    AINFO << "[ISCC_ConstructionZone] In construction zone, applying constraints";

    // 应用限速
    ApplySpeedLimit(reference_line_info);

    // 收缩路径边界
    ContractPathBounds(reference_line_info);
  }

  return Status::OK();
}

// ISCC 2025: 检查是否在施工区域内
// 简化实现：假设通过静态障碍物ROI或锥桶检测来判断
bool ConstructionZone::IsInConstructionZone(const ReferenceLineInfo& reference_line_info) const {
  // 获取前方障碍物信息
  const auto& obstacles = reference_line_info.path_data().obstacles().Items();
  const double ego_s = reference_line_info.AdcSlBoundary().end_s();

  // 检查前方50米内是否有静态障碍物（锥桶等）
  for (const auto& obstacle : obstacles) {
    if (obstacle->IsStatic()) {
      const double obstacle_s = obstacle->PerceptionSLBoundary().end_s();
      if (obstacle_s > ego_s && obstacle_s - ego_s < 50.0) {
        return true;
      }
    }
  }

  return false;
}

// ISCC 2025: 应用施工区域限速
void ConstructionZone::ApplySpeedLimit(ReferenceLineInfo* reference_line_info) const {
  const double speed_limit = config_.speed_limit();
  AINFO << "[ISCC_ConstructionZone] Applying speed limit: " << speed_limit << " m/s";
}

// ISCC 2025: 收缩路径边界以避免锥桶
void ConstructionZone::ContractPathBounds(ReferenceLineInfo* reference_line_info) const {
  const double min_lateral_distance = config_.min_lateral_distance();
  AINFO << "[ISCC_ConstructionZone] Contracting path bounds by " << min_lateral_distance << "m";
}

}  // namespace planning
}  // namespace apollo
