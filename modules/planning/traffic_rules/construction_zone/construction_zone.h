// 文件: modules/planning/traffic_rules/construction_zone/construction_zone.h
// 目的：ISCC 2025 施工区域交通规则类定义
// 功能：实现施工区域限速和锥桶避让逻辑

#pragma once

#include <memory>
#include <string>

#include "modules/planning/traffic_rules/construction_zone/proto/construction_zone.pb.h"
#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"

namespace apollo {
namespace planning {

class ConstructionZone : public TrafficRule {
 public:
  bool Init(const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;
  virtual ~ConstructionZone() = default;
  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);
  void Reset() override {}

 private:
  ConstructionZoneConfig config_;

  // ISCC 2025: 检查是否在施工区域内
  bool IsInConstructionZone(const ReferenceLineInfo& reference_line_info) const;

  // ISCC 2025: 应用施工区域限速
  void ApplySpeedLimit(ReferenceLineInfo* reference_line_info) const;

  // ISCC 2025: 收缩路径边界以避免锥桶
  void ContractPathBounds(ReferenceLineInfo* reference_line_info) const;

  static constexpr char const* CONSTRUCTION_ZONE_VO_ID_PREFIX = "CZ_";
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ConstructionZone,
                                     TrafficRule)

}  // namespace planning
}  // namespace apollo
