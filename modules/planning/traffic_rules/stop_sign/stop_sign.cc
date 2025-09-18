/*****************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/traffic_rules/stop_sign/stop_sign.h"

#include <memory>

#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/common.h"
#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::hdmap::PathOverlap;

bool StopSign::Init(const std::string& name,
                    const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  // Load the config this task.
  return TrafficRule::LoadConfig<StopSignConfig>(&config_);
}

Status StopSign::ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info) {
  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}

void StopSign::MakeDecisions(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!config_.enabled()) {
    return;
  }

  const auto& stop_sign_status =
      injector_->planning_context()->planning_status().stop_sign();
  const double adc_back_edge_s = reference_line_info->AdcSlBoundary().start_s();

  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info->reference_line().map_path().stop_sign_overlaps();
  for (const auto& stop_sign_overlap : stop_sign_overlaps) {
    if (stop_sign_overlap.end_s <= adc_back_edge_s) {
      continue;
    }

    if (stop_sign_overlap.object_id ==
        stop_sign_status.done_stop_sign_overlap_id()) {
      continue;
    }

    // MODIFICATION FOR ISCC 2025: Implement precision stop for stop sign (2.0-2.5m)
    // 计算车辆前端到后轴中心的距离补偿
    const auto& vehicle_config = common::VehicleConfigHelper::GetConfig();
    double front_edge_to_center = vehicle_config.vehicle_param().front_edge_to_center();
    
    // ISCC竞赛要求：停车在停止线前2.0-2.5米，选择2.25米作为目标
    double target_stop_distance = 2.25;
    double adjusted_stop_distance = target_stop_distance + front_edge_to_center;
    
    AINFO << "[ISCC_StopSign] Precision stop: target=" << target_stop_distance 
          << "m, front_edge_compensation=" << front_edge_to_center 
          << "m, final_stop_distance=" << adjusted_stop_distance << "m";

    // build stop decision
    ADEBUG << "BuildStopDecision: stop_sign[" << stop_sign_overlap.object_id
           << "] start_s[" << stop_sign_overlap.start_s << "]";
    const std::string virtual_obstacle_id =
        STOP_SIGN_VO_ID_PREFIX + stop_sign_overlap.object_id;
    const std::vector<std::string> wait_for_obstacle_ids(
        stop_sign_status.wait_for_obstacle_id().begin(),
        stop_sign_status.wait_for_obstacle_id().end());
    util::BuildStopDecision(
        virtual_obstacle_id, stop_sign_overlap.start_s, adjusted_stop_distance,
        StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids, Getname(),
        frame, reference_line_info);
  }
}

}  // namespace planning
}  // namespace apollo

        STOP_SIGN_VO_ID_PREFIX + stop_sign_overlap.object_id;
    const std::vector<std::string> wait_for_obstacle_ids(
        stop_sign_status.wait_for_obstacle_id().begin(),
        stop_sign_status.wait_for_obstacle_id().end());
    util::BuildStopDecision(
        virtual_obstacle_id, stop_sign_overlap.start_s, adjusted_stop_distance,
        StopReasonCode::STOP_REASON_STOP_SIGN, wait_for_obstacle_ids, Getname(),
        frame, reference_line_info);
  }
}

}  // namespace planning
}  // namespace apollo
