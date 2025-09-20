load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:apollo_cc.bzl", "apollo_cc_library", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "construction_zone",
    srcs = [
        "construction_zone.cc",
    ],
    hdrs = [
        "construction_zone.h",
    ],
    deps = [
        "//modules/planning/traffic_rules/construction_zone/proto:construction_zone_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/planning_interface_base/traffic_rules_base:apollo_planning_planning_interface_base_traffic_rules_base",
        "//modules/common_msgs/planning_msgs:apollo_common_planning",
        "//modules/common/util:apollo_common_util",
        "//modules/planning/planning_base/common:apollo_planning_planning_base_common",
        "@eigen",
    ],
)

apollo_package()

filegroup(
    name = "conf",
    srcs = [
        "conf/default_conf.pb.txt",
    ],
)
