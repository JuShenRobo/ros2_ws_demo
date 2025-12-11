/**
* feature.h
* The Motion Planner feature
*
* Created By Tony Huang @ 2014-5-22
* Copyright (c) 2014 Shanghai SlamTec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/feature.h>
#include <rpos/core/pose.h>
#include <rpos/core/geometry.h>
#include <rpos/core/pose_entry.h>
#include <rpos/features/system_resource/laser_scan.h>
#include "move_action.h"
#include "velocity_control_move_action.h"

#include <boost/optional.hpp>

namespace rpos {
    namespace features {

        namespace detail {
            class MotionPlannerImpl;
        }

        namespace motion_planner {

            enum NavigationMode
            {
                NavigationModeFree = 0,
                NavigationModeStrictVirtualTrack,
                NavigationModePriorityVirtualTrack,
                NavigationModeFollowPathPoints,
                NavigationModeReverseWalk,
                NavigationModeStrictVirtualTrackReverseWalk
            };

            enum MoveOptionFlag
            {
                MoveOptionFlagNone = 0,
                MoveOptionFlagAppending = 1,
                MoveOptionFlagMilestone = 2,                    //deprecated
                MoveOptionFlagNoSmooth = 4,
                MoveOptionFlagKeyPoints = 8,                    //deprecated
                MoveOptionFlagPrecise = 16,
                MoveOptionFlagWithYaw = 32,
                MoveOptionFlagReturnUnreachableDirectly = 64,
                MoveOptionFlagKeyPointsWithOA = 128,            //deprecated
                MoveOptionFlagWithFailRetryCount = 512,
                MoveOptionFlagFindPathIgnoringDynamicObstacles = 1024,
                MoveOptionFlagWithDirectedVirtualTrack = 2048
            };

            struct RPOS_CORE_API MoveOptions
            {
                MoveOptions();
                MoveOptions(NavigationMode optionMode, MoveOptionFlag optionFlag);
                MoveOptions& operator = (const MoveOptions& other)
                {
                    this->mode = other.mode;
                    this->flag = other.flag;
                    this->speed_ratio = other.speed_ratio;
                    this->fail_retry_count = other.fail_retry_count;
                    this->yaw = other.yaw;
                    this->acceptable_precision = other.acceptable_precision;
                    return *this;
                }

                NavigationMode mode;
                MoveOptionFlag flag;
                boost::optional<double> speed_ratio;
                boost::optional<int> fail_retry_count;
                boost::optional<double> yaw;
                boost::optional<double> acceptable_precision;

                bool hasFlag(MoveOptionFlag flag);
                void addFlag(MoveOptionFlag flag);
            };

            enum RecoverLocalizationMovement
            {
                RecoverLocalizationMovementUnknown,
                RecoverLocalizationMovementNoMove,
                RecoverLocalizationMovementRotateOnly,
                RecoverLocalizationMovementAny,
                RecoverLocalizaitonMovementManual
            };

            struct RPOS_CORE_API RecoverLocalizationOptions
            {
                RecoverLocalizationOptions(boost::optional<int> ms = boost::none);
                boost::optional<int> maxRecoverTimeInMilliSeconds;
                boost::optional<RecoverLocalizationMovement> recoverMovementType;
                boost::optional<rpos::features::system_resource::LaserScan> observation;
            };

            enum GoHomeFlag {
                Dock,
                NoDock
            };

            struct RPOS_CORE_API GoHomeOptions
            {
                GoHomeOptions();
                GoHomeOptions(GoHomeFlag flag, MoveOptions optionFlag, bool enableBack, int retryCnt);

                GoHomeFlag flag;
                MoveOptions move_options;
                boost::optional<bool> enable_back_to_landing_point_after_fail;
                boost::optional<int> charging_base_fail_retry_count;
                boost::optional<float> stop_distance_if_no_dock;      //if flag is NoDock, this indicates where robot should stop
            };

            enum VelocityControlFlag {
                MonitoredByLocalizationQuality = 0,
                NotMonitored
                };

            enum ElevatorDoor {
                ElevatorDoorFront,
                ElevatorDoorRear
            };

            enum ElevatorStoppingYaw {
                FaceToFrontDoor,
                FaceToRearDoor
            };

            enum PoseToElevatorCalcFlag {
                PoseToElevatorCalcFlagWithPoint,
                PoseToElevatorCalcFlagWithBlock
            };

            enum PositionalRelationToElevator {
                UnKnown = 0,
                InElevator,
                NearElevatorSill,
                OutOfElevator
            };
            
            struct RPOS_CORE_API EnterElevatorActionOptions
            {
                EnterElevatorActionOptions();
                boost::optional<int> timeout_in_ms;
                motion_planner::ElevatorDoor elevator_door_flag;
                motion_planner::ElevatorStoppingYaw elevator_stopping_yaw;
                boost::optional<bool> use_conservative_mode;
                MoveOptionFlag move_flag;
            };

            struct RPOS_CORE_API LeaveElevatorActionOptions
            {
                LeaveElevatorActionOptions(); 
                boost::optional<int> timeout_in_ms;
                boost::optional<int> arrive_door_timeout_in_ms;
                boost::optional<int> search_path_timeout_in_ms;
                boost::optional<int> on_elevator_door_timeout_in_ms;
                motion_planner::ElevatorDoor elevator_door_flag;
                bool if_need_reach_milestone;
                MoveOptions move_options;
            };

            struct RPOS_CORE_API RotateActionOptions
            {
                RotateActionOptions();
                boost::optional<int> timeout_in_ms;
                bool if_rotate_circumcircle_collide;
                bool wait_after_can_rotate;
            };

            enum TagType {
                TagTypeVisualTag = 0,
                TagTypeLaserTag,
                TagTypeLaserReflectTag
            };

            struct RPOS_CORE_API MoveToTagOptions
            {
                MoveToTagOptions();
                MoveToTagOptions(TagType type);

                TagType tag_type;
                /*
                * required relative location to target tag, x for vertical distance, y for lateral deviation
                */
                boost::optional<rpos::core::Pose> target_relative_pose;
                /*
                * reverse move to tag if it's true
                */
                bool backward_docking;
                /*
                * after robot stoped, rotate specified radian, turn left if it's greater than 0, otherwise turn right
                */
                float turn_radian;
                /*
                * id of visual tag, only for MoveToVisualTag
                */
                std::vector<int32_t> tag_ids;          
                /*
                * number of laser reflectors(1 or 2), only for MoveToLaserReflectTag
                */
                boost::optional<int> reflect_tag_num;
            };

            enum BackUpModeFromTag {
                BackUpFree = 0,
                BackUpInNarrowCorridor
            };

            struct RPOS_CORE_API BackFromTagOptions
            {
                BackFromTagOptions();
                BackFromTagOptions(BackUpModeFromTag mode, TagType type);
                BackUpModeFromTag back_mode;
                TagType tag_type;
                boost::optional<float> back_up_distance; 
                /*
                * if backward docking is true, then actually moving forward when backUpAfterMoveToTag invoked
                */
                bool backward_docking; 
            };

            enum CheckOccupiedKinds
            {
                CheckOccupiedKindsLocal = 1,
                CheckOccupiedKindsGlobal = 2,
            };
        }

        class RPOS_CORE_API MotionPlanner : public rpos::core::Feature{
        public:
            typedef detail::MotionPlannerImpl impl_t;

            RPOS_OBJECT_CTORS_WITH_BASE(MotionPlanner, rpos::core::Feature);
            MotionPlanner(boost::shared_ptr<detail::MotionPlannerImpl> impl);
            ~MotionPlanner();

        public:
            rpos::actions::MoveAction goHome(const rpos::features::motion_planner::GoHomeOptions& options);
            rpos::actions::MoveAction goHome();
            rpos::actions::MoveAction moveTo(const std::vector<rpos::core::Location>& locations, bool appending);
            rpos::actions::MoveAction moveTo(const rpos::core::Location& location, bool appending); 
            rpos::actions::MoveAction moveTo(const std::vector<rpos::core::Location>& locations, const motion_planner::MoveOptions& options );
            rpos::actions::MoveAction moveTo(const rpos::core::Location& location, const motion_planner::MoveOptions& options);
            rpos::actions::MoveAction moveBy(const rpos::core::Direction& direction);
            rpos::actions::MoveAction moveBy(const rpos::core::Direction& direction, const motion_planner::MoveOptions& options);
            rpos::actions::MoveAction moveBy(float theta, const motion_planner::MoveOptions& options);
            rpos::actions::MoveAction rotateTo(const rpos::core::Rotation& orientation);
            rpos::actions::MoveAction rotateTo(const rpos::core::Rotation& orientation, const motion_planner::RotateActionOptions& options);
            rpos::actions::MoveAction rotate(const rpos::core::Rotation& rotation);
            rpos::actions::MoveAction rotate(const rpos::core::Rotation& rotation, const motion_planner::RotateActionOptions& options);
            rpos::actions::MoveAction recoverLocalization(const core::RectangleF& area, const motion_planner::RecoverLocalizationOptions& options);
            rpos::actions::MoveAction recoverLocalizationByDock();
            void recoverLocalizationByManual(const core::RectangleF& area, const motion_planner::RecoverLocalizationOptions& options);

            virtual rpos::actions::VelocityControlMoveAction velocityControl(rpos::features::motion_planner::VelocityControlFlag flag = rpos::features::motion_planner::MonitoredByLocalizationQuality);
            rpos::actions::MoveAction getCurrentAction();
            rpos::features::motion_planner::Path searchPath(const rpos::core::Location& location, int timeoutMs);
            std::vector<std::string> enumSupportedMotionStrategies();
            bool setMotionStrategy(const std::string& strategy);
            std::string getCurrentMotionStrategy();
            std::vector<std::string> getElevatorIDList();
            //vector[0]: center scheduling pose, others are additional scheduling poses
            std::vector<rpos::core::Pose> getElevatorSchedulingPoses(const std::string& elevatorID, motion_planner::ElevatorDoor doorFlag = motion_planner::ElevatorDoorFront);
            rpos::features::motion_planner::PositionalRelationToElevator getPoseRelationToElevator(const std::string& elevatorID, motion_planner::PoseToElevatorCalcFlag poseCalcFlag);
            rpos::actions::MoveAction enterElevator(const std::string& elevatorID);
            rpos::actions::MoveAction enterElevator(const std::string& elevatorID, const features::motion_planner::EnterElevatorActionOptions& options);
            rpos::actions::MoveAction leaveElevator(const std::string& elevatorID, const core::Location& location);
            rpos::actions::MoveAction leaveElevator(const std::string& elevatorID, const core::Location& location, const features::motion_planner::LeaveElevatorActionOptions& options);
            rpos::actions::MoveAction moveToTag(const rpos::core::VisualTagPose& tagPose, const boost::optional<rpos::core::Pose>& targetPoseToTag);
            rpos::actions::MoveAction moveToTag(const rpos::core::Pose& dockingPose, const motion_planner::MoveToTagOptions& options);
            rpos::actions::MoveAction backUpAfterMoveToTag(const features::motion_planner::BackFromTagOptions &options);
            std::vector<rpos::core::Location> doMultTaskDispatch(boost::optional<rpos::core::Location> & startPoint, std::vector<rpos::core::Location> passPoints,boost::optional<rpos::core::Location>& endPoint);
            bool searchElevatorPath(const std::string& elevatorID, int timeoutInMs);
            bool checkOccupied(const rpos::core::Pose& pose, rpos::features::motion_planner::CheckOccupiedKinds kinds);
            rpos::message::base::MotionRequest getSpeed();
        };

    }
}
