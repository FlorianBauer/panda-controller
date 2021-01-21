#ifndef SERVICEDEFS_H
#define SERVICEDEFS_H

static constexpr char PANDA_ARM[] = "panda_arm";
static constexpr char PANDA_LINK_BASE[] = "panda_link0";
static constexpr char PANDA_LINK_HAND[] = "panda_hand";
static constexpr char PANDA_FINGER_1[] = "panda_finger_joint1";
static constexpr char PANDA_FINGER_2[] = "panda_finger_joint2";
static constexpr char PANDA_SERVICE_NAME[] = "panda_controller_service";
static constexpr char PANDA_CLIENT_NAME[] = "panda_controller_client";
static constexpr char SRV_FOLLOW_FRAMES[] = "follow_frames";
static constexpr char SRV_FOLLOW_PATHS[] = "follow_paths";
static constexpr char SRV_GET_CURRENT_FRAME[] = "get_current_joints";
static constexpr char SRV_GET_CURRENT_POSE[] = "get_current_pose";
static constexpr char SRV_MOVE_TO_POSE[] = "move_to_pose";
static constexpr char SRV_SET_TO_FRAME[] = "set_to_frame";
static constexpr size_t MAX_JOINTS = 7;

#endif /* SERVICEDEFS_H */
