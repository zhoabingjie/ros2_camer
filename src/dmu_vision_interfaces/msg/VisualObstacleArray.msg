# Stable result from the monocular visual obstacle perception node.
#
# STATIC STATUS VALUES:
#   0 UNKNOWN      - not enough information yet
#   1 CLEAR        - fresh detections arrived and no configured obstacle was found
#   2 CANDIDATE    - candidates exist but consecutive-frame confirmation is pending
#   3 OBSTACLE     - confirmed obstacle candidate in the configured ROI
#   4 INPUT_TIMEOUT- detection input stopped
#   5 CAMERA_ERROR - camera diagnostics reported an error
#   6 DETECTOR_ERROR - detector diagnostics reported an error
#
# Obstacles are 2D image-space candidates only. Consumers must not interpret
# their bbox size or position as metric distance.

uint8 STATUS_UNKNOWN=0
uint8 STATUS_CLEAR=1
uint8 STATUS_CANDIDATE=2
uint8 STATUS_OBSTACLE=3
uint8 STATUS_INPUT_TIMEOUT=4
uint8 STATUS_CAMERA_ERROR=5
uint8 STATUS_DETECTOR_ERROR=6

std_msgs/Header header

uint8 status
string status_text
bool input_valid
string input_topic
Obstacle2D[] obstacles
