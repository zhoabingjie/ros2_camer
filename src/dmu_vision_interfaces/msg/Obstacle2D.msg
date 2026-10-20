# One detected obstacle candidate in the source image.
#
# Coordinates and sizes are pixels only. This message intentionally has no
# distance, depth or 3D position field: a monocular 2D detector cannot provide
# those values without another calibrated measurement source.

int32 class_id
string class_name
float64 score
string track_id

float64 center_x
float64 center_y
float64 width
float64 height
