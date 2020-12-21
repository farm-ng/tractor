#ifndef FARM_NG_CALIBRATION_POSE_UTILS_H_
#define FARM_NG_CALIBRATION_POSE_UTILS_H_
#include <string>

namespace farm_ng {
namespace perception {

// Constructs a frame name, using the convention <name>/<number %05d><kind>
// Kind can be used here for things like "_depthmap", or "_debug".
std::string FrameNameNumber(const std::string& name, int number,
                            std::string kind = "");

// Constructs a frame name of a tag associated with a rig in the form
// <rig_name>/tag/<tag_id %05d>
std::string FrameRigTag(const std::string& rig_name, int tag_id);

bool StartsWith(const std::string& x, const std::string& prefix);

}  // namespace perception
}  // namespace farm_ng

#endif  // FARM_NG_CALIBRATION_CAMERA_MODEL_H_
