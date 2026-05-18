#ifndef JIE_OCTOMAP__PCD_IO_MINIMAL_HPP_
#define JIE_OCTOMAP__PCD_IO_MINIMAL_HPP_

#include <array>
#include <string>
#include <vector>

namespace dong_octomap_pcd
{

/// Load 3D points (x,y,z) from a PCD file (ASCII, binary, or binary_compressed).
/// Supports common PCD v0.7 with fields x y z (additional fields ignored).
/// @return false on parse/read error; @p err_msg is set for logging.
bool load_pcd_xyz(
  const std::string & path,
  std::vector<std::array<double, 3>> & out_points,
  std::string & err_msg);

}  // namespace dong_octomap_pcd

#endif  // JIE_OCTOMAP__PCD_IO_MINIMAL_HPP_
