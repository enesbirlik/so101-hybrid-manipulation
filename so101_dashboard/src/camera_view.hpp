// Camera panel: any sensor_msgs/CompressedImage topic (the wrist camera now, processed images later).
#pragma once

#include <raylib.h>

#include "ros_link.hpp"

namespace so101_dashboard
{

class CameraView
{
public:
  ~CameraView();   // before CloseWindow
  void draw(State & state, bool * open);

private:
  void upload(const CameraFrame & frame);

  Texture2D texture_{};
  uint64_t version_ = 0;
  bool crosshair_ = false;
};

}  // namespace so101_dashboard
