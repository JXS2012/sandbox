FILE(REMOVE_RECURSE
  "../src/vicon_bridge/msg"
  "../src/vicon_bridge/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/vicon_bridge/viconCalibrateSegment.h"
  "../srv_gen/cpp/include/vicon_bridge/viconGrabPose.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
