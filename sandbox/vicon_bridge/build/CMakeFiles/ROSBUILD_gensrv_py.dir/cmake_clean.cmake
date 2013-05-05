FILE(REMOVE_RECURSE
  "../src/vicon_bridge/msg"
  "../src/vicon_bridge/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/vicon_bridge/srv/__init__.py"
  "../src/vicon_bridge/srv/_viconCalibrateSegment.py"
  "../src/vicon_bridge/srv/_viconGrabPose.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
