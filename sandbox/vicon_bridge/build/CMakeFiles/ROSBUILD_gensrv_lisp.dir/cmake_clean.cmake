FILE(REMOVE_RECURSE
  "../src/vicon_bridge/msg"
  "../src/vicon_bridge/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/viconCalibrateSegment.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_viconCalibrateSegment.lisp"
  "../srv_gen/lisp/viconGrabPose.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_viconGrabPose.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
