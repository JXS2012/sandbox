FILE(REMOVE_RECURSE
  "../src/vicon_bridge/msg"
  "../src/vicon_bridge/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/msvc_bridge.dir/src/msvc_bridge.o"
  "../lib/libmsvc_bridge.pdb"
  "../lib/libmsvc_bridge.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/msvc_bridge.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
