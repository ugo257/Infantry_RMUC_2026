# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "Infantry_RMUC_2026.bin"
  "Infantry_RMUC_2026.hex"
  "Infantry_RMUC_2026.map"
  )
endif()
