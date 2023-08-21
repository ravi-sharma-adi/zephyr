# SPDX-License-Identifier: Apache-2.0

foreach(extra_flags EXTRA_CPPFLAGS EXTRA_LDFLAGS EXTRA_CFLAGS EXTRA_CXXFLAGS EXTRA_AFLAGS)
  zephyr_get(${extra_flags} MERGE ARGLIST SYSBUILD GLOBAL)
endforeach()

if(EXTRA_CPPFLAGS)
  zephyr_compile_options(${EXTRA_CPPFLAGS})
endif()
if(EXTRA_LDFLAGS)
  zephyr_link_libraries(${EXTRA_LDFLAGS})
endif()
if(EXTRA_CFLAGS)
  foreach(F ${EXTRA_CFLAGS})
    zephyr_compile_options($<$<COMPILE_LANGUAGE:C>:${F}>)
  endforeach()
endif()
if(EXTRA_CXXFLAGS)
  foreach(F ${EXTRA_CXXFLAGS})
    zephyr_compile_options($<$<COMPILE_LANGUAGE:CXX>:${F}>)
  endforeach()
endif()
if(EXTRA_AFLAGS)
  foreach(F ${EXTRA_AFLAGS})
    zephyr_compile_options($<$<COMPILE_LANGUAGE:ASM>:${F}>)
  endforeach()
endif()
