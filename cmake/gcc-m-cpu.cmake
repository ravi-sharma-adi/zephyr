# SPDX-License-Identifier: Apache-2.0

# Determines what argument to give to -mcpu= based on the
# KConfig'uration and sets this to GCC_M_CPU

if("${ARCH}" STREQUAL "arm")
  if    (CONFIG_CPU_CORTEX_M0)
    set(GCC_M_CPU cortex-m0)
  elseif(CONFIG_CPU_CORTEX_M0PLUS)
    set(GCC_M_CPU cortex-m0plus)
  elseif(CONFIG_CPU_CORTEX_M3)
    set(GCC_M_CPU cortex-m3)
  elseif(CONFIG_CPU_CORTEX_M4)
    set(GCC_M_CPU cortex-m4)
    if(NOT CONFIG_CPU_HAS_VFP)
      set(GCC_M_CPU ${GCC_M_CPU}+nofp)
    endif()
  elseif(CONFIG_CPU_CORTEX_M7)
    set(GCC_M_CPU cortex-m7)
    if(CONFIG_CPU_HAS_VFP)
      if(NOT CONFIG_VFP_FEATURE_DOUBLE_PRECISION)
        set(GCC_M_CPU ${GCC_M_CPU}+nofp.dp)
      endif()
    else()
      set(GCC_M_CPU ${GCC_M_CPU}+nofp)
    endif()
  elseif(CONFIG_CPU_CORTEX_M23)
    set(GCC_M_CPU cortex-m23)
  elseif(CONFIG_CPU_CORTEX_M33)
    set(GCC_M_CPU cortex-m33)
    if(NOT CONFIG_CPU_HAS_VFP)
      set(GCC_M_CPU ${GCC_M_CPU}+nofp)
    endif()
    if(NOT CONFIG_CPU_CORTEX_M_HAS_DSP)
      set(GCC_M_CPU ${GCC_M_CPU}+nodsp)
    endif()
  elseif(CONFIG_CPU_CORTEX_R4)
    if(CONFIG_CPU_HAS_VFP)
      set(GCC_M_CPU cortex-r4f)
    else()
      set(GCC_M_CPU cortex-r4)
    endif()
  elseif(CONFIG_CPU_CORTEX_R5)
    set(GCC_M_CPU cortex-r5)
    if(CONFIG_CPU_HAS_VFP)
      if(NOT CONFIG_VFP_FEATURE_DOUBLE_PRECISION)
        set(GCC_M_CPU ${GCC_M_CPU}+nofp.dp)
      endif()
    else()
      set(GCC_M_CPU ${GCC_M_CPU}+nofp)
    endif()
  elseif(CONFIG_CPU_CORTEX_A53)
    set(GCC_M_CPU cortex-a53)
  else()
    message(FATAL_ERROR "Expected CONFIG_CPU_CORTEX_x to be defined")
  endif()
elseif("${ARCH}" STREQUAL "arc")
  if(CONFIG_CPU_EM4_FPUS)
    set(GCC_M_CPU em4_fpus)
  elseif(CONFIG_CPU_EM4_DMIPS)
    set(GCC_M_CPU em4_dmips)
  elseif(CONFIG_CPU_EM4_FPUDA)
    set(GCC_M_CPU em4_fpuda)
  elseif(CONFIG_CPU_ARCHS)
    set(GCC_M_CPU hs)
  elseif(CONFIG_CPU_EM4)
    set(GCC_M_CPU arcem)
  elseif(CONFIG_CPU_EM6)
    set(GCC_M_CPU arcem)
  endif()
endif()
