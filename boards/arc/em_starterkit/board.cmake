set(FLASH_SCRIPT arc_debugger.sh)
set(DEBUG_SCRIPT arc_debugger.sh)

set_property(GLOBAL APPEND PROPERTY FLASH_SCRIPT_ENV_VARS
  OPENOCD_LOAD_CMD="load_image     ${PROJECT_BINARY_DIR}/${KERNEL_ELF_NAME} ${CONFIG_FLASH_BASE_ADDRESS}"
  OPENOCD_VERIFY_CMD="verify_image ${PROJECT_BINARY_DIR}/${KERNEL_ELF_NAME} ${CONFIG_FLASH_BASE_ADDRESS}"
  )
