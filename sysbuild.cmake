set(partition_overlay_dir ${CMAKE_CURRENT_LIST_DIR}/dts/partitions)
set(mcuboot_overlay_dir ${CMAKE_CURRENT_LIST_DIR}/sysbuild)
set(partition_overlay)

if(DEFINED EXTRA_DTC_OVERLAY_FILE)
  list(APPEND ${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE ${EXTRA_DTC_OVERLAY_FILE})
endif()

if(SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_BOARD STREQUAL "holyiot_21017")
  message(FATAL_ERROR
          "holyiot_21017 uses its existing nRF5/OpenDFU boot path; MCUboot is not supported")
elseif(SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_BOARD STREQUAL "xiao_ble")
  set(partition_overlay ${partition_overlay_dir}/nrf52840_xiao_mcuboot.overlay)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE ${partition_overlay})
elseif(SB_CONFIG_BOOTLOADER_MCUBOOT AND
       SB_CONFIG_BOARD STREQUAL "nrf52840dongle")
  set(partition_overlay ${partition_overlay_dir}/nrf52840_dongle_mcuboot.overlay)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE ${partition_overlay})
elseif(SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_SOC_NRF52840)
  set(partition_overlay ${partition_overlay_dir}/nrf52840_mcuboot.overlay)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE ${partition_overlay})
elseif(SB_CONFIG_BOOTLOADER_MCUBOOT AND SB_CONFIG_SOC_NRF52833)
  set(partition_overlay ${partition_overlay_dir}/nrf52833_mcuboot.overlay)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE ${partition_overlay})
elseif(SB_CONFIG_BOARD STREQUAL "xiao_ble")
  set(partition_overlay ${partition_overlay_dir}/nrf52840_xiao.overlay)
elseif(SB_CONFIG_BOARD MATCHES "^nrf52840dongle$|^holyiot_21017$|^holyiot_22046$")
  set(partition_overlay ${partition_overlay_dir}/nrf52840_dongle.overlay)
elseif(SB_CONFIG_SOC_NRF52833)
  set(partition_overlay ${partition_overlay_dir}/nrf52833_uf2.overlay)
elseif(SB_CONFIG_SOC_NRF52840)
  set(partition_overlay ${partition_overlay_dir}/nrf52840_uf2.overlay)
endif()

if(SB_CONFIG_BOOTLOADER_MCUBOOT)
  set_property(TARGET mcuboot APPEND PROPERTY _EP_CMAKE_ARGS
               -DBOARD_DEFCONFIG:FILEPATH=${mcuboot_overlay_dir}/mcuboot_board_defconfig)
  list(APPEND mcuboot_EXTRA_DTC_OVERLAY_FILE
       ${mcuboot_overlay_dir}/mcuboot.overlay
       ${mcuboot_overlay_dir}/mcuboot_boot_mode.overlay)
  list(APPEND ${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE
       ${mcuboot_overlay_dir}/mcuboot_boot_mode.overlay)
  list(APPEND ${DEFAULT_IMAGE}_EXTRA_CONF_FILE
       ${CMAKE_CURRENT_LIST_DIR}/boards/mcuboot.conf)
  list(APPEND mcuboot_EXTRA_CONF_FILE
       ${mcuboot_overlay_dir}/mcuboot_usb_legacy.conf)
  list(REMOVE_DUPLICATES mcuboot_EXTRA_CONF_FILE)
  set(mcuboot_EXTRA_CONF_FILE
      ${mcuboot_EXTRA_CONF_FILE}
      CACHE INTERNAL "")
  list(REMOVE_DUPLICATES mcuboot_EXTRA_DTC_OVERLAY_FILE)
  set(mcuboot_EXTRA_DTC_OVERLAY_FILE
      ${mcuboot_EXTRA_DTC_OVERLAY_FILE}
      CACHE INTERNAL "")
endif()

if(partition_overlay)
  list(APPEND ${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE ${partition_overlay})
  set(${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE
      ${${DEFAULT_IMAGE}_EXTRA_DTC_OVERLAY_FILE}
      CACHE INTERNAL "")
endif()

if(DEFINED ${DEFAULT_IMAGE}_EXTRA_CONF_FILE)
  list(REMOVE_DUPLICATES ${DEFAULT_IMAGE}_EXTRA_CONF_FILE)
  set(${DEFAULT_IMAGE}_EXTRA_CONF_FILE
      ${${DEFAULT_IMAGE}_EXTRA_CONF_FILE}
      CACHE INTERNAL "")
endif()
