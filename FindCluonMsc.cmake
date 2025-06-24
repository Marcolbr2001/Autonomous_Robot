# Copyright (C) 2024 OpenDLV

if (NOT CLUONMSC_FOUND)
  find_file(
    CLUONMSC_PATH cluon-msc
    PATHS
    /usr/bin
    /usr/local/bin)

  if (CLUONMSC_PATH)
    set (CLUONMSC_FOUND TRUE)
  endif (CLUONMSC_PATH)

  if (CLUONMSC_FOUND)
    message(STATUS "Found cluon-msc: ${CLUONMSC_PATH}")
  else (CLUONMSC_FOUND)
    if (CluonMsc_FIND_REQUIRED)
      message (FATAL_ERROR "Could not find cluon-msc")
    else (CluonMsc_FIND_REQUIRED)
      message(STATUS "Proceeding without cluon-msc")
    endif (CluonMsc_FIND_REQUIRED)
  endif (CLUONMSC_FOUND)

endif (NOT CLUONMSC_FOUND)
