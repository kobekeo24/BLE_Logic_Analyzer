# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/13372/esp/esp-idf/components/bootloader/subproject"
  "C:/ESP_Projects/BLE_Logic_Analyzer/BLE_Logic_Analyzer/build/bootloader"
  "C:/ESP_Projects/BLE_Logic_Analyzer/BLE_Logic_Analyzer/build/bootloader-prefix"
  "C:/ESP_Projects/BLE_Logic_Analyzer/BLE_Logic_Analyzer/build/bootloader-prefix/tmp"
  "C:/ESP_Projects/BLE_Logic_Analyzer/BLE_Logic_Analyzer/build/bootloader-prefix/src/bootloader-stamp"
  "C:/ESP_Projects/BLE_Logic_Analyzer/BLE_Logic_Analyzer/build/bootloader-prefix/src"
  "C:/ESP_Projects/BLE_Logic_Analyzer/BLE_Logic_Analyzer/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/ESP_Projects/BLE_Logic_Analyzer/BLE_Logic_Analyzer/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/ESP_Projects/BLE_Logic_Analyzer/BLE_Logic_Analyzer/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
