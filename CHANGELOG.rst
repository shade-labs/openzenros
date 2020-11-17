^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openzen_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2020-11-17)
------------------
* fixed conditions check for build or download OpenZen options
* fixed name of primary maintainer
* added install target for openzen_node
* manually setting component ID in case a TestSensor is used
* running rostest as part of the catkin CMake file
* switched binary downloads to OpenZen 1.2.0
* updated OpenZen to version 1.2.0
* added output of GNSS measurement as ROS NavSatFix message
* Contributors: LP-Research Inc. Team

1.0.1 (2020-09-03)
------------------
* changed OpenZen version to release 1.1.3
* added option to select binary download and disabled binary downloads by default
* updated to OpenZen 1.1.2 for ARM64
* updated OpenZen version to support LPMS-BE1
* added support for ARM64 binary
* checking if environment is modern enough to build OpenZen, otherwise download binary release
* added licenses of OpenZen and its dependencies
* added License header to source files
* added License file
* Contributors: LP-Research Inc. Team

1.0.0 (2020-04-13)
------------------
* Initial Release of OpenZen ROS driver
* Contributors: LP-Research Inc. Team
