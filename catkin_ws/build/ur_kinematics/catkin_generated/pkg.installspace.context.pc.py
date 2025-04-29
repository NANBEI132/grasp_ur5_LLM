# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include".split(';') if "${prefix}/include;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;geometry_msgs;pluginlib;tf_conversions".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lur3_kin;-lur5_kin;-lur10_kin;/usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0".split(';') if "-lur3_kin;-lur5_kin;-lur10_kin;/usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0" != "" else []
PROJECT_NAME = "ur_kinematics"
PROJECT_SPACE_DIR = "/home/zzq/UR5-Pick-and-Place-Simulation/catkin_ws/install"
PROJECT_VERSION = "1.2.5"
