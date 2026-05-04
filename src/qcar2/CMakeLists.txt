cmake_minimum_required(VERSION 3.8)
project(qcar2)

find_package(ament_cmake REQUIRED)

# Installation des répertoires pour qu'ils soient trouvables après le build
install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()