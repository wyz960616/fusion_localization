find_package(PCL  REQUIRED)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS})