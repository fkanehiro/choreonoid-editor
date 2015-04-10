option(BUILD_MODELEDIT_PLUGIN "Building Model Edit Plugin" ON)

if(NOT BUILD_MODELEDIT_PLUGIN)
  return()
endif()

set(sources
    ModelEditPlugin.cpp
    EditableModelItem.cpp
    EditableModelBase.cpp
    LinkItem.cpp
    PrimitiveShapeItem.cpp
    JointItem.cpp
    SensorItem.cpp
  )

set(headers
  EditableModelItem.h
  EditableModelBase.h
  LinkItem.h
  PrimitiveShapeItem.h
  JointItem.h
  SensorItem.h
)

set(target CnoidModelEditPlugin)

make_gettext_mofiles(${target} mofiles)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles} )
target_link_libraries(${target} CnoidUtil CnoidBase )
apply_common_setting_for_plugin(${target} "${headers}")
