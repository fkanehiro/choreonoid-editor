cmake_minimum_required(VERSION 2.6)
cmake_policy(SET CMP0003 NEW)
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

include(FindPkgConfig)
include(CheckIncludeFiles)

project(ChoreonoidEditor)

pkg_check_modules(CHOREONOID choreonoid)
include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=plugindir choreonoid
                    OUTPUT_VARIABLE CNOID_PLUGIN_SUBDIR
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

STRING(REGEX REPLACE "([0-9]+).[0-9]+.[0-9]+" "\\1" CNOID_MAJOR_VERSION ${CHOREONOID_VERSION})
STRING(REGEX REPLACE "[0-9]+.([0-9]+).[0-9]+" "\\1" CNOID_MINOR_VERSION ${CHOREONOID_VERSION})
STRING(REGEX REPLACE "[0-9]+.[0-9]+.([0-9]+)" "\\1" CNOID_PATCH_VERSION ${CHOREONOID_VERSION})
set(CNOID_VERSION ${CNOID_MAJOR_VERSION}.${CNOID_MINOR_VERSION})
set(CNOID_FULL_VERSION ${CNOID_MAJOR_VERSION}.${CNOID_MINOR_VERSION}.${CNOID_PATCH_VERSION})

if(MSVC AND CMAKE_CL_64)
   add_definitions(-D_WIN64)
endif()

if(MSVC)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG _ITERATOR_DEBUG_LEVEL=1)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

if(UNIX)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -finline-functions")
  option(ENABLE_GCC_FVISIBILITY_HIDDEN "Use the -fvisibility=hidden option when the sources are compiled" ON)
endif()

set(ADDITIONAL_CXX_FLAGS_RELEASE ${ADDITIONAL_CXX_FLAGS_RELEASE} CACHE STRING "Additional c++ compiler optimization flags")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${EXTRA_CXX_FLAGS_RELEASE} ${ADDITIONAL_CXX_FLAGS_RELEASE}")


if(MSVC)

  set(ext_compiler_options "/Ob2 /Ox /Oi /Ot /Oy /GT /GS- /fp:fast")
  if(CMAKE_CL_64)
    set(MSVC_ENABLE_SSE OFF CACHE BOOL "Enable SSE instructions on VC++." FORCE)
  else()
    option(MSVC_ENABLE_SSE "Enable SSE instructions on VC++." ON)
  endif()
  if(MSVC_ENABLE_SSE)
    set(ext_compiler_options "${ext_compiler_options} /arch:SSE /arch:SSE2 /fp:fast")
  endif()
  option(MSVC_ENABLE_AVX "Enable AVX instructions on VC++." OFF)
  if(MSVC_ENABLE_AVX)
    set(ext_compiler_options "${ext_compiler_options} /arch:AVX")
  endif()
  set(ext_linker_options "")

  option(MSVC_ENABLE_GLOBAL_OPTIMIZATION "Global optimization with compiler option /GL and linker option /LTCG" ON)
  if(MSVC_ENABLE_GLOBAL_OPTIMIZATION)
    set(ext_compiler_options "${ext_compiler_options} /GL")
    set(ext_linker_options "${ext_linker_options} /LTCG")
  endif()

  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${ext_compiler_options}")
  set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} ${ext_linker_options}")
  set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} ${ext_linker_options}")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${ext_linker_options}")

  if(MSVC_ENABLE_SSE)
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /GS- /fp:fast /arch:SSE2 /arch:SSE2")

    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /fp:fast /arch:SSE2 /arch:SSE2")
  endif()
endif()

# commands
if(UNIX)
  set(RMDIR rm -fr)
elseif(WIN32)
  set(RMDIR rmdir /S/Q)
endif()

# check "dlfcn.h" for using dlopen() and dlclose()
if(UNIX)
  check_include_files(dlfcn.h HAVE_DLFCN_H)
  if(NOT HAVE_DLFCN_H)
    message(FATAL_ERROR "Could not find dlfcn.h")
  endif()
endif()


# gettext
option(CNOID_ENABLE_GETTEXT "Enable the gettext library and translation messages for the internationalization" ON)

if(CNOID_ENABLE_GETTEXT)
  if(WIN32)
    if(CMAKE_CL_64)
      FIND_PROGRAM(GETTEXT_MSGFMT_EXECUTABLE msgfmt ${PROJECT_SOURCE_DIR}/thirdparty/windows64/bin)
    else()
      FIND_PROGRAM(GETTEXT_MSGFMT_EXECUTABLE msgfmt ${PROJECT_SOURCE_DIR}/thirdparty/windows/bin)
    endif()
  else()
    FIND_PROGRAM(GETTEXT_MSGFMT_EXECUTABLE msgfmt)
  endif()
  if(NOT GETTEXT_MSGFMT_EXECUTABLE)
    message(FATAL_ERROR "Could not find the msgfmt command and gettext cannot be enabled.")
  endif()
  get_filename_component(GETTEXT_BINARY_DIR ${GETTEXT_MSGFMT_EXECUTABLE} PATH)
  get_filename_component(GETTEXT_DIR ${GETTEXT_BINARY_DIR} PATH)
  set(GETTEXT_INCLUDE_DIR ${GETTEXT_DIR}/include)
  set(GETTEXT_LIBRARY_DIR ${GETTEXT_DIR}/lib)
  include_directories(${GETTEXT_INCLUDE_DIR})
  link_directories(${GETTEXT_LIBRARY_DIR})
  if(WIN32)
    set(GETTEXT_LIBRARIES intl)
  elseif(APPLE)
    set(GETTEXT_LIBRARIES intl)
    #set(GETTEXT_STATIC_LIBRARIES
    #  ${GETTEXT_LIBRARY_DIR}/libintl.a 
    #  ${GETTEXT_LIBRARY_DIR}/libiconv.a
    #  System.B 
    #  /System/Library/Frameworks/CoreFoundation.framework)
  else()
    set(GETTEXT_LIBRARIES "")
  endif()
  if(MSVC)
    #install_external_libraries(${GETTEXT_BINARY_DIR} ${GETTEXT_LIBRARY_DIR} ${GETTEXT_LIBRARIES})
    if(CMAKE_CL_64)
      install(FILES thirdparty/windows64/bin/libintl-8.dll DESTINATION bin)
    else()
      install(FILES thirdparty/windows/bin/intl.dll DESTINATION bin)
    endif()
    if(INSTALL_SDK)
      if(CMAKE_CL_64)
        install(FILES thirdparty/windows64/lib/intl.lib DESTINATION lib)
      else()
        install(FILES thirdparty/windows/lib/intl.lib DESTINATION lib)
      endif()
      install(FILES "${GETTEXT_INCLUDE_DIR}/libintl.h" DESTINATION ${CNOID_HEADER_SUBDIR})
    endif()
  endif()
else()
  set(GETTEXT_LIBRARIES "")
endif()

# boost

# set(Boost_NO_SYSTEM_PATHS true)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_ADDITIONAL_VERSIONS "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0" "1.46.1" "1.47" "1.47.0" "1.48" "1.48.0" "1.49.0" "1.50.0")


if(MSVC)
  find_package(Boost 1.36.0 QUIET OPTIONAL_COMPONENTS  bzip2 zlib)
endif()

set(boost_packages system filesystem program_options regex thread iostreams date_time)

find_package(Boost 1.36.0 REQUIRED COMPONENTS ${boost_packages})

if(NOT Boost_FOUND)
  set(BOOST_ROOT ${BOOST_ROOT} CACHE PATH "set the directory of the boost library")
  message(FATAL_ERROR "Boost cannot be found. Please specify the boost top directory to BOOST_ROOT.")
endif()

if(Boost_VERSION GREATER "104699")
  find_package(Boost 1.47.0 REQUIRED COMPONENTS chrono)
endif()

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS_RELEASE BOOST_DISABLE_ASSERTS)
# set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS BOOST_FILESYSTEM_VERSION=2)

# eigen
if(NOT EIGEN_DIR)
  if(UNIX)
    pkg_check_modules(EIGEN eigen3)
    if(EIGEN_FOUND)
      set(EIGEN_DIR ${EIGEN_INCLUDE_DIRS})
    endif()
  endif()
else()
  set(EIGEN_INCLUDE_DIRS ${EIGEN_DIR})
endif()

set(EIGEN_DIR ${EIGEN_DIR} CACHE PATH "The directory of the Eigen library")

if(NOT EIGEN_INCLUDE_DIRS)
  message(FATAL_ERROR "Please specify the Eigen directory to EIGEN_DIR.")
else()
  include_directories(${EIGEN_INCLUDE_DIRS})
endif()

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS EIGEN_NO_DEBUG)
#set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS EIGEN_NO_DEBUG EIGEN_DONT_ALIGN)

# sdformat
set(SDFORMAT_DIR ${SDFORMAT_DIR} CACHE PATH "set the top directory of the SDFORMAT ")
if(UNIX)
  if(NOT SDFORMAT_DIR)
    pkg_check_modules(SDFORMAT REQUIRED sdformat)
  endif()
elseif(MSVC)
  if(NOT SDFORMAT_DIR)
    message(FATAL_ERROR "Please specify the directory of the SDFORMAT to SDFORMAT_DIR.")
  endif()
endif()

if(NOT SDFORMAT_FOUND)
  set(SDFORMAT_LIBRARIES sdformat)
  set(SDFORMAT_INCLUDE_DIRS ${SDFORMAT_DIR}/include)
  set(SDFORMAT_LIBRARY_DIRS ${SDFORMAT_DIR}/lib)
endif()

include_directories(${SDFORMAT_INCLUDE_DIRS})
link_directories(${SDFORMAT_LIBRARY_DIRS})

# doxygen
# find_package(Doxygen)

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG CNOID_DEBUG)

if(UNIX)
  
  option(PUT_ALL_WARNINGS "put all warnings in compile" OFF)
  if(PUT_ALL_WARNINGS)
    list(APPEND CMAKE_C_FLAGS "-Wall")
    list(APPEND CMAKE_CXX_FLAGS "-Wall")
  endif()

  option(CHECK_UNRESOLVED_SYMBOLS "check unresolved symbols in the object files when creating shared libraries" OFF)
  #mark_as_advanced(CHECK_UNRESOLVED_SYMBOLS)
  if(CHECK_UNRESOLVED_SYMBOLS)
    if(NOT APPLE)
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--unresolved-symbols=ignore-in-shared-libs -Wl,--warn-unresolved-symbols")
    endif()
  endif()


  # The RTTI such as dynamic_cast cannot work well with this option in the older GCC like that of Ubuntu 10.04.
  # So currently this option should be only enabled for modules which may cause symbol conficts
  #set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-Bsymbolic") 

elseif(MSVC)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS _CRT_SECURE_NO_WARNINGS)
  set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS NOMINMAX _USE_MATH_DEFINES)
  #list(APPEND CMAKE_CXX_FLAGS "/wd4250 /wd4251 /wd4275")
endif()

link_directories(${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})


# Document installaiton
#install(FILES NEWS DESTINATION ${CNOID_DOC_SUBDIR})
#install(FILES LICENSE DESTINATION ${CNOID_DOC_SUBDIR})

function(add_cnoid_library)

  add_library(${ARGV})

  if(ENABLE_GCC_FVISIBILITY_HIDDEN)
    get_target_property(compile_flags ${ARGV0} COMPILE_FLAGS)
    if(NOT compile_flags)
      set(compile_flags "")
    endif()
    if(ARGV1 STREQUAL "STATIC")
      set(compile_flags "${compile_flags} -fPIC")
    endif()
    set_target_properties(${ARGV0} PROPERTIES COMPILE_FLAGS "${compile_flags} -fvisibility=hidden")
  endif()

  set_target_properties(${ARGV0} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

  if(ENABLE_INSTALL_RPATH)
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "@rpath")
      set_target_properties(${ARGV0} PROPERTIES LINK_FLAGS "-Wl,-rpath,@loader_path")
    else()
      set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN")
    endif()
  else()
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "")
    endif()      
  endif()

endfunction()


function(add_cnoid_plugin)

  add_library(${ARGV})

  if(ENABLE_GCC_FVISIBILITY_HIDDEN)
    get_target_property(compile_flags ${ARGV0} COMPILE_FLAGS)
    if(compile_flags)
      set_target_properties(${ARGV0} PROPERTIES COMPILE_FLAGS "${compile_flags} -fvisibility=hidden")
    else()
      set_target_properties(${ARGV0} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
    endif()
  endif()

  #set_target_properties(${ARGV0} PROPERTIES
  #  LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
  #  ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}
  #  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR})

  if(ENABLE_INSTALL_RPATH)
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "@rpath")
      set_target_properties(${ARGV0} PROPERTIES LINK_FLAGS "-Wl,-rpath,@loader_path,-rpath,@loader_path/..")
    else()
      set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN:$ORIGIN/..")
    endif()
  else()
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_NAME_DIR "")
    else()
      set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN")
    endif()
  endif()

endfunction()


function(add_cnoid_executable)

  add_executable(${ARGV})

  set_target_properties(${ARGV0} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

  if(ENABLE_INSTALL_RPATH)
    if(APPLE)
      set_target_properties(${ARGV0} PROPERTIES LINK_FLAGS "-Wl,-rpath,@executable_path/../lib")
    elseif(UNIX)
      set_target_properties(${ARGV0} PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
    endif()
  endif()

  apply_common_setting_for_target(${ARGV0})

  install(TARGETS ${ARGV0} RUNTIME DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)

endfunction()


function(apply_common_setting_for_target target)

  set(headers ${ARGV1})

  if(MSVC)
    if(MSVC_IDE)
      if(headers)
        source_group("Header Files" FILES ${headers})
      endif()
      #set_target_properties(${target} PROPERTIES PREFIX "../" IMPORT_PREFIX "../")
    endif()

    get_target_property(existing_compile_flags ${target} COMPILE_FLAGS)
    if(existing_compile_flags STREQUAL existing_compile_flags-NOTFOUND)
      set(existing_compile_flags "")
    endif()
    set_target_properties(${target} PROPERTIES COMPILE_FLAGS "${existing_compile_flags} /MP /wd4250 /wd4251 /wd4275 /wd4819")

    get_target_property(existing_link_flags ${target} LINK_FLAGS)
    if(existing_link_flags STREQUAL existing_link_flags-NOTFOUND)
      set(existing_link_flags "")
    endif()
    set_target_properties(${target} PROPERTIES LINK_FLAGS "${existing_link_flags} /NODEFAULTLIB:LIBCMT")

    set_target_properties(${target} PROPERTIES DEBUG_POSTFIX d)
  endif()

endfunction()

function(apply_common_setting_for_library_core target)

  set(headers ${ARGV1})
  set_target_properties(${target} PROPERTIES VERSION ${CNOID_VERSION})
  apply_common_setting_for_target(${target} "${headers}")

  if(INSTALL_SDK)
    install(TARGETS ${target}
      RUNTIME DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
  else()
    get_target_property(target_type ${target} TYPE)
    if(target_type STREQUAL STATIC_LIBRARY)
      install(TARGETS ${target}
	LIBRARY DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
	ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
    else()
      install(TARGETS ${target}
	RUNTIME DESTINATION bin CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
	LIBRARY DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
      endif()
  endif()

endfunction()


# (target "header1 header2 header3 ...")
function(apply_common_setting_for_library target)
  apply_common_setting_for_library_core(${ARGV})
  if(INSTALL_SDK)
    set(headers ${ARGV1})
    if(headers)
      file(RELATIVE_PATH rel_src_dir ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
      install(FILES ${headers} DESTINATION ${CNOID_HEADER_SUBDIR}/cnoid/${rel_src_dir})
    endif()
  endif()
endfunction()


function(apply_common_setting_for_ext_library)
  apply_common_setting_for_library_core(${ARGV})
endfunction()


function(apply_common_setting_for_plugin target)

  set(headers ${ARGV1})
  apply_common_setting_for_target(${target} "${headers}")

  if(INSTALL_SDK)
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      ARCHIVE DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
    if(headers)
      file(RELATIVE_PATH header_subdir ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
      install(FILES ${headers} DESTINATION ${CNOID_HEADER_SUBDIR}/cnoid/${header_subdir})
    endif()
  else()
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel
      LIBRARY DESTINATION ${CNOID_PLUGIN_SUBDIR} CONFIGURATIONS Release Debug RelWithDebInfo MinSizeRel)
  endif()

endfunction()


#function(make_gettext_mofiles out_mofiles)
function(make_gettext_mofiles target out_mofiles)

  configure_file(${PROJECT_SOURCE_DIR}/gettext.h.in ${CMAKE_CURRENT_SOURCE_DIR}/gettext.h)
 
  set(${out_mofiles} "")

  if(NOT CNOID_ENABLE_GETTEXT)
    return()
  endif()

  file(GLOB pofiles ${CMAKE_CURRENT_SOURCE_DIR}/po/*.po)

  foreach(pofile ${pofiles})
    get_filename_component(lang ${pofile} NAME_WE)
    set(message_location share/locale/${lang}/LC_MESSAGES)
    #file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${message_location})
    #set(mofile ${PROJECT_BINARY_DIR}/${message_location}/${target}-${CNOID_VERSION}.mo)
    file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/${message_location})
    set(mofile ${PROJECT_SOURCE_DIR}/${message_location}/${target}-${CNOID_VERSION}.mo)
    add_custom_command(
      OUTPUT ${mofile}
      COMMAND ${GETTEXT_MSGFMT_EXECUTABLE} -o ${mofile} ${pofile}
      DEPENDS ${pofile}
      )
    list(APPEND mofiles ${mofile})
    install(FILES ${mofile} DESTINATION "share/locale/${lang}/LC_MESSAGES")
  endforeach()

  set(${out_mofiles} ${mofiles} PARENT_SCOPE)

endfunction()

add_subdirectory(src)

configure_file(Doxyfile.in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile @ONLY)

