
macro(generate_ros_parameter_files)
    set(CFG_FILES "${ARGN}")
    set(ROSPARAM_HANDLER_ROOT_DIR "${ROSPARAM_HANDLER_CMAKE_DIR}/..")
    if (${PROJECT_NAME}_CATKIN_PACKAGE)
        message(FATAL_ERROR "generate_parameter_files() must be called before catkin_package() in project '${PROJECT_NAME}'")
    endif ()

    # ensure that package destination variables are defined
    catkin_destinations()
    if(dynamic_reconfigure_FOUND_CATKIN_PROJECT)
        add_definitions(-DDYNAMIC_RECONFIGURE_FOUND)
    endif()

    set(_autogen "")
    foreach (_cfg ${CFG_FILES})
        # Construct the path to the .cfg file
        set(_input ${_cfg})
        if (NOT IS_ABSOLUTE ${_input})
            set(_input ${PROJECT_SOURCE_DIR}/${_input})
        endif ()

        get_filename_component(_cfgext ${_cfg} EXT)
        if( _cfgext STREQUAL ".params" OR _cfgext STREQUAL ".mrtcfg")
            # Define required input files
            set(genparam_build_files
                    ${ROSPARAM_HANDLER_ROOT_DIR}/templates/ConfigType.h.template
                    ${ROSPARAM_HANDLER_ROOT_DIR}/templates/Parameters.h.template
                    )

            # Define output files
            get_filename_component(_cfgonly ${_cfg} NAME_WE)
            set(_output_cfg ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg/${_cfgonly}.cfg)
            set(_output_cpp ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${_cfgonly}Parameters.h)
            set(_output_py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/param/${_cfgonly}Parameters.py)

            # We need to explicitly add the devel space to the PYTHONPATH
            # since it might contain dynamic_reconfigure or Python code of the current package
            set("_CUSTOM_PYTHONPATH_ENV")
            if(EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}")
                configure_file(
                    "${ROSPARAM_HANDLER_CMAKE_DIR}/setup_custom_pythonpath_rosif.sh.in"
                    "setup_custom_pythonpath_rosif.sh"
                    @ONLY
                )
                set("_CUSTOM_PYTHONPATH_ENV" "${CMAKE_CURRENT_BINARY_DIR}/setup_custom_pythonpath_rosif.sh")
            endif()

            # Create command
            assert(CATKIN_ENV)
            set(_cmd
                    ${CATKIN_ENV}
                    ${_CUSTOM_PYTHONPATH_ENV}
                    ${_input}
                    ${ROSPARAM_HANDLER_ROOT_DIR}
                    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
                    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
                    )

            add_custom_command(OUTPUT
                    ${_output_cpp} ${_output_cfg} ${_output_py}
                    COMMAND ${_cmd}
                    DEPENDS ${_input} ${genparam_build_files}
                    COMMENT "Generating parameter files from ${_cfgonly}"
                    )

            list(APPEND ${PROJECT_NAME}_LOCAL_CFG_FILES "${_output_cfg}")
            list(APPEND ${PROJECT_NAME}_params_generated ${_output_cpp} ${_output_cfg} ${_output_py})

            # make file show up in ides
            STRING(REGEX REPLACE "/" "-" IDE_TARGET_NAME ${PROJECT_NAME}-show-${_cfgonly}-rosparam)
            add_custom_target(${IDE_TARGET_NAME} SOURCES ${_input})

            install(
                    FILES ${_output_cpp}
                    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
            )
        elseif( _cfgext STREQUAL ".cfg" )
            list(APPEND ${PROJECT_NAME}_LOCAL_CFG_FILES "${_cfg}")
        else()
            message(WARNING "Unknown file ending : ${_cfgext}. Skipping")
        endif()

    endforeach (_cfg)

    # genparam target for hard dependency on generate_parameter generation
    add_custom_target(${PROJECT_NAME}_genparam ALL DEPENDS ${${PROJECT_NAME}_params_generated})

    # register target for catkin_package(EXPORTED_TARGETS)
    list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_genparam)

    # make sure we can find generated headers and that they overlay all other includes
    include_directories(BEFORE ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    # pass the include directory to catkin_package()
    list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    # ensure that the folder exists
    file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})

    # install python files
    install_ros_python_parameter_files()

    # generate dynamic reconfigure files
    if(dynamic_reconfigure_FOUND_CATKIN_PROJECT)
        if(${PROJECT_NAME}_LOCAL_CFG_FILES)
            generate_dynamic_reconfigure_options(${${PROJECT_NAME}_LOCAL_CFG_FILES})
        endif()
    else()
        message(WARNING "Dependency to dynamic_reconfigure is missing, or find_package(dynamic_reconfigure) was not called yet. Not building dynamic config files")
    endif()
endmacro()

macro(install_ros_python_parameter_files)
    if(NOT install_ros_python_parameter_files_CALLED)
        set(install_ros_python_parameter_files_CALLED TRUE)

        # mark that generate_dynamic_reconfigure_options() was called in order to detect wrong order of calling with catkin_python_setup()
        set(${PROJECT_NAME}_GENERATE_DYNAMIC_RECONFIGURE TRUE)

        # check if catkin_python_setup() installs an __init__.py file for a package with the current project name
        # in order to skip the installation of a generated __init__.py file
        set(package_has_static_sources ${${PROJECT_NAME}_CATKIN_PYTHON_SETUP_HAS_PACKAGE_INIT})
        if(NOT EXISTS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py)
          file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py "")
        endif()
        if(NOT package_has_static_sources)
            # install package __init__.py
            install(
               FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py
               DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
               )
        endif()
            
        # generate param module __init__.py
        if(NOT EXISTS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/param/__init__.py)
            file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/param/__init__.py "")
        endif()
        
        # compile python code before installing
        find_package(PythonInterp REQUIRED)
        install(CODE "execute_process(COMMAND \"${PYTHON_EXECUTABLE}\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/param\")")
        install(
            DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/param
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
            )
    endif()
endmacro()
