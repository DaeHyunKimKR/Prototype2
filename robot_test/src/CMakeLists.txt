

        add_executable(baesong_run
        test_thread.cpp
        subsrc/wheel.cpp 
        subsrc/tray.cpp    
		subsrc/unified_controller.cpp
        )
        
       
        target_link_libraries(baesong_run pthread ${catkin_LIBRARIES})
        add_dependencies(baesong_run ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
