SHELL := /bin/bash

name :=
urdf_path :=
lang :=

.PHONY: create create-bringup create-interfaces create-description

#####################################################
# create a ros2 package for nodes
#####################################################

create: 
# Verify if both 'name' and 'lang' were defined
ifdef name
ifdef lang
	@echo "" 
	@echo "===================================="
	@echo "Package name: $(name)"
	@echo "Language: $(lang)"
	@echo "===================================="
	@echo ""

# Check the value of 'lang'
ifeq ($(lang),python)
	@echo "Creating Python package..."
	@echo ""
	@ros2 pkg create $(name) --build-type ament_python --dependencies rclpy --license Apache-2.0
	@echo ""
else ifeq ($(lang),cpp)
	@echo "Creating C++ package..."
	@echo ""
	@ros2 pkg create $(name) --build-type ament_cmake --dependencies rclcpp --license Apache-2.0
	@echo ""
# if lang is neither 'python' nor 'cpp', show an error message
else
	$(error 'lang' must be 'python' or 'cpp'. Example: lang:=python)
endif

# Validate the changes
	@echo "===================================="
	@echo "Building the package..."
	@echo " "
	@cd .. && colcon build --packages-select $(name)
	@echo "===================================="
	@echo " "
	@echo "Sourcing the environment..."
	@cd .. && source ~/.bashrc
	@echo "Setting permissions..."
	@chmod -R a+rwx $(name)
	@echo " "
	@echo "Package created succesfully!"

# If lang is not defined show an error message
else
	$(error 'lang' was not defined.)
endif

# if neither name nor lang was defined show an error message
else ifndef lang
	$(error Both 'name' and 'lang' were not defined.)

# If name was not defined show an error message
else
	$(error 'name' was not defined.)
endif

#####################################################
# create a ros2 package for bringup launch files
#####################################################

create-bringup:

# verify is the name was defined
ifdef name
# if name is defined, then create the package
	@echo "" 
	@echo "===================================="
	@echo "Package name: $(name)_bringup"
	@echo "===================================="
	
	@echo ""
	@echo "Creating bringup package..."
	@echo ""
	@ros2 pkg create $(name)_bringup --license Apache-2.0
	@echo ""

# remove the unused directories
	@cd $(name)_bringup && rm -rf src/ include/
	@cd $(name)_bringup && mkdir launch config

# copy and edit the CMakeList.txt file
	@cp /root/dependencies/package_creation/bringup/CMakeLists.txt $(name)_bringup/CMakeLists.txt
	@sed -i '2s/.*/project($(name)_bringup)/' $(name)_bringup/CMakeLists.txt

# copy and edit the package.xml file
	@cp /root/dependencies/package_creation/bringup/package.xml $(name)_bringup/package.xml
	@sed -i '4s/.*/  <name>$(name)_bringup<\/name>/' $(name)_bringup/package.xml

# copy launch file to launch folder
	@cp /root/dependencies/package_creation/bringup/launch/my_robot.launch.py $(name)_bringup/launch/$(name).launch.py

# validate the changes
	@echo "===================================="
	@echo "Building the package..."
	@echo " "
	@cd .. && colcon build --packages-select $(name)_bringup
	@echo "===================================="
	@echo ""
	@echo "Sourcing the environment..."
	@cd .. && source ~/.bashrc
	@echo "Setting permissions..."
	@chmod -R a+rwx $(name)_bringup/
	@echo ""
	@echo "Package created succesfully!"

# if name is not defined, show an error message
else
	$(error 'name' was not defined. Example: make create-bringup name=my_pkg)
endif

#####################################################
# create a ros2 package for interfaces
#####################################################

create-interfaces:

# verify is the name was defined
ifdef name
# if name is defined, then create the package
	@echo ""
	@echo "===================================="
	@echo "Package name: $(name)_interfaces"
	@echo "===================================="
	
	@echo ""
	@echo "Creating interface package..."
	@echo ""
	@ros2 pkg create $(name)_interfaces --license Apache-2.0
	@echo ""

# remove the unused directories
	@cd $(name)_interfaces && rm -rf src/ include/
	@cd $(name)_interfaces && mkdir msg srv action

# copy and edit the CMakeList.txt file
	@cp /root/dependencies/package_creation/interfaces/CMakeLists.txt $(name)_interfaces/CMakeLists.txt
	@sed -i '2s/.*/project($(name)_interfaces)/' $(name)_interfaces/CMakeLists.txt

# copy and edit the package.xml file
	@cp /root/dependencies/package_creation/interfaces/package.xml $(name)_interfaces/package.xml
	@sed -i '4s/.*/  <name>$(name)_interfaces<\/name>/' $(name)_interfaces/package.xml

# validate the changes
	@echo "===================================="
	@echo "Building the package..."
	@echo " "
	@cd .. && colcon build --packages-select $(name)_interfaces
	@echo "===================================="
	@echo ""
	@echo "Sourcing the environment..."
	@cd .. && source ~/.bashrc
	@echo "Setting permissions..."
	@chmod -R a+rwx $(name)_interfaces/
	@echo ""
	@echo "Package created succesfully!"

# if name is not defined, show an error message
else
	$(error 'name' was not defined. Example: make create-interfaces name=my_pkg)
endif

#####################################################
# create a ros2 package for description (urdf files)
#####################################################

create-description:
# verify is the name was defined
ifdef name
# if name is defined, then create the package
	@echo "" 
	@echo "===================================="
	@echo "Package name: $(name)_description"
	@echo "===================================="
	
	@echo ""
	@echo "Creating description package..."
	@echo ""
	@ros2 pkg create $(name)_description --license Apache-2.0
	@echo ""

# remove the unused directories
	@cd $(name)_description && rm -rf src/ include/
	@cd $(name)_description && mkdir urdf launch meshes rviz

ifdef urdf_path
# move the existing urdf to inside the directority n
	@mv $(name).urdf $(urdf_path)
endif 

# copy and edit the CMakeList.txt file
	@cp /root/dependencies/package_creation/description/CMakeLists.txt $(name)_description/CMakeLists.txt
	@sed -i '2s/.*/project($(name)_description)/' $(name)_description/CMakeLists.txt

# copy and edit the package.xml file
	@cp /root/dependencies/package_creation/description/package.xml $(name)_description/package.xml
	@sed -i '4s/.*/  <name>$(name)_description<\/name>/' $(name)_description/package.xml

# copy launch file to launch folder
	@cp /root/dependencies/package_creation/description/launch/display.launch.py $(name)_description/launch/display.launch.py

# validate the changes
	@echo "===================================="
	@echo "Building the package..."
	@echo ""
	@cd .. && colcon build --packages-select $(name)_description
	@echo "===================================="
	@echo ""
	@echo "Sourcing the environment..."
	@cd .. && source ~/.bashrc
	@echo "Setting permissions..."
	@chmod -R a+rwx $(name)_description/
	@echo ""
	@echo "Package created succesfully!"

# if name is not defined, show an error message
else
	$(error 'name' was not defined. Example: make create-description name=my_pkg urdf_path="/root/estudos_ws/src/my_robot.urdf" (optional))
endif