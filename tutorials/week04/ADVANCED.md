ADVANCED MATERIAL 
-----------------

The below material is classified advanced is not examinable, and beyond the scope of the subject. Yet, it is here for completeness should you desire to create and distribute your own libraries on a linux system and subsequently use them. All material is in `advanced` folder. Creating debian packages can be done via `CPack` in `CmakeLists.txt` and this is `expert` level material.

### Library Installation

We can now install the library to be part of our system. 

If you have `sudo` access on your computer, the install process will install your library to default install locations which on a Linux system is `/usr/local`

Alternative, you can specify to be any directory of choice and we have two examples below that can be uncommented in your `CMakeLists.txt`. It is common to install under `opt` as ROS does.

* 1. Installs to your build directory `set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR})`
* 2. This would install to your home folder and within subfolder of project_name (ie /home/user/shape_library) `set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/${PROJECT_NAME}")`


To compile the executable

```bash
mkdir build
cd build
cmake ..
make
make install
```

Keep note where the install directory is (if it is not in the system)

### Library Linking

Create a project using the `shape_processing` folder and  [CMakeLists.txt](./advanced/shape_processing/CMakeLists.txt) that links to your library static.

Modify the `CMakeLists.txt` and add the `include_directories` and `link_directories` if needed (if your library is not installed to be part of system).

You can use files created as part of the start/examples code.