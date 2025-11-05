# Doppler ICP Stitcher (V0.0)

A ROS2/C++ node that performs point cloud stitching using Doppler information and ICP optimizations with Open3D and CUDA acceleration.

Table of contents
- Project overview
- Key features
- Repository layout
- Requirements & dependencies
- Installation (system & project)
- Building the package
- Running the node
- Visualization (Foxglove Studio)
- Configuration / Parameters
- Topics, services and interfaces
- Testing & debugging
- Troubleshooting & common errors
- Development & contribution guidelines
- License & authors

---

Project overview
----------------
Doppler ICP Stitcher is a ROS 2 ament_cmake package written in C++ (C++17) that stitches point cloud data using DICP (Doppler Iterative Closest Point) techniques, leveraging Open3D for point cloud processing and CUDA for acceleration. It includes integration with ROS 2 messaging, tf2, and rosbag2 for playback/ingest of recorded data.

This repository contains a node executable (`stitch_node`) which is designed to:
- Consume sensor point cloud messages and Doppler-related data,
- Preprocess and align sequential clouds,
- Run ICP registration (using Open3D APIs),
- Publish stitched/merged clouds and relevant transforms,
- Optionally read from rosbag2 to reconstruct environments offline.

Key features
------------
- Native C++17 implementation using ROS 2 ament_cmake.
- Open3D integration for point cloud processing and ICP algorithms.
- CUDA support for performance-sensitive operations (CUDA Toolkit required).
- TF2-aware to keep the stitched output aligned in a common coordinate frame.
- rosbag2 integration for playback and offline processing.
- Designed for extensibility and pairing with additional sensor modalities.

Repository layout
-----------------
- CMakeLists.txt : build definitions (ament_cmake, CUDA, Open3D, Eigen3).
- package.xml : ROS 2 package metadata and dependencies.
- include/ : public headers (intended location; may be empty initially).
- src/ : source files. Main executable: `src/stitch_node_Version.cpp`.
- requirements.txt : 

Requirements & dependencies
---------------------------
System requirements (recommended)
- Linux (Ubuntu LTS generally preferred for ROS 2 compatibility)
- ROS 2 (Distro of your choice : ensure package dependencies match your distro)
- CMake >= 3.8
- A C++ compiler with C++17 support (GCC/Clang)
- CUDA Toolkit (for GPU support) : appropriate version for your GPU and driver
- Open3D (C++ library) with headers and libraries available to CMake
- Eigen3 (Eigen3::Eigen target used)
- pthread (standard on Linux)

ROS 2 package dependencies (declared in package.xml and used in CMakeLists.txt)
- ament_cmake
- rclcpp
- sensor_msgs
- geometry_msgs
- std_msgs
- tf2_ros
- rosbag2_cpp

Notes on Open3D
- The CMakeLists will look for Open3D via find_package(Open3D) and then fallback to manual include/library paths. If Open3D is not found, the build will fail with a clear message.
- You may install Open3D using system packages or build it from source. Ensure Open3D's include path (containing open3d/Open3D.h) and its libraries are discoverable, or set Open3D_DIR.

Installation (system & dependencies)
------------------------------------
1. ROS 2
   - Install a supported ROS 2 distribution (e.g., Humble, Rolling, Galactic) following the official instructions for your OS.
   - Source the ROS 2 environment:
     ```bash
     source /opt/ros/<ros2-distro>/setup.bash
     ```

2. CUDA (optional but recommended for performance)
   - Install CUDA Toolkit matching your GPU and driver. Verify CUDA is available:
     ```bash
     nvcc --version
     ```
   - If CUDA is not required (CPU-only), you may still be able to build , but performance and some code paths may be affected.

3. Eigen3
   - Install via system package manager:
     ```bash
     sudo apt-get install libeigen3-dev
     ```

4. Open3D (C++ library)
   - Option A : system package (if available for your OS/distro)
   - Option B : build and install Open3D from source and install it to a known prefix (e.g., /usr/local)
   - After installation, ensure CMake can find:
     - open3d/Open3D.h (include path)
     - Open3D library files
   - If Open3D is installed to a custom path, set CMake variables or environment:
     ```bash
     export Open3D_DIR=/path/to/open3d/cmake
     ```

5. Additional ROS 2 packages
   - Install any missing ROS 2 dependencies:
     ```bash
     sudo apt-get install ros-<ros2-distro>-rclcpp ros-<ros2-distro>-sensor-msgs ros-<ros2-distro>-geometry-msgs ros-<ros2-distro>-tf2-ros ros-<ros2-distro>-rosbag2-*
     ```

Building the package
--------------------
This project uses ament_cmake. 
Recommended approach: create or use an existing ROS 2 workspace.

1. Create a workspace (if you don't have one):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. Clone this repository into the workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/LinaSaii/Doppler_Icp_Stitcher_V0.0.git
   ```

3. Install any OS-level dependencies and ensure Open3D is discoverable.

4. Build the workspace:
   ```bash
   cd ~/ros2_ws
   source /opt/ros/<ros2-distro>/setup.bash
   colcon build --symlink-install
   ```

   - If CMake fails complaining about Open3D not found, either install Open3D via package manager or set Open3D include/lib paths / Open3D_DIR appropriately.

5. Source the overlay:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

Running the node
----------------
After sourcing the workspace (see above), run the node with the package name and executable defined in CMake:

```bash
ros2 run doppler_icp_stitcher_open3d stitch_node
```

Notes:
- If you have launch files in the repository's launch/ directory, prefer running them:
  ```bash
  ros2 launch doppler_icp_stitcher_open3d <launch-file>.py
  ```
- The node expects point cloud inputs and TF frames. Use bag playback to feed recorded topics:
  ```bash
  ros2 bag play <bagfile>
  ```
Visualization (Foxglove Studio)
-------------------------------
This project supports visualization with Foxglove Studio. Below are recommended ways to connect and example panel settings to visualize the stitched point cloud, Doppler-derived values, and TF frames.

1) Install Foxglove Studio
- Download the desktop app or use the web app at https://foxglove.dev/studio

2) Connect Foxglove to your live ROS 2 system
- Start ROS 2 and the stitch node (see "Running the node")
- Start a WebSocket bridge between ROS 2 and Foxglove. Common options:
  - rosbridge_server (if installed for your ROS 2 distro):
    ```
    ros2 run rosbridge_server rosbridge_websocket
    ```
    This exposes a websocket at ws://localhost:9090 by default; point Foxglove to that URL.
  - If you use another ROS<->WebSocket bridge (or Foxglove's connector tools), run that bridge and point Foxglove Studio to the provided websocket URL.

3) Alternative: open recorded data in Foxglove
- Foxglove Studio can open rosbag2 recordings directly. Use "Open" → select your Rosbag2 session/folder and Foxglove will list topics for playback and inspection.

4) Recommended Foxglove panels and settings
- 3D Panel (Point Cloud)
  - Topic: `/stitch/merged_cloud` (sensor_msgs/PointCloud2)
  - Message type: sensor_msgs/PointCloud2
  - Color mode: choose `Intensity` or `RGB` depending on message fields (if Doppler intensity is encoded, choose Intensity)
  - Point size: increase for visibility (e.g., 2–5 px)
  - Decimation / Voxel: enable if very dense to improve rendering speed
  - Frame: set Fixed Frame to your global frame (e.g., `map`) and ensure TF is published

- TF Panel
  - Add a TF panel to visualize the coordinate frames being published (topics `/tf` and `/tf_static`)

- Numeric / Scalar panels (Doppler values)
  - If you publish Doppler-derived scalars or diagnostics (std_msgs/Float32 or custom), add a Time Series or Histogram panel to monitor those values over time.

- Image / Camera panels
  - If you publish camera images, add Image panels and synchronize with the 3D view.

5) Tips for smooth visualization
- Ensure TF frames used by the point cloud are present in the TF tree (use TF panel to verify).
- If point cloud appears in the wrong pose, check the message header.frame_id and TF between that frame and the Fixed Frame.
- Use Foxglove's playback controls when using rosbag2 to pause/seek and inspect specific timestamps.
  
Configuration / Parameters
--------------------------
This repository currently builds a single executable target called `stitch_node` (defined in CMakeLists.txt via `src/stitch_node_Version.cpp`). The node's runtime parameters and configurable options should be defined in the node source or launch files. Typical parameter examples you may add or expect:

- input_topic (string): topic name for incoming point clouds (e.g., `/camera/points`)
- frame_id (string): target/world frame to stitch into (e.g., `map`)
- use_cuda (bool): enable/disable CUDA-accelerated code paths
- voxel_size (float): voxel size for downsampling
- max_icp_iterations (int): maximum ICP iterations
- tolerance (float): convergence tolerance for ICP
- publish_merged_cloud (bool): whether merged results are published
- output_topic (string): topic for published stitched cloud

Add or document exact parameters in the node code or in launch files for clarity.

Topics, services and interfaces
------------------------------
The package references and links to the following ROS message packages:
- sensor_msgs (likely sensor_msgs/PointCloud2)
- geometry_msgs
- std_msgs
- tf2_ros (for transforms)
- rosbag2_cpp (for bag playback/processing within the node)

Suggested topic conventions:
- Subscribed:
  - /input/pointcloud (sensor_msgs/PointCloud2) : incoming point clouds
  - /tf (tf2) : transform frames
- Published:
  - /stitch/merged_cloud (sensor_msgs/PointCloud2) : merged point cloud
  - /stitch/info (std_msgs/String or custom msg) : status/diagnostics

Testing & debugging
-------------------
- Unit tests: Add gtest or similar unit tests inside a test/ directory and hook them up in CMakeLists.
- Runtime testing:
  - Use ros2 topic echo to inspect published topics.
  - Use Foxglove for visualizing incoming and merged point clouds .
  - Run ros2 bag play to provide input data and monitor outputs.

Debugging tips:
- If build fails with "Open3D not found", verify Open3D installation and set Open3D_DIR if needed.
- If CUDA symbols fail linking, ensure CUDA Toolkit and drivers match and CMake finds CUDAToolkit.
- For runtime TF issues, ensure the frames referenced in your point clouds have corresponding TF broadcasters.

Troubleshooting & common errors
-------------------------------
- Open3D not found
  - Cause: Open3D headers/libs not installed or installed to non-standard prefix.
  - Fix: Install Open3D, or set Open3D_DIR to your Open3D CMake package directory.
    Example:
    
    ```bash
    export Open3D_DIR=/usr/local/lib/cmake/Open3D
    ```

- CUDA not found / NVCC errors
  - Cause: CUDA Toolkit not installed or PATH/LD_LIBRARY_PATH not set.
  - Fix: Install CUDA Toolkit appropriate for your GPU. Ensure `nvcc` is in PATH and libraries are discoverable.

- ROS 2 package dependency missing
  - Cause: required ROS 2 package not installed for your distro.
  - Fix: Install required ROS 2 packages using apt or rosdep.

- Build fails due to missing Eigen3
  - Fix: Install libeigen3-dev via your package manager.

Development & contribution guidelines
------------------------------------
- Coding style: Use C++17 and follow ROS 2 best practices for node lifecycle and parameter handling.
- Branching: Use feature branches and open pull requests for review.
- Tests: Include unit tests where possible and CI checks for both linting and building.
- Documentation: Keep README and inline code documentation up to date. Add API docs for public headers in include/.

License & authors
-----------------
- License:
- Authors / Maintainers:
- Repository owner:

Acknowledgements & references
-----------------------------
- Open3D : powerful library for 3D data processing: https://www.open3d.org
- ROS 2 documentation : for ament_cmake, rclcpp, tf2, and rosbag2
- CUDA Toolkit : for GPU acceleration and cuBLAS

Contact & support
-----------------
For issues, feature requests or contributions, please open an issue or a pull request on the repository:
https://github.com/FarnessApp/Doppler_Icp_Stitcher_V0.0

---
