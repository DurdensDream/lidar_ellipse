# Lidar Data Clustering with Elliptical Kernels

## Overview
This project implements a DBSCAN clustering algorithm using elliptical kernels for Lidar data. It provides 3D visualizations and allows flexible tuning of clustering parameters.

## Requirements
This project was developed and tested on **Windows** using the following tools:
- **Point Cloud Library (PCL)**: Includes Boost, Eigen3, and VTK.
- **CMake**
- **Visual Studio**: Community Edition 2022 (or later).

---

## Setup Instructions

### 1. Prerequisites
1. Install **PCL**: Download and install from [PCL's website](https://pointclouds.org/).
2. Install **CMake**: [Download from CMake](https://cmake.org/).
3. Install **Visual Studio**: Ensure the **Desktop Development with C++** workload is installed.

---

### 2. Build Instructions (via PowerShell)
1. Open **PowerShell** and navigate to the project directory:
   
  ```  cd path\to\lidar_clustering  ```

2. Create a build directory and navigate into it:

  ``` mkdir build ```
  ``` cd build ```

3. Run CMake to generate the Visual Studio project files:

   ``` cmake .. -G "Visual Studio 17 2022" -A x64 ```

4. Build the project in Release mode:

   ``` cmake --build . --config Release ```

5. Verify the build succeeded by checking for the lidar_clustering.exe file in the Release folder.

6. Run the executable with a sample .pcd file:

   ``` .\Release\lidar_clustering.exe ..\data\sample.pcd  ```


### 3. Clustering Parameters
You can adjust the following parameters in main.cpp to optimize clustering for your dataset:

```// Near the bottom of main() ```
```double eps = 0.5;          // Neighborhood size```
```int min_points = 10;       // Minimum points for core point```
```double major_axis = 0.8;   // Ellipse major axis```
```double minor_axis = 0.4;   // Ellipse minor axis```
```double orientation = M_PI / 4; // Ellipse orientation (45 degrees)```
```Adjust these values based on your Lidar data's density and characteristics to achieve better clustering results.```

### 4. Testing
1. The .pcd files are included in the data folder. Replace this with any .pcd files for testing.
2. The output clusters are displayed visually with bounding boxes in a 3D viewer. 
3. Notes:
    a. If you encounter issues, ensure that PCL, CMake, and Visual Studio are properly installed and configured.
    b. For non-Windows platforms, ensure equivalent versions of the dependencies are installed and adjust the CMake generator as needed.
    c. The clustering parameters may need to be adjusted based on the characteristics of your Lidar data

### 5. Example Output
After successful execution, the clusters will be displayed visually in the 3D viewer, with bounding boxes highlighting identified clusters.

---

This README is now tailored for users to build and run the project entirely via PowerShell, with detailed instructions and parameter adjustment options. Let me know if you'd like additional refinements or need help bundling the files!


