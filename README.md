# Trajectory Alignment

This project implements a program for aligning two trajectories (e.g., ground truth and estimated trajectories) using Umeyama alignment and visualizing them. It also computes the Root Mean Square Error (RMSE) between the two trajectories after alignment.

## Features

- **Trajectory Alignment**: Aligns two trajectories using the Umeyama method.
- **RMSE Calculation**: Computes the RMSE between the aligned and ground truth trajectories.
- **Visualization**: Visualizes the original and aligned trajectories using Pangolin.

## Prerequisites

Before building the project, ensure the following dependencies are installed:

1. **C++ Compiler**: A compiler with C++11 or later support (e.g., GCC, Clang).
2. **CMake**: Version 2.8 or higher.
3. **Pangolin**: For visualization.
4. **Sophus**: For SE(3) transformations.

The program is only tested under Ubuntu 20.04.

### Installing Dependencies

#### Ubuntu
```bash
sudo apt update
sudo apt install build-essential cmake libpangolin-dev
```

For Sophus, you may need to build it from source:
```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## Building the Project

1. **Clone the Repository**:
   ```bash
   git clone <repository-url>
   cd <repository-directory>
   ```

2. **Create a Build Directory**:
   ```bash
   mkdir build
   cd build
   ```

3. **Run CMake**:
   ```bash
   cmake ..
   ```

4. **Compile the Program**:
   ```bash
   make -j
   ```

## Running the Program

The program expects two trajectory files as input: a ground truth trajectory and an estimated trajectory. Each file should contain poses in the following format, consistent with TUM format:

```
timestamp tx ty tz qx qy qz qw
```

Where:
- `tx, ty, tz` are the translation components.
- `qx, qy, qz, qw` are the quaternion components of the rotation.

Note:
Current implementation does not include the time alignment, so you have to input two timely sychronized trajectries.

### Example Usage

```bash
./main <groundtruth_file> <estimated_file>
```

### Output

1. **RMSE**: The program outputs the RMSE between the aligned and ground truth trajectories.
2. **Visualization**: A Pangolin window opens to display the original and aligned trajectories.

### Example Input Files

GT (`./example/trajectory_gt.txt`), estimated (`./example/trajectory_vo.txt`).

```bash
./main ./example/trajectory_gt.txt ./example/trajectory_vo.txt
```

## Troubleshooting

1. **Missing Pangolin or Sophus**:
   Ensure both libraries are installed and their include paths are correctly set in the `CMakeLists.txt` file.

2. **C++11 Not Supported**:
   Use a modern compiler like GCC 5.0+ or Clang 3.3+.

3. **Segmentation Fault**:
   Verify the input files are formatted correctly and contain valid data.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contact

If you have any questions or issues, feel free to open an issue in the repository or contact the [maintainer](jyubt@connect.ust.hk).

### Reference

The trajectory alignment is based on the Umeyama algorithm:

[1] B. Umeyama, "Least-squares estimation of transformation parameters between two point patterns," in *IEEE Transactions on Pattern Analysis and Machine Intelligence*, vol. 13, no. 4, pp. 376–380, April 1991.  



<!-- # TrajectoryErrorCalculationAndDrawing
This is a small demo to compare and plot SLAM estimated trajectories and ground-truth trajectories.

The whole demo is tested in **Ubuntu** Platorforms

## Mathematical theory
<div align=center>  
  
![](https://github.com/TianQi-777/TrajectoryErrorCalculationAndDrawing/blob/master/images/formula1.png)
</div>

## Data description
**ground-truth.txt**:ground-truth trajectories data  
**estimate.txt**:estimated trajectories data  

**Data storage form**  
Time  Translation-x  Translation-y  Translation-z  Quaternion-x  Quaternion-y  Quaternion-z  Quaternion-w  

## Additional Prerequisites for this demo
**Pangolin**  
Use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and interface. 
Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

**Sophus**  
Use [Sophus](https://github.com/strasdat/Sophus) for Lie groups commonly used for 2d and 3d geometric problems. 
Dowload and install instructions can be found at: https://github.com/strasdat/Sophus.

## Build and Run
```
cd XX/XX(include estimated.cpp ,estimated.txt ,groundtruth.txt and CMakeLists.txt)  
mkdir build  
cd build  
cmake ..  
make -j2  
./estimated
```

## Result
**Pangolin GUI:** .  
<div align=center>  
  
![](https://github.com/TianQi-777/TrajectoryErrorCalculationAndDrawing/blob/master/images/drawing.png)
</div>




 -->
