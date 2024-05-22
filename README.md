# IR Retro-reflective Marker Tracking with Intel RealSense

[![GitHub release](https://img.shields.io/github/release/stytim/RealSense-ToolTracker.svg?style=flat-square)](https://github.com/stytim/RealSense-ToolTracker/releases)

This project leverages the versatility of Intel RealSense cameras, enabling high-quality tracking of passive IR sphere markers without the need for large and expensive equipment like NDI trackers.

<p align="center">
	<img width="80%" src="image/overview.gif">
</p>


## Project Highlights
* **Compact and Cost-Effective**: Break free from the constraints of traditional tracking systems. Our project utilizes the small, affordable Intel RealSense camera for efficient tracking.
* **Versatile and User-Friendly**: Efficiently handle simultaneous tracking of various markers. Enjoy robust features such as multiple marker tracking, occlusion resistance, and support for various marker types and sizes.
* **Smooth and Stable Tracking**: Integrate Kalman and low-pass filters for stable tracking.
* **Enhanced Communication and Compatibility**: Utilize UDP messaging for transmitting tracking results and support for NDI .rom files
* **Simple Calibration**: A simple marker array calibration is provided if the marker configuration is not known beforehand
* **Cross-Platform Availability**: Works across Linux, Windows, and MacOS environments.

## Installation
**Quick Start with Precompiled Binary (Windows):** Download the latest precompiled binary from our releases section for immediate use.

**Building from Source:** For those interested in customizing or contributing to the project, please see the build instructions below.

## Prerequisites

* Intel RealSense SDK (MacOS users, please refer to the FAQ for installation guidance)
* OpenCV 4
* CMake 

## Building
Ensure all the prerequisites are installed before proceeding with the build process.

### For Linux and MacOS:

```bash
mkdir build && cd build
cmake ..
make
```
### For Windows
On Windows, you'll need to use CMake to generate build files specific to your platform (e.g., Visual Studio). After generating these files, you can build the project using your chosen IDE or build system.
Note: I have only tested this build process with Visual Studio 2022.


## Running

### For Linux and Windows:
```bash
./ir-tracking-app
```
### For MacOS
Due to certain permissions and security features in MacOS, you might need to run the application with elevated privileges.
```bash
sudo ./ir-tracking-app
```
## RealSense Camera Modification: Adding a Light Diffuser

The laser projector of the RealSense camera emits a sharp, focused IR dot pattern. While this is generally beneficial for depth sensing, it is not ideal for doing thresholding on IR stream to find retroreflective surfaces.

Therefore, a crucial modification is required: the addition of a physical light diffuser filter in front of the camera's laser projector. This softens the laser dot pattern in the IR stream. 
    <table>
        <tr>
            <td align="center" width="45%">
                <img src="image/light_diffuser.jpg" alt="RealSense with light diffuser"><br>
                RealSense with Light Diffuser
            </td>
            <td align="center" width="45%">
                <img src="image/no_diffuser.png" alt="Without Light Diffuser"><br>
                Without Light Diffuser, the visible dot pattern can interfere with detecting retroreflective surfaces in the IR stream.
            </td>
        </tr>
    </table>
</p>




## Usage Guide

Upon launching the RealSense Tool Tracker, the application will attempt to load previously defined tool configurations from the 'Tools' directory. Follow the steps below to set up your environment and begin tracking:

### 1. Tool Configuration:

* Number of Tools: Specify how many tools you wish to track.
* Tool Details: For each tool, define the following:
  - Number of Spheres: Indicate how many spheres are attached to each tool.
  - Sphere Radius: Enter the radius for each sphere to ensure precise tracking.
  - Tool Name: Assign a unique name for easy identification.
  - Sphere Positions: Input the positions of each sphere relative to the tool's coordinate system.

* Tool Configuration:
  - Load ROM (Optional): To use pre-defined configurations, click "Load ROM" and choose the relevant NDI ROM file
  - Calibrate Tool (Optional): If the configuration is unknown, ensure no reflective objects are in the camera's view, then calibrate by clicking "Calibrate Tool."
  - Add Tool Definition: Finalize the setup and enable tracking by clicking "Add Tool Definition."
  - Repeat this process for multiple tools as needed.

### 2. Network Configuration

* Input the "IP Address" and "Port" for networked UDP messaging.
* Transmission Rate: Confirm the "Frequency" of tracking updates to suit your needs.
* UDP Enable: Ensure the "UDP" checkbox is selected to activate network transmission.

### 3. Start Tracking
* Initiate Tracking: With all parameters set, initiate tracking by clicking "Start Tracking."
* Real-Time Adjustments: You can tweak the Tracking Parameters in real-time to adapt to varying conditions.
  - Infrared Sensitivity: Adjust the "IR Threshold" to optimize infrared tracking sensitivity.
  - Pixel Range: Set the "Min Px" and "Max Px" to define the acceptable pixel range for detection.
* Results Monitor: The application will display the position and orientation data for each configured tool.

### Additional Resources
A sample Python and a C# Unity scripts is provided for receiving and processing the tracking data via UDP.

## Contributing
Contributions are welcome! When submitting a pull request, please include a description of your improvements and reference any relevant issue numbers.

## Frequently Asked Questions (FAQs)
### Can I use Azure Kinect Cameras for this project?

Currently, Azure Kinect Cameras have an issue with marker overexposure, which affects the retrieval of valid depth values. For more insights, please refer to the discussion on [Azure-Kinect-Sensor-SDK Issue #1349](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1349). 
However, I have a hack regarding this issue. To put it simply, I sampled the pixels around the overexposed areas and used them to estimate the depth of the marker. Please refer to the [AzureKinect Tool Tracker](https://github.com/stytim/AzureKinect-ToolTracker)

### I'm encountering issues while installing the Intel RealSense SDK on MacOS. What should I do?

Installation issues on MacOS can be tricky due to Intel dropping support for newer MacOS and Mac with Apple silicon. I recommend this comprehensive guide on [setting up Intel RealSense on MacOS](https://lightbuzz.com/realsense-macos/). 


## License and Citation

This project is distributed under the MIT License.

The core idea of the tracking algorithm is based on the following publication:

```bibtex
@ARTICLE{10021890,
  author={Martin-Gomez, Alejandro and Li, Haowei and Song, Tianyu and Yang, Sheng and Wang, Guangzhi and Ding, Hui and Navab, Nassir and Zhao, Zhe and Armand, Mehran},
  journal={IEEE Transactions on Visualization and Computer Graphics}, 
  title={STTAR: Surgical Tool Tracking using Off-the-Shelf Augmented Reality Head-Mounted Displays}, 
  year={2023},
  volume={},
  number={},
  pages={1-16},
  doi={10.1109/TVCG.2023.3238309}}

```
A significant portion of the tracking code is adapted from Andreas Keller's work:
```BibTeX
@misc{keller2023hl2irtracking,
  author =       {Andreas Keller},
  title =        {HoloLens 2 Infrared Retro-Reflector Tracking},
  howpublished = {\url{https://github.com/andreaskeller96/HoloLens2-IRTracking}},
  year =         {2023}
}
```



<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=stytim/RealSense-ToolTracker&type=Date&theme=dark" />
  <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=stytim/RealSense-ToolTracker&type=Date" />
  <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=stytim/RealSense-ToolTracker&type=Date" />
</picture>
