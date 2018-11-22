## Added by Chris
This project makes use of the fantastic OpenPose project by CMU-Perceptual-Computing-Lab, which detects human skeleton points from 2D image feeds, such as images, videos and webcam.
https://github.com/CMU-Perceptual-Computing-Lab/openpose

To use this code with the OpenPose library, one has to build this code (a.k.a. user custom code) together with the library code from source. I have only tried building it using Visual Studio 2017 and Cuda SDK 9 on Windows 10.
https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation.md
https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/user_code/README.md

Skeleton points json output format:
https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md



Sections below are from the OpenPost library.



Adding and Testing Custom Code
====================================



## Purpose
You can quickly add your custom code into this folder so that quick prototypes can be easily tested without having to create a whole new project just for it.



## How-to
1. Install/compile OpenPose as usual.
2. Add your custom *.cpp / *.hpp files here,. Hint: You might want to start by copying the [OpenPoseDemo](../openpose/openpose.cpp) example or any of the [examples/tutorial_api_cpp/](../tutorial_api_cpp/) examples. Then, you can simply modify their content.
3. Add the name of your custom *.cpp / *.hpp files at the top of the [examples/user_code/CMakeLists.txt](./CMakeLists.txt) file.
4. Re-compile OpenPose.
```
# Ubuntu/Mac
cd build/
make -j`nproc`
# Windows
# Close Visual Studio, re-run CMake, and re-compile the project in Visual Studio 
```
5. **Run step 4 every time that you make changes into your code**.



## Running your Custom Code
Run:
```
./build/examples/user_code/{your_custom_file_name}
```
