## Pan&Zoom

**Pan&amp;Zoom** is a system which enables controlling the display of content on the computer screen using hand gestures.

The system is based on two modules:  
- [MediaPipe](https://google.github.io/mediapipe/) framework - used to detect the key points of the palms and recognize hand gestures
- [Image-viewer](https://github.com/palacaze/image-viewer) - manages the display of graphical content and the execution of pan and zoom actions  

Two interaction modalities were designed and implemented for graphical content management:  
- **MH (Multi-hand zoom)** - allows panning with EITHER hand, while BOTH hands have to be used simultaneously for zooming  
- **SH (Single-hand zoom)** - only the LEFT hand is used for panning and the RIGHT hand for zooming

<table>
  <tr>
     <td align="center">Modality</td>
     <td align="center">Pan</td>
     <td align="center">Zoom-In</td>
     <td align="center">Zoom-Out</td>
  </tr>
  <tr>
    <td align="center">MH</td>
    <td align="center"><img src="https://user-images.githubusercontent.com/61973790/222386894-a1b2e982-a18c-421d-9cda-c57cf760d98c.png"></td>
    <td align="center"><img src="https://user-images.githubusercontent.com/61973790/222395813-8b40a362-afc6-47ac-8c8c-07252c674629.png"></td>
    <td align="center"><img src="https://user-images.githubusercontent.com/61973790/222395829-29c732be-bb19-4aa0-9bd7-7f64385e65fc.png"></td>
  </tr>
  <tr>
    <td align="center">SH</td>
    <td align="center"><img src="https://user-images.githubusercontent.com/61973790/222395685-15c6835a-09d7-475a-b1e5-311a75628ef9.png"></td>
    <td align="center"><img src="https://user-images.githubusercontent.com/61973790/222395967-95d0baf0-bbe2-4702-b460-9e589aa9c666.png"></td>
    <td align="center"><img src="https://user-images.githubusercontent.com/61973790/222395983-f5be9f35-8fcb-42a4-9b4f-66b7f2dce287.png"></td>
  </tr>
 </table>

## HCI experiment

In order to compare the **efficiency** and **interaction workload** of the mentioned modalities, an HCI experiment was conducted with **30 test users** that included corresponding tasks from three different complexity categories **(Easy, Medium and Hard)**.

[Statistical analysis](https://github.com/MMaricevic64/Pan_Zoom/tree/master/statistical%20analysis/average%20test%20time%20analysis) of the obtained results showed that the **SH modality performs panning and zooming actions significantly more efficiently**. It was also shown that the **task's complexity significantly impacts the time of its execution**.  

The above results were additionally supported by the [analysis of standardized questionnaires](https://github.com/MMaricevic64/Pan_Zoom/tree/master/statistical%20analysis/questionnaires%20analysis), which showed that **the MH modality leads to a significantly higher workload**. The test users prefer the SH modality because it requires less physical effort, causes a lower level of frustration, and provides better efficiency.

<table>
  <tr>
     <td align="center">Easy</td>
     <td align="center">Medium</td>
     <td align="center">Hard</td>
  </tr>
  <tr>
    <td align="center"><img src="https://user-images.githubusercontent.com/61973790/222899718-5f12568b-a989-4f65-9c99-9b103b7cffe8.png"></td>
    <td align="center"><img src="https://user-images.githubusercontent.com/61973790/222899733-bcc8b0f2-e96e-4f5f-9180-a3c858f5a47c.png"></td>
    <td align="center"><img src="https://user-images.githubusercontent.com/61973790/222899751-f7c8bc6b-9bed-44b0-a28d-f0a3531676d5.png"></td>
  </tr>
</table>

It is necessary to delete all previously obtained [results](https://github.com/MMaricevic64/Pan_Zoom/tree/master/results) before conducting the new HCI experiment.

### Add new tasks

1.  Create an image containing rectangles of different colors using the MS Paint application or some other tool (preferably with the highest possible resolution)
2.  Number the rectangles arbitrarily in ascending order as shown in examples above
3.  Save the image under a specific name (eg <image_name>.png) and add it to the [pictures](https://github.com/MMaricevic64/Pan_Zoom/tree/master/data/pictures) folder
4.  Create a .txt file, give it the **SAME** name as the created image and save it in the [pictures](https://github.com/MMaricevic64/Pan_Zoom/tree/master/data/pictures) folder
5.  Look for the corresponding color in RGB format for each rectangle in the picture following ascending numerical order and add it to the created .txt file
6.  After the last added RGB value, save the contents of the .txt file

The order of tasks was not the same for all test users. 6 different [task orders](https://github.com/MMaricevic64/Pan_Zoom/tree/master/data) have been made according to the principle of Balanced latin squares (each order was used 5 times). Each task is represented by a path to an image of a certain complexity and the modality by which it must be performed **(eg data/pictures/1 multi-hand-zoom)**.

7.  Create a new task order file (.txt), give it the specific name (eg <test\-sequence\-X>.txt) and save it in the [data](https://github.com/MMaricevic64/Pan_Zoom/tree/master/data) folder
8.  Add already created tasks **(number of pictures x 2 modalities)** and 2 new tasks **(path to the new image x 2 modalities)** in the new task order file respecting the principle of Balanced latin squares
9.  Add 2 new tasks to the already created task orders respecting the principle of Balanced latin squares

## System requirements

- **Operating system:** Linux or Windows  
- **Hardware requests:** desktop or laptop computer, 720p HD camera (at least)  
- **Software requests:** depending on used OS

### Linux

**1. Download and extract Pan&Zoom system code** (using git clone or download ZIP option)

**2. Python 3 and *numpy* library**

     sudo apt-get install python3.10
     sudo apt install python3-pip
     pip3 install numpy
    
**3. Bazel 5.2.0**
  - https://bazel.build/install/ubuntu?fbclid=IwAR3JMFT8iRm7a5aAlMoVkQ589eLbki41jBaZCNulKpLzyOnh--wpI1K-hw#install-packages
  - https://github.com/bazelbuild/bazel/releases/tag/5.2.0 - Bazel 5.2.0 binary installer
  
**4. OpenCV**

    cd Pan_Zoom
    cd mediapipe
    chmod +x setup_opencv.sh
    ./setup_opencv.sh

**5. OpenGL ES**
  - https://developers.google.com/mediapipe/framework/getting_started/gpu_support#opengl_es_setup_on_linux_desktop

**6. Qt**
  - https://www.qt.io/download-open-source - Download the **Qt Online Installer** and place it on Desktop

    ```
    cd Desktop
    chmod +x <name_of_installation_package>.run
    ./<name_of_installation_package>.run (eg qt-unified-linux-x64-4.5.1-online)
    ```
  - If the installation does not start, it is necessary to write the following command and then run the Qt installer again:
     ```
     sudo apt install --reinstall libxcb-xinerama0
     ```
- Log in/register in order to continue with the installation of the Qt software
- Select the **"Custom Installation"** option and any **Qt 6 version**
- From the selected version, mark only the **"Desktop gcc 64-bit"** option
- Continue and complete the installation process

### Windows

**1. Download and extract Pan&Zoom system code** (using git clone or download ZIP option)

**2. MSYS2**
  - https://www.msys2.org/ - Download the **MSYS2 installer**, run it as administrator and complete the installation process
  - Run MSYS2 MSYS terminal and enter the following commands:

    ```
    pacman -Syu
    pacman -Su
    pacman -S mingw-w64-x86_64-toolchain
    ```
  - Add paths to **bin** folders to **"Path"** environment variable (eg *C:\msys64\usr\bin* and *C:\msys64\mingw64\bin*)
  
**3. Python 3 and *numpy* library**
  - https://www.python.org/downloads/windows/ - Download and run the **Windows Installer (64-bit)** for Python 3 (eg 3.10.8)
  - Select option **„Add python.exe to PATH“** and continue with the installation process 
    **Note:** If this option was not selected, it is necessary to manually add the paths to the python.exe file and its scripts to **"Path"** environmental variable  
    (eg *C:\Users\Mario\AppData\Local\Programs\Python\Python310\\* and *C:\Users\Mario\AppData\Local\Programs\Python\Python310\Scripts\\*)
  - Open Command Prompt and enter the following command:
  
    ```
    pip install numpy
    ```

**4. Microsoft C++ Build Tools**
  - https://visualstudio.microsoft.com/visual-cpp-build-tools/ - Download and run **MS C++ Build Tools installer**
  - In the **"Workload"** menu select  **"Desktop development with C++"** option
  - Move to **„Individual components“** menu and select the following options:
  
      - **MSVC v143 – VS 2022 C++ x64/x86 build tools (v14.33-17.3)** instead of the default option
      - **C++ Cmake tools for Windows** (if not already selected)
      - **Windows 10 SDK (10.0.19041.0)** or **Windows 11 SDK (10.0.22000.0)** instead of the default option
   
   - Building of the MediaPipe module has been verified with versions above

**5. Bazel 5.2.0**
  - https://github.com/bazelbuild/bazel/releases/tag/5.2.0 - Download **bazel-5.2.0-windows-x86_64.exe** and rename it to **"bazel.exe"**
  - Add the path to the folder where **"bazel.exe"** is located to the **"Path"** environmental variable
  - Create 3 new environmental variables:
  
      - **Variable name: BAZEL_VC**  
        **Variable value:** path to the VC folder in the installed MS C++ Build Tools (eg *C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC*)
       
      - **Variable name: BAZEL_VS**  
        **Variable value:** path to the Build Tools folder in the installed MS C++ Build Tools (eg *C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools*)
        
      - **Variable name: BAZEL_VC_FULL_VERSION**  
        **Variable value:** 14.33.31629 (installed version of MS C++ Build Tools)
        
**6. OpenCV**
  - https://opencv.org/releases/ - Find and download **OpenCV 3.4.10** (recommended) or any other OpenCV 3 for Windows
  - Install OpenCV (**C:/** disk partition recommended)
  
      - If some other version of OpenCV 3 is installed (not 3.4.10), it is required to change OPENCV_VERSION in the [opencv_windows.BUILD](https://github.com/MMaricevic64/Pan_Zoom/blob/master/mediapipe/third_party/opencv_windows.BUILD) file
      - If the OpenCV 3 is not installed on the C:/ partition, it is necessary to change the path to the OpenCV build folder (line 274) in the [WORKSPACE](https://github.com/MMaricevic64/Pan_Zoom/blob/master/mediapipe/WORKSPACE) file

**7. Cmake**
  - https://cmake.org/download/ - Download and run **Cmake Windows x64 Installer**
  - Select the option to automatically add the path to the **bin** folder in **"Path"** environmental variable and complete the installation process

**8. Qt**
  - https://www.qt.io/download-open-source - Download and run the **Qt Online Installer**
  - Log in/register in order to continue with the installation of the Qt software
  - Select the **"Custom Installation"** option and any **Qt 6 version**
  - From the selected version, mark only the **"MSVC 2019 64-bit"** option
  - Continue and complete the installation process
  - After successful installation, manually add the paths to the **bin** and **lib** folders from the MSVC 2019 compiler in **"Path"** environmental variable (eg *E:\Qt\6.4.0\msvc2019_64\bin* and *E:\Qt\6.4.0\msvc2019_64\lib*)

## Build system modules

### Linux
#### MediaPipe
   ```
   cd Pan_Zoom
   cd mediapipe
   chmod +x build_pan_zoom.sh linux
   ./build_pan_zoom.sh linux
   ```
#### Image-viewer
   ```
   cd Pan_Zoom
   cd image-viewer
   mkdir build
   cd build
   cmake .. -DCMAKE_PREFIX_PATH=“<path_to_the_gcc_64_folder_inside_the_Qt>“ (eg /home/mariopc/Qt/6.4.2/gcc_64/)
   make
   ```
### Windows
#### MediaPipe
   ```
   cd Pan_Zoom
   cd mediapipe
   bash build_pan_zoom.sh windows <path_to_the_python.exe_file> (eg C://Users//Mario//AppData//Local//Programs//Python//Python310//python.exe)
   ```
#### Image-viewer
- Open **„Development Command Prompt for VS“** and enter the following commands:

   ```
   cd Pan_Zoom
   cd image-viewer
   mkdir build
   cd build
   cmake .. -DCMAKE_PREFIX_PATH="<path_to_the_MSVC_folder_inside_the_Qt>" (eg E:\\Qt\\6.4.0\\msvc2019_64\\, use \\ instead of \ or /)
   msbuild PalImageViewer.sln -p:Configuration=Release
   ```
## Run Pan&Zoom
- **Modality training** - 2 simple tasks (1 per each modality)

- **HCI experiment** 
    - all created tasks **(number_of_pictures x 2 modalities)**, starts with a specific order of tasks **(test-sequence-X.txt)**
    - the HCI experiment results are recorded in the [results](https://github.com/MMaricevic64/Pan_Zoom/tree/master/results) folder
    - before conducting a new experiment, it is necessary to delete all [results](https://github.com/MMaricevic64/Pan_Zoom/tree/master/results) from the previous HCI         experiment
    
- **Pan&Zoom termination** - in order to properly close the communication between two system modules, it is necessary to do the following:
    - Press the **"Exit"** button on the graphical interface of Image-viewer module
    - Close the MediaPipe module in terminal using the **"Ctrl+C"** key combination 
### Linux
#### Modality training
   ```
   cd Pan_Zoom
   chmod +x train-experiment.sh
   ./train-experiment.sh linux
   ```
#### HCI experiment
   ```
   cd Pan_Zoom
   chmod +x run-experiment.sh
   ./run-experiment.sh linux data/test-sequence-X.txt
   ```
### Windows
#### Modality training
   ```
   cd Pan_Zoom
   bash train-experiment.sh windows
   ```
#### HCI experiment
   ```
   cd Pan_Zoom
   bash run-experiment.sh windows data/test-sequence-X.txt
   ```
