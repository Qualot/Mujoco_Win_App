# Mujoco_Win_App
mujoco viewer sends json data via udp

![thumbnail](https://raw.githubusercontent.com/Qualot/Mujoco_Win_App/main/media/thumbnail.png)

# Installation
install mujoco on windows https://mujoco.readthedocs.io/en/latest/programming/#getting-started

# Build
```
cd Mujoco_Win_App
mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="\location\of\vcpkg\scripts\buildsystems\vcpkg.cmake"
cmake --build . --config Release
cd ..
cp \location\to\glfw3.dll .\build\Release\
cp \location\to\bin\mujoco.dll .\build\Release\
```

# Execution
```
.\build\Release\Mujoco_Win_App.exe .\models\scene.xml sendIP sendPort recvPort
(e.g. .\build\Release\Mujoco_Win_App.exe .\models\scene.xml 127.0.0.1 5005 5006)
```