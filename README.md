# Description

3D Drone Delivery game where you must collect the mystical spheres and deliver them to a safety zone marked by a red target.  
Avoid trees and bunkers and reach your destination as fast, and as low to the ground as possible!

# How to Run (Windows)

#### Prerequisites
Install [Cmake](https://cmake.org/download/)  
Install [Visual Studio](https://visualstudio.microsoft.com/downloads/)

1. Clone or download the zip
2. Run the batch file in the scripts/ folder
3. Open the .sln file in the build directory in the root and press F5 to run or press the Local Windows Debugger Button

# Features

- Drone mechanics
- 3D Obstacle collisions
- Terrain generated with 2D Perlin Noise
- Custom shaders with GLSL and OpenGL

# Controls

### Drone

- WASD = move
- Up / Down = pitch camera
- Space = drop the package
- Shift = descend faster

### Terrain Editor

Using shift inverses the action of the keybind.
Terrain:
- R = increase / decrease resolution
- T = increase / decrease noise offset
- Y = increase / decrease noise scalar
- U = increase / decrease map height 
- L = increase / decrease map length 
Obstacles:
- Z = increase / decrease trees count
- X = increase / decrease bunkers count

![screenshot_gameplay](readme/preview_drone.gif)
