# Cyclone Physics

The Physics engine that accompanies the book "Game Physics Engine Development" by Ian Millington.
This version uses a modern conan/cmake based build system and should build out of the box on Windows and Linux systems.

## Requirements

Development is done with
* Conan 2.x
* CMake 3.27.x
* Windows: Visual Studio 2022 Community Edition
* Linux: Make

## Building

Conan install for debug and release builds
```
conan install . --build=missing --settings=build_type=Debug
conan install . --build=missing --settings=build_type=Release
```

CMake project generation

On Windows
```
cmake --preset conan-default
```

On Linux
```
cmake --preset conan-debug
cmake --preset conan-release
```

On Windows, open the generaed solution file in the build folder.
On Linux use
```
cmake --build build/Debug
cmake --build build/Release
```

## Screenshots - Demos

Ballistic
![Screenshot 1](./doc/ballistic_screenshot_1.jpg)

BigBallistic
![Screenshot 2](./doc/bigballistic_screenshot_1.jpg)

Blob
![Screenshot 3](./doc/blob_screenshot_1.jpg)

Bridge
![Screenshot 4](./doc/bridge_screenshot_1.jpg)

Explosion
![Screenshot 5](./doc/explosion_screenshot_1.jpg)

Fireworks
![Screenshot 5](./doc/fireworks_screenshot_1.jpg)

FlightSim
![Screenshot 5](./doc/flightsim_screenshot_1.jpg)

Fracture
![Screenshot 5](./doc/fracture_screenshot_1.jpg)
![Screenshot 5](./doc/fracture_screenshot_2.jpg)

Platform
![Screenshot 5](./doc/platform_screenshot_1.jpg)

Ragdoll
![Screenshot 5](./doc/ragdoll_screenshot_1.jpg)

Sailboat
![Screenshot 5](./doc/sailboat_screenshot_1.jpg)