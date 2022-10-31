PclWalkerViewer
===============

PclWalkerViewer is a C++20 utility executable that walk (recursively on demand) a directory and sequentially display
the point clouds its contains into a PCL 3D viewer. It supports PCD and PLY format.
The viewer is controlled by keyboard inputs.

## Build and install

```sh
mkdir build && cd build &&\
cmake -DCMAKE_BUILD_TYPE=Release .. &&\
make &&\
sudo make install
```
# Usage

Upon call, you need to provide the directory to walk into and search clouds:



## Keymap and parameters

You can control the viewer using the following inputs of the keyboard:
- [<-] [->] (Left or Right arrow) to display the previous/next cloud
- [d] to change the color of the cloud
- [t] to toggle the display of origin coordinate system
- [Up] to increase the size of the origin coordinate system
- [Down] to increase the size of the origin coordinate system

## Dependencies

- At least [Point Cloud Libary](https://www.pointclouds.org/) 1.9.
- At least [Boost](https://www.boost.org/) 1.71.0.
- A compiler supporting [C++20](https://en.cppreference.com/w/cpp/compiler_support/20) (concepts and range libraries are required).

## Feedback

Don't hesitate if you have any suggestions for improving this project, or if you find any error. I will be glad to
hear from you. Contributions are welcomed :)

## License

Distributed under the MIT Software License (X11 license).
See accompanying file LICENSE.
