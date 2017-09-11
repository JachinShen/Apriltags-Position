# AprilTag Position

author of CameraPosition is JachinShen (jachinshen@foxmail.com).

## AprilTags library

AprilTags were developed by Professor Edwin Olson of the University of
Michigan.  His Java implementation is available on this web site:
  http://april.eecs.umich.edu.

Olson's Java code was ported to C++ and integrated into the Tekkotsu
framework by Jeffrey Boyland and David Touretzky.

See this Tekkotsu wiki article for additional links and references:
  http://wiki.tekkotsu.org/index.php/AprilTags
  
This C++ code was further modified by
Michael Kaess (kaess@mit.edu) and Hordur Johannson (hordurj@mit.edu)
and the code has been released under the LGPL 2.1 license.

## Function: 

Detect the apriltags on the ground to get camera's position (location and direction).

## Usage:

> cd path/to/apriltags\
> make\
> ./build/bin/demo

Then it will show you a video with tags drawed, and the terminal will print location and direction.

## Example

Example code is in example/demo.cpp.

## Details

For more details, please go to [My Blog](https://jachinshen.github.io/robomaster/2017/09/06/%E5%9F%BA%E5%9C%B0%E4%BA%8C%E7%BB%B4%E7%A0%81%E5%AE%9A%E4%BD%8D.html)
