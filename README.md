# AprilTag Position

author of cameraPosition is JachinShen (jachinshen@foxmail.com)

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

## Function: detect the apriltags on the ground to get camera's position (location and direction)

## Usage:

> cd path/to/apriltags
> make
> ./build/bin/demo

Then it will show you a video with tags drawed, and the terminal will print location and direction

Example code is in example/demo.cpp
