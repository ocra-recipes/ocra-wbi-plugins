# ocra-wbi-plugins
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.1118370.svg)](https://doi.org/10.5281/zenodo.1118370)

Controller implementations and plugins for communicating between the whole body controller libraries developed at ISIR, [`ocra-recipes`](https://github.com/ocra-recipes/ocra-recipes), and the iCub Whole Body Interface, [`WBI`](https://github.com/robotology/wholebodyinterface), libraries.

#### Build Status
| master | dev |
|:------:|:---:|
| [![Build Status](https://travis-ci.org/ocra-recipes/ocra-wbi-plugins.svg?branch=master)](https://travis-ci.org/ocra-recipes/ocra-wbi-plugins) | [![Build Status](https://travis-ci.org/ocra-recipes/ocra-wbi-plugins.svg?branch=dev)](https://travis-ci.org/ocra-recipes/ocra-wbi-plugins) |




## Code Structure
Here's how everything is laid out...

### [ocra-icub](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/ocra-icub)

Interface libraries between WBI and ocra. The interface consists primarily of an inherited [`Model`](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/ocra-icub/src/ocraWbiModel.cpp) class who's virtual functions are implemented using WBI's functions. A [`Utilities`](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/ocra-icub/src/Utilities.cpp) class is also available for converting between various dynamic representations.


### [ocra-icub-server](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/ocra-icub-server)

This is an implementation of the ocra-server interface which gets the robot specific model info, calls the `computeTorque()` function and then sends it to the robot. The rest of the code is designed around yarp `RFModule` and `RateThread`, to parse command line args, and talk to the WBI. At the core of this code is the implementation of [`ocra_recipes::ControllerServer`](https://github.com/ocra-recipes/ocra-recipes/tree/master/ocra-recipes/src/ContControllerServer.cpp) class. The rest of the code is icub specific.  


### [ocra-icub-clients](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/ocra-icub-clients)

### [cmake](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/cmake)

This folder provides various cmake modules which allow cmake to find various packages and build packages from these projects.


## Installation

Everything here should be built with [`codyco-superbuild`](https://github.com/robotology/codyco-superbuild) to make sure you have all of the required libs.

## Usage

See the video tutorials... Work in progress.

## Authors

### current developers

 - [Ryan Lober](https://github.com/rlober)
 - [Antoine Hoarau](https://github.com/ahoarau)
 - [Jorhabib Eljaik](https://github.com/jeljaik)
 - [Silvio Traversaro](https://github.com/traversaro)


### past developers

 - [Darwin Lau](https://github.com/darwinlau)
 - [Mingxing Liu](https://github.com/mingxing-liu)
