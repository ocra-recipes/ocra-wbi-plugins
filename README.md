#ocra-wbi-plugins

Controller implementations and plugins for communicating between the whole body controller libraries developed at ISIR, [`ocra-recipes`](https://github.com/ocra-recipes/ocra-recipes), and the iCub Whole Body Interface, [`WBI`](https://github.com/robotology/wholebodyinterface), libraries.


##Code Structure

Here's how everything is laid out...

###libs

- [ocraWbiPlugins](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/libs/ocraWbiPlugins)

    - Interface libraries between WBI and ocra. The interface consists primarily of an inherited [`Model`](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/libs/ocraWbiPlugins/src/ocraWbiModel.cpp) class who's virtual functions are implemented using WBI's functions. A [`utilities`](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/libs/ocraWbiPlugins/src/ocraWbiUtil.cpp) class is also available for converting between various dynamic representations.

- [taskSequences](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/libs/taskSequences)

    - This library contains a dictionary of "sequences" which are hard coded sets of tasks and control logic. New sequences go in the subdirectory [`sequences/`](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/libs/taskSequences/include/taskSequences/sequences) and are included in [`sequenceLibrary.h`](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/libs/taskSequences/include/sequenceLibrary.h) and their defintions are included in [`sequenceLibrary.cpp`](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/libs/taskSequences/src/sequenceLibrary.cpp).

- [ocraObservers](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/libs/ocraObservers)

    - *under construction*
    - This library will eventually provide the base set of classes for creating "observerThreads" which can be launched in parallel to the controller for recording data in a way which will not affect controller performance.

- [activityManagers](https://github.com/ocra-recipes/ocra-wbi-plugins/#)

    - *under construction*
    - This library will provide the base classes for creating "activityManagerModules" which will provide high level control logic and task set management.


###modules

- [wocraController](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/modules/wocraController)

    - wocra controller implementation for the iCub. This module builds an executable which controls the iCub using a weighted QP controller. More info [here](https://github.com/ocra-recipes/ocra-recipes).

- [gocraController](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/modules/gocraController)

    - gocra controller implementation for the iCub. This module builds an executable which controls the iCub using a generalized hierarchical QP controller. More info [here](https://github.com/ocra-recipes/ocra-recipes).

- [ocraObservers](https://github.com/ocra-recipes/ocra-wbi-plugins/tree/master/modules/ocraObservers)

    - *under construction*
    - "ObserverThreads" which can be launched in parallel to the controller for recording data in a way which will not affect controller performance will be held here.


###cmake

This folder provides various cmake modules which allow cmake to find various packages and build packages from these projects.


##Installation

Superbuild! More details soon...

##Usage

See the video tutorials... Work in progress.

##Authors

###current developers

 - [Ryan Lober](https://github.com/rlober)
 - [Antoine Hoarau](https://github.com/ahoarau)
 - [Silvio Traversaro](https://github.com/traversaro)


###past developers

 - [Darwin Lau](https://github.com/darwinlau)
 - [Mingxing Liu](https://github.com/mingxing-liu)
