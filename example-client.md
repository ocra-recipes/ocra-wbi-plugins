---
layout: page
title:  example-client
permalink: /example-client/
---

# A simple client example

This is the simplest among all the clients and will serve as a sort of tutorial. We will therefore break it down into the different files and pieces of code from the source files.

#### Objective:
> To take the left hand to three different targets in the air while maintaining
> balance and the right hand at a fixed target.

Running the `example-client`
------





Let us break down this client into its main components.

CMakeLists.txt
------
The first thing we need to look into is of course the `CMakeLists.txt` file for this project (`ocra-icub-clients/example-client/`)

First the name of the project is set and source and headers are stored in the variable `folder_header` and `folder_source`.

```
SET(PROJECTNAME example-client)
PROJECT(${PROJECTNAME} CXX)

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/${PROJECTNAME}/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})
```
We will always need to add the `YARP` headers besides the project's (doh!). The `ocra-wbi-plugins` headers can be retrieved with the `CMake` variable `OcraIcub_INCLUDE_DIRS` while the `ocra-recipes` library headers are stored in the variable `OcraRecipes_INCLUDE_DIRS` as shown next:

```
INCLUDE_DIRECTORIES(
${PROJECT_SOURCE_DIR}/include
${YARP_INCLUDE_DIRS}
${OcraIcub_INCLUDE_DIRS}
${OcraRecipes_INCLUDE_DIRS}
)
```

The executable to the project is added next using the specified source files...

```
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})
```

... and the necessary libraries linked. The minimum required are `YARP`'s, `ocra-recipes` as well as the `ocra-wbi-plugins`libraries

```
LIST(APPEND link_libs   ${YARP_LIBRARIES}
                        ${OcraRecipes_LIBRARIES}
                        ocra-icub
                        )

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${link_libs})
```
At last, we indicate `CMake` to install the project in the `/bin` directory

```
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
```

example-client.cpp
------
This is the main file for this example. It accomplishes the following tasks:

  + Check the `yarp` network is up.
  + Create a model of the robot.
  + Instantiates the controller client.
  + Instantiates the client manager.
  + Instantiates and configures a `yarp::os::ResourceFinder` object used to find configuration files and handle parameters passed to the module which are particular to the robot being used.
  + Provides `help` to the user.
  + Finally Starts/runs the client thread.

ExampleClient.cpp
------
