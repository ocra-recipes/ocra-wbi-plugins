---
layout: post
title:  "iCub Quickstart Guide"
date:   2016-11-26 11:00
categories: icub
author: ryan
---

# Introduction
In this guide we will show you how to get OCRA and it's iCub plugins installed and running as well as write our very first client. For an overview on how OCRA works and the server-client paradigm please check out this article: [link].

# Installation
With iCub, installing OCRA is a snap thanks to the [codyco-superbuild](https://github.com/robotology/codyco-superbuild). First you need to follow their [installation instructions](https://github.com/robotology/codyco-superbuild#installation) depending on your particular platform. For now OCRA for iCub has only been tested on OSX (see Eigen 3.3 Warning below) and Linux, so Windows users be aware that you may have significant difficulty getting everything working.

Assuming all of the `codyco-superbuild` dependencies are installed you have cloned `codyco-superbuild` somewhere, all that remains is to run the following:

```
cd [path_to_where_you_cloned_superbuild]/codyco-superbuild
mkdir build
cd build/
cmake .. -DCODYCO_BUILD_OCRA_MODULES=1
make
```

Once superbuild completes you are all set. If you already have built `codyco-superbuild`, just run the cmake command above and rebuild everything.

## Warning about Eigen 3.3 
The latest release of the linear algebra library [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) is currently incompatible with OCRA and if you have it installed then you will need to remove it and replace it with 3.2.X. We are working on a fix.

# Test the install
To test OCRA on iCub we need to open a few terminals...

**Terminal 1**

```
yarpserver --write
```

**Terminal 2**

```
cd [path_to_where_you_cloned_icub_gazebo]/icub-gazebo/world
gazebo icub.world
```

**Terminal 3**

```
ocra-icub-server --floatingBase --taskSet wholeBodyTaskSet
```

If all goes well you should see the the following output in **Terminal 3**

[photo here]

and the robot should not move. Hurray!


# Launching a client
Assuming that your controller-server is still running from the installation test, we can now launch a client to see the robot actually move. 

**Terminal 4**

```
example-client
```

You should see the left hand start to move in a square pattern. If this is the case then you are all ready to build your own client, but first, be sure to `ctrl+c` the `example-client`. 

# Creating your own clients
With OCRA and iCub, creating your own controller clients is easy-peasy. First, navigate to where you would like to build your client project. Then all that is left is to pick a snazzy client name, like, "hello-world", and use the `icub-client-generator` to scaffold out the project for you.

```
cd /home/user_name/ocra_clients/
icub-client-generator hello-world
```

You should get the following output:
```
Scaffolding client code for: hello-world
-- Client class will be named: HelloWorldClient
```

## iCub client structure

When you create a new client project you will automatically have the following project structure:

```
hello-world/
├── CMakeLists.txt
├── build
├── cmake
│   └── Modules
│       └── AddUninstallTarget.cmake
├── include
│   └── hello-world
│       └── HelloWorldClient.h
└── src
    ├── HelloWorldClient.cpp
    └── main.cpp
```

This is compilable "as is" but won't do anything until you fill in the functionality. To make sure that your `cmake` paths are correct run,
```
cd hello-world/build
cmake ..
make 
```
If everything builds, then we are good. Otherwise you probably are missing some environment variables from the `codyco-superbuild` installation steps. 

Now the bread and butter of the clients are the inherited `iCubClient` classes, which in this case is simply, `HelloWorldClient.cpp` and `HelloWorldClient.h`. This is where we will make our changes. 

## Make a reaching movement
So for our first client we simply want the iCub to reach down towards the floor with its right hand. To do so we will need to control the `RightHandCartesian` task which was added by the [`wholeBodyTaskSet.xml`](https://github.com/ocra-recipes/ocra-wbi-plugins/blob/master/ocra-icub-server/app/robots/icubGazeboSim/taskSets/wholeBodyTaskSet.xml), and make a trajectory for it to follow. These two objectives are made easy by [`TaskConnection`](https://github.com/ocra-recipes/ocra-recipes/blob/master/ocra-recipes/include/ocra-recipes/TaskConnection.h) and [`TrajectoryThread`](https://github.com/ocra-recipes/ocra-recipes/blob/master/ocra-recipes/include/ocra-recipes/TrajectoryThread.h).

**HelloWorldClient.h**

```
#ifndef HELLOWORLDCLIENT_H
#define HELLOWORLDCLIENT_H

#include <ocra-icub/IcubClient.h>
#include <ocra-recipes/TrajectoryThread.h>
#include <ocra-recipes/ControllerClient.h>


class HelloWorldClient : public ocra_recipes::ControllerClient
{
DEFINE_CLASS_POINTER_TYPEDEFS(HelloWorldClient)

public:
    HelloWorldClient (std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod);
    virtual ~HelloWorldClient ();

protected:
    virtual bool initialize();
    virtual void release();
    virtual void loop();

private:
    ocra_recipes::TaskConnection::Ptr rightHandTask;
    ocra_recipes::TrajectoryThread::Ptr rightHandTrajectory;
};


#endif // HELLOWORLDCLIENT_H
```

### Code explained
...


**HelloWorldClient.cpp**

```
#include "hello-world/HelloWorldClient.h"
HelloWorldClient::HelloWorldClient(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)
: ocra_recipes::ControllerClient(modelPtr, loopPeriod)
{
    // add your code here...
}

HelloWorldClient::~HelloWorldClient()
{
    // add your code here...
}

bool HelloWorldClient::initialize()
{
    // Set the task name
    std::string rightHandTaskName("RightHandCartesian");

    // Create the TaskConnection to communicate with the right hand task
    rightHandTask = std::make_shared<ocra_recipes::TaskConnection>(rightHandTaskName);

    // Get the current position of the right hand task frame
    Eigen::Vector3d rightHandStartPosition = rightHandTask->getTaskState().getPosition().getTranslation();

    // Make an end position for the right hand which is the same as the start position but with the "z" component 30cm above the ground.
    Eigen::Vector3d rightHandEndPosition = rightHandStartPosition;
    rightHandEndPosition(2) = 0.3;

    // Create the right hand TrajectoryThread
    rightHandTrajectory = std::make_shared<ocra_recipes::TrajectoryThread>(10, rightHandTaskName, rightHandEndPosition, ocra_recipes::TRAJECTORY_TYPE::MIN_JERK, ocra_recipes::TERMINATION_STRATEGY::REVERSE_STOP);

    rightHandTrajectory->start();

    return true;
}

void HelloWorldClient::release()
{
    // add your code here...
}

void HelloWorldClient::loop()
{
    // When the trajectory thread is finshed, stop the client and close.
    if (!rightHandTrajectory->isRunning()) {
        std::cout << "Closing the hello-world client." << std::endl;
        stop()
    }
}

```

### Code explained

...
