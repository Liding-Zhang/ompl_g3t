The Open Motion Planning Library (OMPL)
=======================================

Linux / macOS [![Build Status](https://travis-ci.org/ompl/ompl.svg?branch=main)](https://travis-ci.org/ompl/ompl)
Windows [![Build status](https://ci.appveyor.com/api/projects/status/valuv9sabye1y35n/branch/main?svg=true)](https://ci.appveyor.com/project/mamoll/ompl/branch/main)

Visit the [OMPL installation page](https://ompl.kavrakilab.org/core/installation.html) for
detailed installation instructions.

OMPL has the following required dependencies:

* [Boost](https://www.boost.org) (version 1.58 or higher)
* [CMake](https://www.cmake.org) (version 3.5 or higher)
* [Eigen](http://eigen.tuxfamily.org) (version 3.3 or higher)

The following dependencies are optional:

* [ODE](http://ode.org) (needed to compile support for planning using the Open Dynamics Engine)
* [Py++](https://github.com/ompl/ompl/blob/main/doc/markdown/installPyPlusPlus.md) (needed to generate Python bindings)
* [Doxygen](http://www.doxygen.org) (needed to create a local copy of the documentation at
  https://ompl.kavrakilab.org/core)

To change the iteration time lagging, replace the following condition in **RandomGeomrtricGraph.cpp**:

   *while (newSamples_.size() < numNewStates && !terminationCondition);*
    
  $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$ $\Downarrow$

   *while (newSamples_.size() < numNewStates);*

## Installation of OPENRAVE

install python 2.7
    
    sudo apt-get update
    sudo apt-get install python2

check default python version

    python --version && python3 --version
Python 2.7 as the default version by creating a symbolic link

    sudo update-alternatives --install /usr/bin/python python /usr/bin/python2 1
    sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 2


manually select python version 2.7, you can change it back when OPENRAVE installed

    sudo update-alternatives --config python


install of OPENRAVE repos

    cd ~
    git clone https://github.com/crigroup/openrave-installation
    cd openrave-insstallation
install and build OPERAVE in order

    sudo ./install-dependencies.sh
    sudo ./install-osg.sh
    sudo ./install-fcl.sh
    sudo ./install-openrave.sh

## OMPL and PDT Installation
Once dependencies are installed, you can build OMPL on Linux, macOS,
and MS Windows. Go to the top-level directory of OMPL and type the
following commands

    mkdir build
    cd build
You can use macro commands to toggle the code you want to compile

    cmake -DCMAKE_BUILD_TYPE=Debug ..
    make -j 4 # depends on core you are using

install build into /usr/local folder

    sudo cmake --install .

switch to pdt folder

    cd ../..
    cd pdt/build
    cmake -DPDT_OMPL_DIR=/usr/local -DPDT_OPEN_RAVE=ON ..
    make -j 4 # depends on core you are using

show OPENRAVE visualization result according to the open_rave_demo config  

    ./bin/open_rave_gui -c ../parameters/demo/open_rave_demo.json
    
in case you are old version of PDT: change in open_rave_demo.json

    "viewer": "qtcoin",
    to
    "viewer": "qtosg",

show visualization result according to the demo config    

    ./bin/visualization -c ../parameters/demo/visualization_demo.json 

run benchmark result according to the demo config (also fit for OPENRAVE)

    ./bin/benchmark -c ../parameters/demo/benchmark_demo.json 

reproduce benchmark pdf according to the demo config

creat report_config.json in demo/ folder

    ./bin/benchmark_report -c ../parameters/demo/report_config.json

## Some useful **seeds** to test:

| Dimension | RandomRectangles  | WallGap | DividingWalls | Narrow Passages |
| :-------: |:-----------------:| :-------:|:-------:|:-------:|
| 2D        | 18439216171772023587 | 18439216101061917922 |10924827021061192|10924827021061192|
|           | 10927753138516356    | - |18439216260414396571||18439216260414396571|
| 2D vis    | 10927839490220270    | - |10929751161392370| - |
| 2D vis    | 10927840588229561    | - |10929751708513941| - |
| 2D vis    | 10929550337638445    | - |10929752226107993| - |
| 2D vis    | 10930003745054246    | - |10929752226107993| - |
| 4D        | 18439216260414396571    | 18439216356605802780 |10925074149542957| 10929992020276563 |
| 8D        | 18439216263452514894 | 18439218256663499061 |10929382775375323| - |
| 16D       | 10929320195794061 | 18439218256663499061 |10929382034216219| 10929994540264705 |