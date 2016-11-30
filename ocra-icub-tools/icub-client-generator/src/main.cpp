#include <iostream>
#include <algorithm>
#include <string>
#include <fstream>
#include <boost/filesystem.hpp>

std::string classNameFromClientName(const std::string& clientName);
bool checkClientNameFormat(std::string& clientName, std::string& className);
std::string getCmakeString(const std::string& clientName);
std::string getUninstallString();
std::string getMainString(const std::string& clientName, const std::string& className);
std::string getIncludeString(const std::string& clientName, const std::string& className);
std::string getSourceString(const std::string& clientName, const std::string& className);





int main(int argc, char const *argv[])
{
    std::string clientName("");
    std::string className("");

    if (argc == 2) {
        clientName = argv[1];
    } else {
        std::cout << "Please provide a client name." << std::endl;
        std::cin >> clientName;
    }

    if (!checkClientNameFormat(clientName, className)) {
        return -1;
    }
    std::cout << "Scaffolding client code for: " << clientName << std::endl;
    std::cout << "-- Client class will be named: " << className << std::endl;

    std::string rootFolderPath = "./" + clientName;
    bool overwrite = false;
    if (boost::filesystem::exists(rootFolderPath)) {
        std::cout << "[WARNING] The directory:\n\n-- " << boost::filesystem::canonical(rootFolderPath).string() << "\n\nalready exists. Are you sure you want to overwrite it? (y/n) [n]: " << std::endl;
        std::string shouldContinue;
        std::cin >> shouldContinue;
        if ( !( (shouldContinue=="y") || (shouldContinue=="Y") ) ) {
            std::cout << "Aborting." << std::endl;
            return -1;
        } else {
            boost::filesystem::remove_all(boost::filesystem::canonical(rootFolderPath));
            overwrite = true;
        }
    }

    if (!boost::filesystem::create_directory(rootFolderPath) && !overwrite) {
        std::cout << "[ERROR] Could not create directory:\n\n-- " << boost::filesystem::canonical(rootFolderPath).string() << std::endl;
        return -1;
    }

    rootFolderPath = boost::filesystem::canonical(rootFolderPath).string();

    std::string cmakeFolderPath = rootFolderPath + "/cmake";
    boost::filesystem::create_directory(cmakeFolderPath);
    cmakeFolderPath += "/Modules";
    boost::filesystem::create_directory(cmakeFolderPath);

    std::string includeFolderPath = rootFolderPath + "/include";
    boost::filesystem::create_directory(includeFolderPath);
    includeFolderPath += "/" + clientName;
    boost::filesystem::create_directory(includeFolderPath);

    std::string sourceFolderPath = rootFolderPath + "/src";
    boost::filesystem::create_directory(sourceFolderPath);

    std::string buildFolderPath = rootFolderPath + "/build";
    boost::filesystem::create_directory(buildFolderPath);


    std::string uninstallFilePath = cmakeFolderPath + "/AddUninstallTarget.cmake";
    std::ofstream uninstallFile;
    uninstallFile.open(uninstallFilePath.c_str());
    if (uninstallFile.is_open()) {
        uninstallFile << getUninstallString();
    } else {
        std::cout << "[ERROR] Could not write file:\n\n-- " << boost::filesystem::canonical(uninstallFilePath).string() << std::endl;
    }
    uninstallFile.close();

    std::string cmakeFilePath = rootFolderPath + "/CMakeLists.txt";
    std::ofstream cmakeFile;
    cmakeFile.open(cmakeFilePath.c_str());
    if (cmakeFile.is_open()) {
        cmakeFile << getCmakeString(clientName);
    } else {
        std::cout << "[ERROR] Could not write file:\n\n-- " << boost::filesystem::canonical(cmakeFilePath).string() << std::endl;
    }
    cmakeFile.close();

    std::string mainFilePath = sourceFolderPath + "/main.cpp";
    std::ofstream mainFile;
    mainFile.open(mainFilePath.c_str());
    if (mainFile.is_open()) {
        mainFile << getMainString(clientName, className);
    } else {
        std::cout << "[ERROR] Could not write file:\n\n-- " << boost::filesystem::canonical(mainFilePath).string() << std::endl;
    }
    mainFile.close();

    std::string sourceFilePath = sourceFolderPath + "/"+className+".cpp";
    std::ofstream sourceFile;
    sourceFile.open(sourceFilePath.c_str());
    if (sourceFile.is_open()) {
        sourceFile << getSourceString(clientName, className);
    } else {
        std::cout << "[ERROR] Could not write file:\n\n-- " << boost::filesystem::canonical(sourceFilePath).string() << std::endl;
    }
    sourceFile.close();

    std::string includeFilePath = includeFolderPath + "/"+className+".h";
    std::ofstream includeFile;
    includeFile.open(includeFilePath.c_str());
    if (includeFile.is_open()) {
        includeFile << getIncludeString(clientName, className);
    } else {
        std::cout << "[ERROR] Could not write file:\n\n-- " << boost::filesystem::canonical(includeFilePath).string() << std::endl;
    }
    includeFile.close();




    return 0;
}




bool checkClientNameFormat(std::string& clientName, std::string& className)
{
    // Transform to lowercase
    std::transform(clientName.begin(), clientName.end(), clientName.begin(), ::tolower);

    for (auto c : clientName)
    {
        // check for letters numbers and hyphens only
        bool test = false;
        test |= (c >= 'a') && (c <= 'z');
        test |= (c >= 'A') && (c <= 'Z');
        test |= (c >= '0') && (c <= '9');
        test |=  c == '-';
        if (!test) {
            std::cout << "ERROR: Only use letters [a-z, A-Z] numbers [0-9] and hyphens '-' in your client's name." << std::endl;
            return false;
        }
    }
    className = classNameFromClientName(clientName);
    return true;
}

std::string classNameFromClientName(const std::string& clientName)
{
    bool capitalizeNextLetter = true;
    std::string className("");

    for (auto c : clientName) {
        if(c == '-') {
            capitalizeNextLetter = true;
        } else {
            if (capitalizeNextLetter) {
                c = std::toupper(c);
                capitalizeNextLetter = false;
            }
            className += c;
        }
    }

    std::size_t found = clientName.find("client");
    if (found==std::string::npos) {
        className += "Client";
    }

    return className;
}

std::string getMainString(const std::string& clientName, const std::string& className)
{
    std::string mainString = "/*! \file       main.cpp\n *  \brief\n *  \\details\n *  \author     [Your Name](url of your github site)\n *  \\date       [date]\n *  \\copyright  GNU General Public License.\n */\n/*\n *  This file is part of "+clientName+".\n *  Copyright (C) [year] [institution]\n *\n *  This program is free software: you can redistribute it and/or modify\n *  it under the terms of the GNU General Public License as published by\n *  the Free Software Foundation, either version 3 of the License, or\n *  (at your option) any later version.\n *\n *  This program is distributed in the hope that it will be useful,\n *  but WITHOUT ANY WARRANTY; without even the implied warranty of\n *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n *  GNU General Public License for more details.\n *\n *  You should have received a copy of the GNU General Public License\n *  along with this program.  If not, see <http://www.gnu.org/licenses/>.\n*/\n\n\x23include <yarp/os/ResourceFinder.h>\n\x23include <yarp/os/Network.h>\n\x23include <yarp/os/Log.h>\n\x23include <yarp/os/LogStream.h>\n\x23include <yarp/os/Time.h>\n\n\x23include <ocra-icub/IcubClient.h>\n\x23include <ocra-recipes/ControllerClient.h>\n\x23include <ocra-recipes/ClientManager.h>\n\n\x23include \""+clientName+"/"+className+".h\"\n\nint main (int argc, char * argv[])\n{\n    yarp::os::Log yLog;\n    yarp::os::Network yarp;\n\n    double network_timeout = 10.0;\n    if (!yarp.checkNetwork(network_timeout))\n    {\n        yLog.fatal() << \"YARP network is not available\";\n        return -1;\n    }\n\n    yLog.info() << \"Making model initializer\";\n    ocra_icub::ModelInitializer modelIni = ocra_icub::ModelInitializer();\n\n    int loopPeriod = 10;\n\n    std::shared_ptr<ocra_recipes::ControllerClient> ctrlClient;\n    yLog.info() << \"Making controller client\";\n\n    if(!modelIni.getModel())\n    {\n        yLog.fatal() << \"Model is not empty.\";\n    }\n\n    ctrlClient = std::make_shared<"+className+">(modelIni.getModel(), loopPeriod);\n\n    std::shared_ptr<ocra_recipes::ClientManager> clientManager;\n    yLog.info() << \"Making client manager\";\n    clientManager = std::make_shared<ocra_recipes::ClientManager>(ctrlClient);\n\n    yLog.info() << \"Resource finder stuff\";\n    yarp::os::ResourceFinder rf;\n    rf.setVerbose(true);\n    rf.setDefaultConfigFile(\""+clientName+".ini\"); //default config file name.\n    rf.setDefaultContext(\""+clientName+"\"); //when no parameters are given to the module this is the default context\n    rf.configure(argc,argv);\n\n    if (rf.check(\"help\"))\n    {\n        clientManager->printHelp();\n        return 0;\n    }\n\n    yLog.info() << \"Configuring\";\n    clientManager->configure(rf);\n\n    yLog.info() << \"Launching client\";\n    return clientManager->launchClient();\n}\n";
    return mainString;
}

std::string getIncludeString(const std::string& clientName, const std::string& className)
{
    std::string defString = className;
    std::transform(defString.begin(), defString.end(), defString.begin(), ::toupper);
    std::string includeString = "\43ifndef "+defString+"_H\n\43define "+defString+"_H\n\n\43include <ocra-icub/IcubClient.h>\n\43include <ocra-recipes/TrajectoryThread.h>\n\43include <ocra-recipes/ControllerClient.h>\n\n\nclass "+className+" : public ocra_recipes::ControllerClient\n{\nDEFINE_CLASS_POINTER_TYPEDEFS("+className+")\n\npublic:\n    "+className+" (std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod);\n    virtual ~"+className+" ();\n\nprotected:\n    virtual bool initialize();\n    virtual void release();\n    virtual void loop();\n\nprivate:\n\n};\n\n\n\43endif // "+defString+"_H\n";
    return includeString;
}

std::string getSourceString(const std::string& clientName, const std::string& className)
{
    std::string sourceString = "\x23include \""+clientName+"/"+className+".h\"\n"+className+"::"+className+"(std::shared_ptr<ocra::Model> modelPtr, const int loopPeriod)\n: ocra_recipes::ControllerClient(modelPtr, loopPeriod)\n{\n    // add your code here...\n}\n\n"+className+"::~"+className+"()\n{\n    // add your code here...\n}\n\nbool "+className+"::initialize()\n{\n    // add your code here...\n    return true;\n}\n\nvoid "+className+"::release()\n{\n    // add your code here...\n}\n\nvoid "+className+"::loop()\n{\n    // add your code here...\n}\n";
    return sourceString;
}

std::string getCmakeString(const std::string& clientName)
{
    std::string cmakeString = "\x23 This file is part of "+clientName+".\n\x23 Copyright (C) [your institution here]\n\x23 author(s): [your name here]\n\x23\n\x23 This program is free software: you can redistribute it and/or modify\n\x23 it under the terms of the GNU General Public License as published by\n\x23 the Free Software Foundation, either version 3 of the License, or\n\x23 (at your option) any later version.\n\x23\n\x23 This program is distributed in the hope that it will be useful,\n\x23 but WITHOUT ANY WARRANTY; without even the implied warranty of\n\x23 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n\x23 GNU General Public License for more details.\n\x23\n\x23 You should have received a copy of the GNU General Public License\n\x23 along with this program.  If not, see <http://www.gnu.org/licenses/>.\n\n\x23 Make sure we are working with at least CMake 2.8.12\ncmake_minimum_required(VERSION 2.8.12)\n\n\x23 Initiate the project\nPROJECT("+clientName+" CXX)\n\n\x23 Make sure you have a C++11 compatible compiler\ninclude(CheckCXXCompilerFlag)\nCHECK_CXX_COMPILER_FLAG(\"-std=c++11\" COMPILER_SUPPORTS_CXX11)\nif(COMPILER_SUPPORTS_CXX11)\n    set(CMAKE_CXX_FLAGS \"\x24{CMAKE_CXX_FLAGS} -std=c++11\")\nelse()\n    message(FATAL_ERROR \"The compiler \x24{CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.\")\n endif() \n \n \x23 Build as Release (Change Release to Debug for better debugging symbols)\nset(CMAKE_BUILD_TYPE Release)\n\n\x23 Set the project version.\nset(\x24{PROJECT_NAME}_MAJOR_VERSION 1)\nset(\x24{PROJECT_NAME}_MINOR_VERSION 0)\nset(\x24{PROJECT_NAME}_PATCH_VERSION 0)\nset(\x24{PROJECT_NAME}_VERSION \x24{\x24{PROJECT_NAME}_MAJOR_VERSION}.\x24{\x24{PROJECT_NAME}_MINOR_VERSION}.\x24{\x24{PROJECT_NAME}_PATCH_VERSION})\n\n\x23 Add some helpful CMake functions\nlist(APPEND CMAKE_MODULE_PATH \x24{CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)\n\n\x23 Find OcraIcub\nfind_package(OcraIcub REQUIRED)\nIF(\x24{OcraIcub_FOUND})\n    message(\"-- Found OcraIcub version \x24{OcraIcub_VERSION}\")\nENDIF()\nset(CMAKE_EXPORT_COMPILE_COMMANDS ON)\n\x23 Get all of the source and header files.\nfile(GLOB folder_source src/*.cpp)\nfile(GLOB folder_header include/\x24{PROJECT_NAME}/*.h)\nsource_group(\"Source Files\" FILES \x24{folder_source})\nsource_group(\"Header Files\" FILES \x24{folder_header})\n\n\x23 Tell the compiler where to look for all other headers\ninclude_directories(\n\x24{PROJECT_SOURCE_DIR}/include\n\x24{OcraIcub_INCLUDE_DIRS}\n)\n\n\x23 Add the client executable (binary)\nadd_executable(\x24{PROJECT_NAME} \x24{folder_source} \x24{folder_header})\n\n\x23 Link to the appropriate libs\ntarget_link_libraries(\n\x24{PROJECT_NAME}\n\x24{OcraIcub_LIBRARIES}\n)\n\n\x23 Install to the bin/ directory if installed.\ninstall(TARGETS \x24{PROJECT_NAME} DESTINATION bin)\n\n\x23 Add an uninstallation target so you can just run - make uninstall - to remove the binary.\ninclude(AddUninstallTarget)\n";
    return cmakeString;
}

std::string getUninstallString()
{
    std::string retString = "\x23.rst:\n\x23 AddUninstallTarget\n\x23 ------------------\n\x23\n\x23 Add the \"uninstall\" target for your project::\n\x23\n\x23   include(AddUninstallTarget)\n\x23\n\x23\n\x23 will create a file cmake_uninstall.cmake in the build directory and add a\n\x23 custom target uninstall that will remove the files installed by your package\n\x23 (using install_manifest.txt)\n\n\x23=============================================================================\n\x23 Copyright 2008-2013 Kitware, Inc.\n\x23 Copyright 2013 iCub Facility, Istituto Italiano di Tecnologia\n\x23   Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>\n\x23\n\x23 Distributed under the OSI-approved BSD License (the \"License\");\n\x23 see accompanying file Copyright.txt for details.\n\x23\n\x23 This software is distributed WITHOUT ANY WARRANTY; without even the\n\x23 implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n\x23 See the License for more information.\n\x23=============================================================================\n\x23 (To distribute this file outside of CMake, substitute the full\n\x23  License text for the above reference.)\n\n\nif(DEFINED __ADD_UNINSTALL_TARGET_INCLUDED)\n  return()\nendif()\nset(__ADD_UNINSTALL_TARGET_INCLUDED TRUE)\n\n\nset(_filename \x24{CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)\n\nfile(WRITE \x24{_filename}\n\"if(NOT EXISTS \\\"\x24{CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt\\\")\n    message(WARNING \\\"Cannot find install manifest: \\\\\\\"\x24{CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt\\\\\\\"\\\")\n    return()\nendif()\n\nfile(READ \\\"\x24{CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt\\\" files)\nstring(STRIP \\\"\\\x24{files}\\\" files)\nstring(REGEX REPLACE \\\"\\\\n\\\" \\\";\\\" files \\\"\\\x24{files}\\\")\nlist(REVERSE files)\nforeach(file \\\x24{files})\n    message(STATUS \\\"Uninstalling: \\\44ENV{DESTDIR}\\\x24{file}\\\")\n    if(EXISTS \\\"\\\44ENV{DESTDIR}\\\x24{file}\\\")\n        execute_process(\n            COMMAND \\\x24{CMAKE_COMMAND} -E remove \\\"\\\44ENV{DESTDIR}\\\x24{file}\\\"\n            OUTPUT_VARIABLE rm_out\n            RESULT_VARIABLE rm_retval)\n        if(NOT \\\"\\\x24{rm_retval}\\\" EQUAL 0)\n            message(FATAL_ERROR \\\"Problem when removing \\\\\\\"\\\44ENV{DESTDIR}\\\x24{file}\\\\\\\"\\\")\n        endif()\n    else()\n        message(STATUS \\\"File \\\\\\\"\\\44ENV{DESTDIR}\\\x24{file}\\\\\\\" does not exist.\\\")\n    endif()\nendforeach(file)\n\")\n\nif(\"\x24{CMAKE_GENERATOR}\" MATCHES \"^(Visual Studio|Xcode)\")\n    set(_uninstall \"UNINSTALL\")\nelse()\n    set(_uninstall \"uninstall\")\nendif()\nadd_custom_target(\x24{_uninstall} COMMAND \"\x24{CMAKE_COMMAND}\" -P \"\x24{_filename}\")\nset_property(TARGET \x24{_uninstall} PROPERTY FOLDER \"CMakePredefinedTargets\")";
    return retString;
}
