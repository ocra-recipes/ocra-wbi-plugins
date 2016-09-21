#include <cstdlib>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "yarp/os/RpcClient.h"
#include "yarp/os/Port.h"
#include "yarp/os/Bottle.h"
#include "yarp/os/Network.h"

std::string generateXmlHeader(int rows, int cols);
std::string generateChartFromJoint(int row, int col, int i, const std::string& jointName);
std::string generateXmlFooter();


int main(int argc, char const *argv[]) {

    yarp::os::Network net;

    yarp::os::RpcClient clientPort;
    std::string clientPortName("/ocra-server-debugger/debug/rpc:o");
    std::string serverPortName("/ocra-icub-server/debug/rpc:i");
    clientPort.open(clientPortName);
    net.connect(clientPortName.c_str(), serverPortName.c_str());

    yarp::os::Bottle request, reply;

    request.addString("listJoints");
    clientPort.write(request, reply);
    int nDoF = reply.size()-1;
    if (nDoF != 0) {
        int nRows, nCols;

        nRows = (int)sqrt(nDoF);
        nCols = nRows;

        while (nCols * nRows < nDoF) {
            ++nCols;
        }

        std::string xmlString = generateXmlHeader(nRows, nCols);
        int row = 0;
        int col = 0;
        for (int i=1; i<nDoF+1; ++i) {
            std::string jointName = reply.get(i).asString();
            xmlString += generateChartFromJoint(row, col, i-1, jointName);
            ++col;
            if (col==nCols) {
                ++row;
                col = 0;
            }
        }
        xmlString += generateXmlFooter();

        std::string xmlFilePathRel = "./ocra-server-debugger_tmp_yarpscope_config.xml";

        std::ofstream xmlFile;
        xmlFile.open(xmlFilePathRel.c_str());
        if (xmlFile.is_open()) {
            xmlFile << xmlString;
        } else {
            std::cout << "[ERROR] Could not write file:\n\n-- " << boost::filesystem::canonical(xmlFilePathRel).string() << std::endl;
        }
        xmlFile.close();

        std::string cmd = "yarpscope --xml " + boost::filesystem::canonical(xmlFilePathRel).string();
        std::system(cmd.c_str());

        boost::filesystem::remove(boost::filesystem::canonical(xmlFilePathRel));
        clientPort.close();
        return 0;
    } else {
        std::cout << "ERROR" << std::endl;
        return -1;
    }

}


std::string generateXmlHeader(int nRows, int nCols)
{
    std::string retString("");
    retString += "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n";
    retString += "<portscope rows=\""+ std::to_string(nRows) +"\" columns=\""+ std::to_string(nCols) +"\" carrier=\"udp\">";
    return retString;
}
std::string generateChartFromJoint(int row, int col, int i, const std::string& jointName)
{
    std::string retString("");
    retString += "<plot\ngridx=\""+std::to_string(col)+"\"\ngridy=\""+std::to_string(row)+"\"\nhspan=\"1\"\nvspan=\"1\"\ntitle=\""+jointName+"\"\nsize=\"60\"\nminval=\"-25\"\nmaxval=\"25\"\nbgcolor=\"LightSlateGrey\">\n<graph remote=\"/ocra-icub-server/debug/ref:o\"\nindex=\""+std::to_string(i)+"\"\ncolor=\"Red\"\ntitle=\"Reference\"\ntype=\"lines\"\nsize=\"3\" />\n<graph remote=\"/ocra-icub-server/debug/real:o\"\nindex=\""+std::to_string(i)+"\"\ncolor=\"Blue\"\ntitle=\"Real\"\nsize=\"2\"\ntype=\"lines\" />\n</plot>";
    return retString;
}
std::string generateXmlFooter()
{
    std::string retString("</portscope>");
    return retString;
}
