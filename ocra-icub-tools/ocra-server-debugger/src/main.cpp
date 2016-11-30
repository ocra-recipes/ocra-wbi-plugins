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
    retString += "<?xml version='1.0' encoding='UTF-8' ?>\n";
    retString += "<portscope rows='"+ std::to_string(nRows) +"' columns='"+ std::to_string(nCols) +"' carrier='udp'>";
    return retString;
}
std::string generateChartFromJoint(int row, int col, int i, const std::string& jointName)
{
    std::string retString("");
    retString += "<plot \
                        gridx='"+std::to_string(col)+"' \
                        gridy='"+std::to_string(row)+"' \
                        hspan='1' \
                        vspan='1' \
                        title='"+jointName+"' \
                        size='60' \
                        minval='-25' \
                        maxval='25' \
                        bgcolor='LightSlateGrey' \
                    > \
                        <graph remote='/ocra-icub-server/debug/ref:o' \
                            index='"+std::to_string(i)+"' \
                            color='Red' \
                            title='Reference' \
                            type='lines' \
                            size='3' \
                        /> \
                        <graph remote='/ocra-icub-server/debug/real:o' \
                            index='"+std::to_string(i)+"' \
                            color='Blue' \
                            title='Real' \
                            size='2' \
                            type='lines' \
                        /> \
                </plot>";
    return retString;
}
std::string generateXmlFooter()
{
    std::string retString("</portscope>");
    return retString;
}
