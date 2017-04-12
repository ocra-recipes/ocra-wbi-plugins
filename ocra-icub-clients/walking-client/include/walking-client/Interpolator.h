#include <Eigen/Dense>
#include <iostream>
#include <vector>

template <class indepVarType, class depVarType>
void linearInterpolation(const indepVarType& xStart, const indepVarType& xEnd, const depVarType& yStart, const depVarType& yEnd, const indepVarType& dx, std::vector<indepVarType>& xInterp, std::vector<depVarType>& yInterp, bool overwrite=true, bool ignoreStart=false)
{
    if (overwrite)
    {
        xInterp.clear();
        yInterp.clear();
    }

    indepVarType xRange = xEnd - xStart;
    depVarType yRange = yEnd - yStart;
    indepVarType x;
    x = xStart;
    if (ignoreStart)
    {
        x+=dx;
    }
    // Add a small margin for roundoff errors in a double comparison.
    while(x <= xEnd+(dx*0.001))
    {
        xInterp.push_back(x);
        yInterp.push_back(((x - xStart)/(xRange))*(yRange) + yStart);
        x += dx;
    }
}

template <class indepVarType, class depVarType>
void linearInterpolation(const std::vector<indepVarType>& xVec, const std::vector<depVarType>& yVec, const indepVarType& dx, std::vector<indepVarType>& xInterp, std::vector<depVarType>& yInterp)
{
    assert(xVec.size()>0);
    assert(yVec.size()>0);
    assert(xVec.size()==yVec.size());

    xInterp.clear();
    yInterp.clear();

    xInterp.push_back(xVec[0]);
    yInterp.push_back(yVec[0]);

    for (int i=0; i<xVec.size()-1; ++i)
    {
        linearInterpolation(xVec[i], xVec[i+1], yVec[i], yVec[i+1], dx, xInterp, yInterp, false, true);
    }
}
