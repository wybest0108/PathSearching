//*********************************
//作者: Wei Ying
//完成日期：2018/9/30
//*********************************

#include "TopoMap.h"

//读入路网基本信息
bool readAtrributeInfo(TopoMap& topoMap, const char* fileName);

//读入路网几何信息
bool readGeometry(TopoMap& topoMap, const char* fileName);

//输出路网拓扑信息，输出的文件可以用Global Mapper打开
bool outputTopoInfo(const TopoMap& topoMap, const char* fileName, const float floorNo);
