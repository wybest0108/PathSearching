//*********************************
//����: Wei Ying
//������ڣ�2018/9/30
//*********************************

#include "TopoMap.h"

//����·��������Ϣ
bool readAtrributeInfo(TopoMap& topoMap, const char* fileName);

//����·��������Ϣ
bool readGeometry(TopoMap& topoMap, const char* fileName);

//���·��������Ϣ��������ļ�������Global Mapper��
bool outputTopoInfo(const TopoMap& topoMap, const char* fileName, const float floorNo);
