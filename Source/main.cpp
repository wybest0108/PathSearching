#include "ioHelper.h"
#include <iostream>
#include <sstream>

using namespace std;

int main(int argc, char* argv[])
{
	char* attriFileName = "./Data/RouteAttribute.txt";
	char* geoFileName = "./Data/RouteGeometry.txt";
	TopoMap topoMapTest;

	//�����ͼ������Ϣ
	if (!readAtrributeInfo(topoMapTest, attriFileName) || !readGeometry(topoMapTest, geoFileName))
	{
		cout << "Opening input files failed!" << endl;
		return -1;
	}
	//outputTopoInfo(topoMapTest, "./Data/topoInfo_F1.xyz", 1.0f);
	topoMapTest.initTopoMap();

	//ָ����ʼ����յ�
	CVector2 startMapPt(28.2017f, 12.2388f); 
	CVector2 destMapPt(28.6680f, 10.5575f);
	float startFloorNo = 4.0f;
	float endFloorNo = 4.0f;
	if (!topoMapTest.searchPath(startMapPt, startFloorNo, destMapPt, endFloorNo))
	{
		cout << "�Ҳ�����ָ������㵽ָ���յ��·��" << endl;
		return -1;
	}
	//������
	cout << topoMapTest.getPathPointResult() << endl;


	//ָ����һ����ʼ����յ�
	cout << endl;
	startMapPt = CVector2(28.6680f, 10.5575f);
	destMapPt = CVector2(28.2525f, 10.1918f);
	startFloorNo = 4.0f;
	endFloorNo = 4.0f;
	if (!topoMapTest.searchPath(startMapPt, startFloorNo, destMapPt, endFloorNo))
	{
		cout << "�Ҳ�����ָ������㵽ָ���յ��·��" << endl;
		return -1;
	}
	//������
	cout << topoMapTest.getPathPointResult() << endl;


	//ָ����һ����ʼ����յ�
	cout << endl;
	startMapPt = CVector2(28.2017f, 12.2388f); 
	destMapPt = CVector2(25.7794f, 9.3303f);
	startFloorNo = 4.0f;
	endFloorNo = 4.0f;
	if (!topoMapTest.searchPath(startMapPt, startFloorNo, destMapPt, endFloorNo))
	{
		cout << "�Ҳ�����ָ������㵽ָ���յ��·��" << endl;
		return -1;
	}
	//������
	cout << topoMapTest.getPathPointResult() << endl;

	topoMapTest.exitAndRelease();
	system("pause");
	return 0;
}