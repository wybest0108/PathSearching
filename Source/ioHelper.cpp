//*********************************
//作者: Wei Ying
//完成日期：2018/9/30
//*********************************

#include <fstream>
#include "ioHelper.h"
#include <string>
#include <iostream>

using namespace std;
//读入路网基本信息
bool readAtrributeInfo(TopoMap& topoMap, const char* fileName)
{
	topoMap.m_currMaxUsedID = 0;
	topoMap.m_originalMaxLineID = 0;
	int id, aNodeID, bNodeID;
	int type, dir, right;
	float floorNo, grade, gridW;
	int starttime, endtime;
	map<ID, vector<ID>> tempMapForAttachNodeIDs;	//用于暂时存储与面状对象连接的节点ID

	ifstream fin(fileName);
	if (!fin.is_open())
	{
		cout << "Opening atrribute file failed!" << endl;
		return false;
	}

	fin.ignore(255, '\n');
	while (!fin.eof())
	{
		fin >> type;        fin.ignore(1);
		fin >> id;          fin.ignore(1);
		fin >> dir;			fin.ignore(1);
		fin >> aNodeID;		fin.ignore(1);
		fin >> bNodeID;		fin.ignore(1);
		fin >> floorNo;		fin.ignore(1);
		fin >> right;		fin.ignore(1);			//暂时没有用到，忽略掉
		fin >> grade;		fin.ignore(1);
		fin >> starttime;	fin.ignore(1);			//暂时没有用到，忽略掉
		fin >> endtime;		fin.ignore(1);			//暂时没有用到，忽略掉
		fin >> gridW;		fin.ignore(1);

		if (fin.fail())	break;

		topoMap.m_currMaxUsedID = id > topoMap.m_currMaxUsedID ? id : topoMap.m_currMaxUsedID;

		switch (type)
		{
		case 0:
		{
				  pLine pNewLine = new Line(id, floorNo, grade, aNodeID, bNodeID, (Direction)dir);
				  topoMap.m_lines.insert(pair<ID, pLine>(id, pNewLine));
				  topoMap.m_originalMaxLineID = id > topoMap.m_originalMaxLineID ? id : topoMap.m_originalMaxLineID;
				  break;
		}

		case 1:
		{
				  pArea pNewArea = new Area(id, floorNo, grade, gridW);
				  topoMap.m_areas.insert(pair<ID, pArea>(id, pNewArea));
				  break;
		}

		case 2:
		{
				  pNode pNewNode = new Node(id, floorNo, grade);
				  if (0 != aNodeID)
				  {
					  tempMapForAttachNodeIDs[aNodeID].push_back(id);
				  }
				  topoMap.m_nodes.insert(pair<ID, pNode>(id, pNewNode));
				  break;
		}

		default:
			break;
		}
	}
	fin.close();

	map<ID, vector<ID>>::iterator iter = tempMapForAttachNodeIDs.begin();
	while (iter != tempMapForAttachNodeIDs.end())
	{
		ID id = iter->first;
		map<ID, pArea>::iterator iterArea = topoMap.m_areas.find(id);
		if (iterArea != topoMap.m_areas.end())
		{
			pArea pArea = iterArea->second;
			pArea->m_attachNodeIDs.assign(iter->second.begin(), iter->second.end());
		}
		++iter;
	}

	return true;
}


//读入路网几何信息
bool readGeometry(TopoMap& topoMap, const char* fileName)
{
	ID id;
	int polygonCount, polygonStartIndex, pointCount;
	vector<int> polygonStartIndexVector;
	vector<CVector3> v3Points;

	ifstream fin(fileName);
	if (!fin.is_open())
	{
		cout << "Opening geometry file failed!" << endl;
		return false;
	}

	fin.ignore(255, '\n');
	while (!fin.eof())
	{
		polygonStartIndexVector.clear();
		v3Points.clear();

		fin >> id;				fin.ignore(1);
		fin >> polygonCount;	fin.ignore(1);

		if (fin.fail())		break;

		if (0 != polygonCount)
		{
			for (int i = 0; i < polygonCount; ++i)
			{
				fin >> polygonStartIndex;	fin.ignore(1);
				polygonStartIndexVector.push_back(polygonStartIndex);
			}
		}

		fin >> pointCount;	fin.ignore(1);
		for (int i = 0; i < pointCount; ++i)
		{
			CVector3 cv3;
			fin >> cv3.x;	fin.ignore(1);
			fin >> cv3.y;	fin.ignore(1);
			fin >> cv3.z;	fin.ignore(1);
			v3Points.push_back(cv3);
		}

		map<ID, pNode>::iterator iterNode = topoMap.m_nodes.find(id);
		if (iterNode != topoMap.m_nodes.end())
		{
			//为节点类型
			iterNode->second->m_location = CVector3(v3Points[0].x, v3Points[0].y, v3Points[0].z);
		}
		else
		{
			map<ID, pLine>::iterator iterLine = topoMap.m_lines.find(id);
			if (iterLine != topoMap.m_lines.end())
			{
				//为线类型
				pLine pLine = iterLine->second;
				pLine->m_points.assign(v3Points.begin(), v3Points.end());
				pLine->calculateLineLength();
			}
			else
			{
				map<ID, pArea>::iterator iterArea = topoMap.m_areas.find(id);
				if (iterArea != topoMap.m_areas.end())
				{
					//为面类型
					pArea pArea = iterArea->second;
					pGeometry pGeo = &pArea->m_geo;
					pGeo->m_points.assign(v3Points.begin(), v3Points.end());
					pGeo->m_pointCount = pGeo->m_points.size();

					if (0 == polygonCount)
					{
						pGeo->m_polygonCount = 1;
						pGeo->m_polygonStartIndex.push_back(0);
					}
					else
					{
						pGeo->m_polygonStartIndex.assign(polygonStartIndexVector.begin(), polygonStartIndexVector.end());
						pGeo->m_polygonCount = pGeo->m_polygonStartIndex.size();
					}
					pArea->calculateClipBox();
				}
			}
		}
	}
	fin.close();
	return true;
}


//输出路网拓扑信息，输出的文件可以用Global Mapper打开
bool outputTopoInfo(const TopoMap& topoMap, const char* fileName, const float floorNo)
{
	ofstream fout(fileName);
	if (!fout.is_open())
	{
		cout << "Opening output file failed!" << endl;
		return false;
	}

	//输出线对象
	map<ID, pLine>::const_iterator iterLine;
	for (iterLine = topoMap.m_lines.begin(); iterLine != topoMap.m_lines.end(); ++iterLine)
	{
		pLine pCurrLine = iterLine->second;
		if (fabs(pCurrLine->m_floorNo - floorNo) > 0.01)		continue;		//不属于指定楼层

		fout << "DESCRIPTION=Unknown Line Type" << endl;
		fout << "NAME=" << pCurrLine->m_id << endl;

		for (size_t i = 0, lineCount = pCurrLine->m_points.size(); i < lineCount; ++i)
		{
			fout << pCurrLine->m_points[i].x << "," << pCurrLine->m_points[i].y << "," << pCurrLine->m_points[i].z << endl;
		}
		fout << endl;
	}

	//输出面对象
	map<ID, pArea>::const_iterator iterArea;
	for (iterArea = topoMap.m_areas.begin(); iterArea != topoMap.m_areas.end(); ++iterArea)
	{
		pArea pCurrArea = iterArea->second;
		if (fabs(pCurrArea->m_floorNo - floorNo) > 0.01)		continue;		//不属于指定楼层

		pGeometry pGeo = &(pCurrArea->m_geo);
		for (int polyIndex = 0; polyIndex < pGeo->m_polygonCount; ++polyIndex)
		{
			if (0 == polyIndex)
			{
				fout << "DESCRIPTION = Unknown Area Type" << endl;
				fout << "NAME=" << pCurrArea->m_id << endl;
				fout << "CLOSED=YES" << endl;
			}
			else
			{
				fout << "DESCRIPTION = Unknown Area Type" << endl;
				fout << "CLOSED=YES" << endl;
				fout << "ISLAND=YES" << endl;
			}
			int startPtIndex = pGeo->m_polygonStartIndex[polyIndex];
			int endPtIndex = (polyIndex == pGeo->m_polygonCount - 1) ? pGeo->m_pointCount - 1 : pGeo->m_polygonStartIndex[polyIndex + 1] - 1;
			for (int j = startPtIndex; j <= endPtIndex; ++j)
			{
				fout << pGeo->m_points[j].x << "," << pGeo->m_points[j].y << "," << pGeo->m_points[j].z << endl;
			}
			fout << endl;
		}
	}

	//输出节点对象
	map<ID, pNode>::const_iterator iterNode;
	for (iterNode = topoMap.m_nodes.begin(); iterNode != topoMap.m_nodes.end(); ++iterNode)
	{
		pNode pNode = iterNode->second;
		if (fabs(pNode->m_floorNo - floorNo) > 0.01)		continue;		//不属于指定楼层

		fout << "DESCRIPTION=Unknown Node Type" << endl;
		fout << "NAME=" << pNode->m_id << endl;
		fout << pNode->m_location.x << "," << pNode->m_location.y << "," << pNode->m_location.z << endl << endl;
	}

	fout.close();
	return true;
}



