//*********************************************************************************************
//作者:	Wei Ying
//完成日期：2018/9/30
//备注：本室内导航算法基于A*算法，算法参考来源：https://www.redblobgames.com/pathfinding/a-star/
//*********************************************************************************************

#include "TopoMap.h"
#include <float.h>
#include <iostream>
#include <sstream>
#include <time.h>		//for test

using namespace std;
//Node对象带参构造函数
Node::Node(ID id, float floorNo, float grade, const CVector3& cv3) : BaseElement(id, floorNo, grade)
{
	m_location = cv3;
}


//计算Line对象的总长度
void Line::calculateLineLength()
{
	m_totalLength = 0.0f;
	for (size_t i = 1; i < m_points.size(); ++i)
	{
		m_totalLength += distBetweenTwoPoints(m_points[i], m_points[i - 1]);
	}
}


//判断点是否在区域的单个多边形内(算法来源于:http://angusj.com/delphi/clipper.php)
int Geometry::_isPointInPolygon(const CVector2& pt, const int startIndex, const int endIndex)
{
	//rusult取值： 0：不在多边形内；1：在多边形内；-1：在多边形边界上
	int result = 0;
	int count = endIndex - startIndex + 1;
	if (count < 3)	return 0;

	pCVector3 pCurrPt = &m_points[startIndex];
	for (int i = startIndex + 1; i <= (endIndex + 1); ++i)
	{
		pCVector3 pNextPt = (i == endIndex + 1) ? &m_points[startIndex] : &m_points[i];
		if (fabs(pNextPt->y - pt.y) < PRECISION)
		{
			if (fabs(pNextPt->x - pt.x) < PRECISION || (fabs(pCurrPt->y - pt.y) < PRECISION && ((pNextPt->x > pt.x) == (pCurrPt->x < pt.x))))
			{
				return -1;
			}
		}

		if ((pCurrPt->y < pt.y) != (pNextPt->y < pt.y))
		{
			if (pCurrPt->x >= pt.x)
			{
				if (pNextPt->x > pt.x)
				{
					result = 1 - result;
				}
				else
				{
					double d = (double)(pCurrPt->x - pt.x) * (pNextPt->y - pt.y) - (double)(pNextPt->x - pt.x) * (pCurrPt->y - pt.y);
					if (!d) return -1;
					if ((d > 0) == (pNextPt->y > pCurrPt->y)) result = 1 - result;
				}
			}
			else
			{
				if (pNextPt->x > pt.x)
				{
					double d = (double)(pCurrPt->x - pt.x) * (pNextPt->y - pt.y) - (double)(pNextPt->x - pt.x) * (pCurrPt->y - pt.y);
					if (!d) return -1;
					if ((d > 0) == (pNextPt->y > pCurrPt->y)) result = 1 - result;
				}
			}
		}

		pCurrPt = pNextPt;
	}
	return result;
}


//判断点是否在区域内，返回值为 0： 不在区域内； 1： 在区域可通性区内； -1：在区域的不可通行区内 
int Geometry::isPointInsideGeometry(const CVector2& point)
{
	int count = 0;
	for (int polyIndex = 0; polyIndex < m_polygonCount; ++polyIndex)
	{
		int startPtIndex = m_polygonStartIndex[polyIndex];
		int endPtIndex = (polyIndex == m_polygonCount - 1) ? m_pointCount - 1 : m_polygonStartIndex[polyIndex + 1] - 1;
		int result = _isPointInPolygon(point, startPtIndex, endPtIndex);
		if (1 == result)	++count;
		//if (count >= 2)		break;	//这句代码假设一个区域内只存在最多两个多边形嵌套，不考虑大于两个多边形嵌套的复杂区域
	}

	if (0 == count)	return 0;
	return count & 1 ? 1 : -1;		//count如果是奇数，则在区域可通性区内；如果是偶数，则在区域的不可通行区内 
}


//计算区域包围盒
bool Geometry::calculateClipBox(CVector3& clipMin, CVector3& clipMax)
{
	if (m_pointCount <= 0)
	{
		return false;
	}

	clipMin = m_points[0];
	clipMax = m_points[0];

	for (int i = 1; i < m_pointCount; ++i)
	{
		clipMin.x = clipMin.x > m_points[i].x ? m_points[i].x : clipMin.x;
		clipMin.y = clipMin.y > m_points[i].y ? m_points[i].y : clipMin.y;
		clipMin.z = clipMin.z > m_points[i].z ? m_points[i].z : clipMin.z;

		clipMax.x = clipMax.x < m_points[i].x ? m_points[i].x : clipMax.x;
		clipMax.y = clipMax.y < m_points[i].y ? m_points[i].y : clipMax.y;
		clipMax.z = clipMax.z < m_points[i].z ? m_points[i].z : clipMax.z;
	}

	return true;
}



//计算区域包围盒
bool Area::calculateClipBox()
{
	return m_geo.calculateClipBox(m_clipMin, m_clipMax);
}


//栅格化区域
bool Area::createGrid()
{
	if (m_gridWidth < MINGRIDWIDTH)			//网格太小，会导致内存过大，所以要做限制
	{
		return false;
	}

	m_grid.m_width = (int)((m_clipMax.x - m_clipMin.x) / m_gridWidth) + 1;
	m_grid.m_height = (int)((m_clipMax.y - m_clipMin.y) / m_gridWidth) + 1;
	m_grid.m_gridSize = (m_grid.m_width + 7) / 8 * m_grid.m_height;
	m_grid.m_passageInfo = new unsigned char[m_grid.m_gridSize];		//思考：当前的方式与只存储不可通行的网格点索引，哪个更节省内存呢？
	memset(m_grid.m_passageInfo, 0, m_grid.m_gridSize);					//初始设为可通行

	//计算每个单元格是否可通行
	for (int i = 0; i < m_grid.m_width; ++i)
	{
		//cout << endl;			//test
		for (int j = 0; j < m_grid.m_height; ++j)
		{
			float x = m_clipMin.x + (i + 0.5f) * m_gridWidth;
			float y = m_clipMin.y + (j + 0.5f) * m_gridWidth;
			CVector2 currMapPt(x, y);
			bool isInsideGeo = (1 == m_geo.isPointInsideGeometry(currMapPt));
			if (isInsideGeo)
			{
				//每个单元格的可通性用一个位存储，那么每一行的八个单元格就组成一个字节
				int sbit = i % 8;
				int byteNumPerRow = (m_grid.m_width + 7) / 8;			//每行的字节数
				unsigned char byteValue = 1;
				byteValue = byteValue << sbit;
				((unsigned char *)m_grid.m_passageInfo)[j * byteNumPerRow + i / 8] |= byteValue;
			}
			//int result = isInsideGeo ? 1 : 0;			//test
			//cout << result << " ";					//test
		}
	}
	return true;
}


//将栅格点位坐标转成实际位置坐标
bool Area::gridToMap(const GridLocation& gridPt, CVector2 &mapPt)
{
	if (gridPt.first >= m_grid.m_width || gridPt.second >= m_grid.m_height || gridPt.first < 0 || gridPt.second < 0)
	{
		return false;
	}

	mapPt.x = m_clipMin.x + (gridPt.first + 0.5f) * m_gridWidth;		//这里取的是单元格中心点
	mapPt.y = m_clipMin.y + (gridPt.second + 0.5f) * m_gridWidth;		//这里取的是单元格中心点

	return true;
}


//实际点位坐标转换成栅格点位坐标
bool Area::mapToGrid(const CVector2& mapPt, GridLocation& gridPt)
{
	if (mapPt.x < m_clipMin.x || mapPt.x > m_clipMax.x || mapPt.y < m_clipMin.y || mapPt.y > m_clipMax.y)
	{
		return false;
	}

	gridPt.first = (int)((mapPt.x - m_clipMin.x) / (m_clipMax.x - m_clipMin.x) * m_grid.m_width);
	gridPt.first = gridPt.first >= m_grid.m_width ? (m_grid.m_width - 1) : gridPt.first;

	gridPt.second = (int)((mapPt.y - m_clipMin.y) / (m_clipMax.y - m_clipMin.y) * m_grid.m_height);
	gridPt.second = gridPt.second >= m_grid.m_height ? (m_grid.m_height - 1) : gridPt.second;
	return true;
}


//在地图区域内查找从起点到终点的通行路径,没有找到则返回false,否则返回true
bool Area::openSpaceSearchOnMap(const CVector3& startMapPt, const CVector3& destMapPt, vector<CVector3>& pathOnMap)
{
	if (startMapPt == destMapPt)	return false;

	GridLocation startGridPt;
	GridLocation destGridPt;
	mapToGrid(CVector2(startMapPt.x, startMapPt.y), startGridPt);
	mapToGrid(CVector2(destMapPt.x, destMapPt.y), destGridPt);
	if (startGridPt == destGridPt)
	{
		pathOnMap.push_back(startMapPt);
		pathOnMap.push_back(destMapPt);
		return true;
	}

	vector<GridLocation> pathOnGrid;
	if (!m_grid.openSpaceSearchOnGrid(startGridPt, destGridPt, pathOnGrid))
	{
		return false;
	}

	float zCoordOfArea = m_geo.m_points[0].z;
	//因为openSpaceSearchOnGrid函数里可能对起点和终点做了些稍稍的偏移，所以下面需要用原始的值修正回来
	pathOnMap.push_back(startMapPt);
	for (size_t i = 1; i < pathOnGrid.size() - 1; ++i)
	{
		CVector2 mapPt;
		gridToMap(pathOnGrid[i], mapPt);
		pathOnMap.push_back(CVector3(mapPt.x, mapPt.y, zCoordOfArea));
	}
	pathOnMap.push_back(destMapPt);
	return true;
}

GridLocation SquareGrid::m_DIRS[8] = { GridLocation(1, 0), GridLocation(0, -1), GridLocation(-1, 0), GridLocation(0, 1), GridLocation(1, 1), GridLocation(1, -1), GridLocation(-1, -1), GridLocation(-1, 1) };


SquareGrid::~SquareGrid()
{
	delete m_passageInfo;
	m_passageInfo = NULL;
}


//判断点是否在网格区域内
bool SquareGrid::isInBounds(const GridLocation& gridPt)
{
	return gridPt.first >= 0 && gridPt.first < m_width && gridPt.second >= 0 && gridPt.second < m_height;
}


//返回单元格是否可通行
bool SquareGrid::isPassable(const GridLocation& gridPt)
{
	int sbit = gridPt.first % 8;
	int byteNumPerRow = (m_width + 7) / 8;
	unsigned char byteValue = m_passageInfo[gridPt.second * byteNumPerRow + gridPt.first / 8];

	return (((byteValue >> sbit) & 0x01) == 0x01);
}


//靠近边界的可通行的实际点，栅格化后，可能会划分到不可通行的单元格上，此时需要寻找离它最近的可通性单元格来代替
bool SquareGrid::_adjustGridLocation(GridLocation& gridPt)
{
	if (isPassable(gridPt))	return true;

	GridLocation tempPt;
	int boundLf, boundRt, boundDn, boundUp;
	int prevLf = gridPt.first;
	int prevRt = gridPt.first;
	int prevDn = gridPt.second;
	int prevUp = gridPt.second;
	for (int radius = 1; radius < 8; ++radius)			//此处8为最大寻找半径。问题：取8能保证一定能查找到吗？这种方法查找出来的并不一定是最近的
	{
		boundLf = gridPt.first - radius;
		boundRt = gridPt.first + radius;
		boundDn = gridPt.second - radius;
		boundUp = gridPt.second + radius;

		if (boundLf < 0)	boundLf = 0;
		if (boundDn < 0)	boundDn = 0;
		if (boundRt >= m_width)		boundRt = m_width - 1;
		if (boundUp >= m_height)	boundUp = m_height - 1;

		for (int i = boundLf; i <= boundRt; ++i)
		{
			for (int j = boundDn; j <= boundUp; ++j)
			{
				if (i >= prevLf && i <= prevRt && j >= prevDn && j <= prevUp)	continue;
				tempPt.first = i;
				tempPt.second = j;
				if (isPassable(tempPt))
				{
					gridPt = tempPt;
					return true;
				}
			}
		}
		prevLf = boundLf;
		prevRt = boundRt;
		prevDn = boundDn;
		prevUp = boundUp;
	}
	return false;
}


//查找网格区域内从起点到终点的通行路径
bool SquareGrid::openSpaceSearchOnGrid(GridLocation& start, GridLocation& dest, vector<GridLocation>& path)
{
	//起点和终点总是在边界上，所以栅格化后，起点或终点可能位于不可通行的网格上，此时需要对起点或终点做些修正，使其所在单元格可通行
	//做法：寻找与起点或终点距离最近的可通性的单元格来代替
	_adjustGridLocation(start);
	_adjustGridLocation(dest);

	if (start == dest)
	{
		path.push_back(start);
		path.push_back(dest);
		return true;
	}

	map<GridLocation, GridLocation>  cameFrom;
	map<GridLocation, float> costSoFar;
	AStarSearchOnGrid(start, dest, cameFrom, costSoFar);
	if (!constructPath(start, dest, cameFrom, path))
	{
		return false;
	}
	refinePath(path);
	return true;
}


//基于网格的A*搜索算法
void SquareGrid::AStarSearchOnGrid(const GridLocation& start, const GridLocation& dest, map<GridLocation, GridLocation>& cameFrom, map<GridLocation, float>& costSoFar)
{
	PriorityQueue<GridLocation, float> frontier;
	frontier.put(start, 0.0f);

	cameFrom[start] = start;
	costSoFar[start] = 0.0f;

	while (!frontier.empty())
	{
		//当有序集不为空的时候，取有序集第一个
		GridLocation current = frontier.get();

		if (current == dest)	break;		//找到目标就停止

		vector<GridLocation> nextSet;
		getNeighbors(current, nextSet);

		for (size_t i = 0; i < nextSet.size(); ++i)
		{
			GridLocation next = nextSet[i];
			//邻居点的cost等于当前点的cost加上位移cost
			float newCost = costSoFar[current] + cost(current, next);
			if (!costSoFar.count(next) || newCost < costSoFar[next])
			{
				//如果next点的cost并不存在或者新得到的cost小于next的cost,则更新next的cost，将next和对应的来源点加入cameFrom中，将next和f值加入有序集frontier中
				costSoFar[next] = newCost;
				frontier.put(next, heuristic(next, dest) + newCost);
				cameFrom[next] = current;
			}
		}
	}
}


//获取与目标单元格相邻且可通行的单元格
void SquareGrid::getNeighbors(const GridLocation& gridPt, vector<GridLocation>& neighbors)
{
	int x, y, dx, dy;
	x = gridPt.first;
	y = gridPt.second;

	for (int i = 0; i < 8; ++i)
	{
		dx = m_DIRS[i].first;
		dy = m_DIRS[i].second;
		GridLocation next(x + dx, y + dy);

		if (isInBounds(next) && isPassable(next))
		{
			neighbors.push_back(next);
		}
	}
}


//对路径进行优化处理，使其更平滑些：当路径中两点间的连线没有障碍时，清除这两点间所有的中间点
void SquareGrid::refinePath(vector<GridLocation>& path)
{
	if (path.size() <= 2)	return;

	GridLocation firstPt = path[0];
	for (size_t i = 2; i < path.size();)
	{
		bool isWayPassable = true;
		GridLocation secondPt = path[i];

		int minX = min(firstPt.first, secondPt.first);
		int maxX = max(firstPt.first, secondPt.first);
		int minY = min(firstPt.second, secondPt.second);
		int maxY = max(firstPt.second, secondPt.second);

		for (int ii = minX; ii <= maxX; ++ii)
		{
			for (int jj = minY; jj <= maxY; ++jj)
			{
				GridLocation tempPt = GridLocation(ii, jj);
				if (!isPassable(tempPt))
				{
					float distFromPt2Line = distFromPointToLine(firstPt, secondPt, tempPt);
					if (distFromPt2Line <= 0.7071)			//障碍点距离当前直线很近
					{
						isWayPassable = false;
						break;
					}
				}
			}
			if (!isWayPassable)		break;
		}

		if (isWayPassable)
		{
			path.erase(path.begin() + i - 1);
		}
		else
		{
			firstPt = path[i - 1];
			++i;
		}
	}
}


//预处理面对象，将面对象栅格化，并将面对象的进口或出口进行两两配对，形成所有可能的线的集合
void TopoMap::_preprocessAreas()
{
	map<ID, pArea>::iterator iterArea;
	for (iterArea = m_areas.begin(); iterArea != m_areas.end(); ++iterArea)
	{
		pArea pCurrArea = iterArea->second;
		vector<ID>* pAttachNodeIDs = &pCurrArea->m_attachNodeIDs;

		pCurrArea->createGrid();				//栅格化区域

		for (size_t i = 0; i < pAttachNodeIDs->size(); ++i)
		{
			for (size_t j = i + 1; j < pAttachNodeIDs->size(); ++j)
			{
				ID aNodeID = (*pAttachNodeIDs)[i];
				ID bNodeID = (*pAttachNodeIDs)[j];
				pNode pANode = m_nodes[aNodeID];
				pNode pBNode = m_nodes[bNodeID];
				pLine pNewLine = new Line((++m_currMaxUsedID), pCurrArea->m_floorNo, pCurrArea->m_grade, aNodeID, bNodeID, BiDir);
				if (!pCurrArea->openSpaceSearchOnMap(pANode->m_location, pBNode->m_location, pNewLine->m_points))
				{
					delete	pNewLine;
					pNewLine = NULL;
					continue;
				}
				pNewLine->calculateLineLength();
				m_lines.insert(pair<ID, pLine>(pNewLine->m_id, pNewLine));
			}
		}
	}
	m_maxUsedIDAfterPreprocess = m_currMaxUsedID;
}


//构造道路拓扑网
void TopoMap::_constructRoadNetwork()
{
	//遍历所有线状对象，将其转变成用于路径查找算法中使用的道路对象
	map<ID, pLine>::iterator iterLine;
	for (iterLine = m_lines.begin(); iterLine != m_lines.end(); ++iterLine)
	{
		pLine pCurrLine = iterLine->second;
		_addNewRoadIntoTopo(pCurrLine, true);
	}
}


//找出指定点在哪个区域内, 返回值为 0： 不在任何区域内； 1： 在某一区域内； -1：在某一区域的内边界内（即在某一区域的不可通行区域内） 
int TopoMap::_inWhichArea(const CVector2& mapPt, const float floorNo, ID& areaID)
{
	map<ID, pArea>::iterator iter;
	for (iter = m_areas.begin(); iter != m_areas.end(); ++iter)
	{
		pArea pCurrArea = iter->second;

		//如果点和区域不属于同一楼层，则继续查找下个区域
		if (fabs(floorNo - pCurrArea->m_floorNo) > 0.001)	continue;

		int result = pCurrArea->m_geo.isPointInsideGeometry(mapPt);
		switch (result)
		{
		case 0:
			continue;
		case 1:
			areaID = pCurrArea->m_id;
			return 1;
		case -1:
			return -1;
		}
	}

	return 0;
}


//找出指定点距离哪条线状道路最近
bool TopoMap::_nearWhichLine(const CVector2& mapPt, const float floorNo, ID& lineID, CVector3& nearestPt, int& segmentNo, float& minDist)
{
	minDist = FLT_MAX;
	CVector3 mapPtCV3 = CVector3(mapPt.x, mapPt.y, 0.0f);			//z值暂时设为0
	CVector3 tempPt;

	map<ID, pLine>::iterator iter;
	for (iter = m_lines.begin(); iter != m_lines.end(); ++iter)
	{
		pLine pCurrLine = iter->second;
		//如果点和线不属于同一楼层，则继续查找下一条线
		if (fabs(floorNo - pCurrLine->m_floorNo) > 0.001)	continue;

		mapPtCV3.z = pCurrLine->m_points[0].z;
		for (size_t i = 1; i < pCurrLine->m_points.size(); ++i)
		{
			//点到线段的最近点和最短距离。如果垂足在线段上，则最近点为垂足；如果垂足不在线段上，则最近点为线段两端点中与点距离最近的点
			float dist;
			int result = minDistFromPointToLineSegment(pCurrLine->m_points[i], pCurrLine->m_points[i - 1], mapPtCV3, tempPt, dist);
			if (-1 == result)	continue;
			if (minDist > dist)
			{
				minDist = dist;
				nearestPt = tempPt;
				lineID = pCurrLine->m_id;
				segmentNo = i;
			}
		}
	}
	return !(minDist >= (FLT_MAX - 0.1f));
}


float TopoMap::_heuristicForRoad(const ID startNodeID, const ID destNodeID)
{
	return distBetweenTwoPoints(m_nodes[startNodeID]->m_location, m_nodes[destNodeID]->m_location);
}


//基于路网的A*搜索算法
void TopoMap::_AStarSearchOnRoadNetwork(const ID& startNodeID, const ID& destNodeID, map<ID, ID>& cameFrom, map<ID, float>& costSoFar)
{
	PriorityQueue<ID, float> frontier;
	frontier.put(startNodeID, 0.0f);
	//cameFrom存储父子节点关系的列表，用于追溯生成路径
	cameFrom[startNodeID] = startNodeID;
	costSoFar[startNodeID] = 0.0f;

	while (!frontier.empty())
	{
		//当有序集不为空的时候，取有序集第一个
		ID currentNodeID = frontier.get();
		if (currentNodeID == destNodeID)
		{
			break;		//找到目标就停止
		}

		pair<multimap<ID, Road>::iterator, multimap<ID, Road>::iterator> ret = m_allRoads.equal_range(currentNodeID);		//找出所有以current为起点的道路
		for (multimap<ID, Road>::iterator iter = ret.first; iter != ret.second; ++iter)
		{
			Road nextRoad = iter->second;
			ID nextNodeID = nextRoad.m_bNodeId;
			float newCost = costSoFar[currentNodeID] + nextRoad.m_roadLength;
			if (!costSoFar.count(nextNodeID) || newCost < costSoFar[nextNodeID])
			{
				costSoFar[nextNodeID] = newCost;
				frontier.put(nextNodeID, _heuristicForRoad(nextNodeID, destNodeID) + newCost);
				cameFrom[nextNodeID] = currentNodeID;
			}
		}
	}
}


//将道路节点路径转换成点位坐标路径
void TopoMap::_roadPthToPointPath(const vector<ID>& roadPath)
{
	for (size_t i = 1, size = roadPath.size(); i < size; ++i)
	{
		ID aNodeID = roadPath[i - 1];
		ID bNodeID = roadPath[i];
		pair<multimap<ID, Road>::iterator, multimap<ID, Road>::iterator> ret = m_allRoads.equal_range(aNodeID);
		for (multimap<ID, Road>::iterator iter = ret.first; iter != ret.second; ++iter)
		{
			if (bNodeID == iter->second.m_bNodeId)
			{
				pLine pCurrLine = m_lines[iter->second.m_id];
				size_t startIndex, endIndex;
				if (aNodeID == pCurrLine->m_aNodeId)
				{
					startIndex = (0 == m_pathOnMap.size()) ? 0 : 1;
					endIndex = pCurrLine->m_points.size() - 1;
					for (size_t j = startIndex; j <= endIndex; ++j)
					{
						m_pathOnMap.push_back(pCurrLine->m_points[j]);
					}
				}
				else
				{
					startIndex = (0 == m_pathOnMap.size()) ? (pCurrLine->m_points.size() - 1) : (pCurrLine->m_points.size() - 2);
					for (int j = startIndex; j >= 0; --j)
					{
						m_pathOnMap.push_back(pCurrLine->m_points[j]);
					}
				}
				break;
			}
		}
	}
}


//在路网上查找从起点到终点的线路
bool TopoMap::_pathSearchOnRoadNetwork(const ID& startNodeID, const ID& destNodeID)
{
	vector<ID> roadPath;
	map<ID, ID> cameFrom;
	map<ID, float> costSoFar;
	_AStarSearchOnRoadNetwork(startNodeID, destNodeID, cameFrom, costSoFar);
	if (!constructPath(startNodeID, destNodeID, cameFrom, roadPath))
	{
		return false;
	}
	_roadPthToPointPath(roadPath);
	return true;
}


//将位于面状区域内的起点或者终点添加到道路拓扑中
ID TopoMap::_addAreaPointIntoTopo(const CVector2& mapPt, const ID areaID)
{
	pArea pTargetArea = m_areas[areaID];
	float zCoordOfArea = pTargetArea->m_geo.m_points[0].z;
	CVector3 mapPtV3(mapPt.x, mapPt.y, zCoordOfArea);

	for (size_t i = 0, size = pTargetArea->m_attachNodeIDs.size(); i < size; ++i)
	{
		ID currNodeID = pTargetArea->m_attachNodeIDs[i];
		if (mapPtV3 == m_nodes[currNodeID]->m_location)
		{
			return currNodeID;
		}
	}

	ID newNodeID = (++m_currMaxUsedID);
	//添加点
	pNode pNewNode = new Node(newNodeID, pTargetArea->m_floorNo, pTargetArea->m_grade, mapPtV3);
	m_nodes.insert(pair<ID, pNode>(newNodeID, pNewNode));

	for (size_t i = 0, size = pTargetArea->m_attachNodeIDs.size(); i < size; ++i)
	{
		ID currNodeID = pTargetArea->m_attachNodeIDs[i];
		pNode currNode = m_nodes[currNodeID];

		//添加线		 
		pLine pNewLine = new Line((++m_currMaxUsedID), pTargetArea->m_floorNo, pTargetArea->m_grade, newNodeID, currNodeID, BiDir);
		if (!pTargetArea->openSpaceSearchOnMap(mapPtV3, currNode->m_location, pNewLine->m_points))
		{
			delete	pNewLine;
			pNewLine = NULL;
			continue;
		}
		pNewLine->calculateLineLength();
		m_lines.insert(pair<ID, pLine>(pNewLine->m_id, pNewLine));

		//添加道路
		_addNewRoadIntoTopo(pNewLine, false);
	}
	return newNodeID;
}


//将不位于任何面状区域内的起点或者终点添加到道路拓扑中（将距离最近的线打断成两条线）
ID TopoMap::_addLinePointIntoTopo(const CVector3& mapPt, const ID lineID, const int segmentNo, const CVector3& nearestPt)
{
	pLine pTargetLine = m_lines[lineID];
	ID mapPtID, nearestPtID;
	bool isNearestNodeCreated = false;
	bool bNeedToBreakInTwo = true;			//是否需要分成两段
	bool isMapPtIDEqualNearestPtID = (mapPt == nearestPt);	//mapPt和nearestPt重合,将使用同一节点ID

	nearestPtID = _addNewNodeOnLineIntoTopo(nearestPt, pTargetLine);
	if (nearestPtID == pTargetLine->m_aNodeId || nearestPtID == pTargetLine->m_bNodeId)
	{
		if (isMapPtIDEqualNearestPtID)
		{
			return nearestPtID;
		}
		bNeedToBreakInTwo = false;
	}

	if (isMapPtIDEqualNearestPtID)
	{
		mapPtID = nearestPtID;
	}
	else
	{
		//添加点
		mapPtID = (++m_currMaxUsedID);
		pNode pNewNode = new Node(mapPtID, pTargetLine->m_floorNo, pTargetLine->m_grade, mapPt);
		m_nodes.insert(pair<ID, pNode>(mapPtID, pNewNode));

		//添加线
		pLine pNewLine = new Line((++m_currMaxUsedID), pTargetLine->m_floorNo, pTargetLine->m_grade, mapPtID, nearestPtID, BiDir);
		pNewLine->m_points.push_back(mapPt);
		pNewLine->m_points.push_back(nearestPt);
		pNewLine->calculateLineLength();
		m_lines.insert(pair<ID, pLine>(pNewLine->m_id, pNewLine));

		//添加道路
		_addNewRoadIntoTopo(pNewLine, false);
	}

	if (!bNeedToBreakInTwo)	return mapPtID;

	//将原先的线分成两条
	_breakLineIntoTwoParts(pTargetLine, segmentNo, nearestPtID, nearestPt);

	return mapPtID;
}


//将线打断成两条线
void TopoMap::_breakLineIntoTwoParts(const pLine& pTargetLine, const int segmentNo, const ID seperatePtID, const CVector3& seperatePt)
{
	ID linePart1ID = (++m_currMaxUsedID);
	ID linePart2ID = (++m_currMaxUsedID);
	pLine pLinePart1 = new Line(linePart1ID, pTargetLine->m_floorNo, pTargetLine->m_grade, pTargetLine->m_aNodeId, seperatePtID, pTargetLine->m_dir);
	pLine pLinePart2 = new Line(linePart2ID, pTargetLine->m_floorNo, pTargetLine->m_grade, seperatePtID, pTargetLine->m_bNodeId, pTargetLine->m_dir);

	for (int i = 0; i < segmentNo; ++i)
	{
		pLinePart1->m_points.push_back(pTargetLine->m_points[i]);
	}
	pLinePart1->m_points.push_back(seperatePt);
	pLinePart1->calculateLineLength();

	if (seperatePt != pTargetLine->m_points[segmentNo])
	{
		pLinePart2->m_points.push_back(seperatePt);
	}
	for (size_t i = segmentNo, size = pTargetLine->m_points.size(); i < size; ++i)
	{
		pLinePart2->m_points.push_back(pTargetLine->m_points[i]);
	}
	pLinePart2->calculateLineLength();
	//添加线
	m_lines.insert(pair<ID, pLine>(linePart1ID, pLinePart1));
	m_lines.insert(pair<ID, pLine>(linePart2ID, pLinePart2));
	//添加道路
	_addNewRoadIntoTopo(pLinePart1, false);
	_addNewRoadIntoTopo(pLinePart2, false);
	//把原先线对应的roads从m_allRoads中移除并备份
	_removeAndBackupRoads(pTargetLine);
}



//当起点和终点在同一线上时，构造从起点到终点的路径
bool TopoMap::_constructPathOnSameLine(const pLine& pTargetLine, const int segmentNoForStartPt, const int segmentNoForDestPt, const vector<CVector3>& ptsInfo)
{
	CVector3 startMapPt = ptsInfo[0];
	CVector3 nearestPtForStartPt = ptsInfo[1];
	CVector3 nearestPtForDestPt = ptsInfo[2];
	CVector3 destMapPt = ptsInfo[3];

	m_pathOnMap.push_back(startMapPt);
	if (startMapPt != nearestPtForStartPt)
	{
		m_pathOnMap.push_back(nearestPtForStartPt);
	}

	if (segmentNoForStartPt == segmentNoForDestPt)
	{
		float dist1 = distBetweenTwoPoints(nearestPtForStartPt, pTargetLine->m_points[segmentNoForStartPt - 1]);
		float dist2 = distBetweenTwoPoints(nearestPtForDestPt, pTargetLine->m_points[segmentNoForStartPt - 1]);
		//当dist1等于dist2时
		if (fabs(dist1 - dist2) < PRECISION)
		{
			if (destMapPt != nearestPtForDestPt)
			{
				m_pathOnMap.push_back(destMapPt);
			}
			return true;
		}

		if (BiDir == pTargetLine->m_dir || (dist1 < dist2 && PosDir == pTargetLine->m_dir) || (dist1 > dist2 && NegDir == pTargetLine->m_dir))
		{
			m_pathOnMap.push_back(nearestPtForDestPt);
			if (destMapPt != nearestPtForDestPt)
			{
				m_pathOnMap.push_back(destMapPt);
			}
			return true;
		}
	}

	if (segmentNoForStartPt < segmentNoForDestPt && (BiDir == pTargetLine->m_dir || PosDir == pTargetLine->m_dir))		//此时当前线路必须是双向或者正向，才能沿着当前线路从起点到达终点
	{
		if (nearestPtForStartPt != pTargetLine->m_points[segmentNoForStartPt])
		{
			m_pathOnMap.push_back(pTargetLine->m_points[segmentNoForStartPt]);
		}

		for (int n = segmentNoForStartPt + 1; n < segmentNoForDestPt; ++n)
		{
			m_pathOnMap.push_back(pTargetLine->m_points[n]);
		}

		if (nearestPtForDestPt != pTargetLine->m_points[segmentNoForDestPt - 1])
		{
			m_pathOnMap.push_back(nearestPtForDestPt);
		}

		if (destMapPt != nearestPtForDestPt)
		{
			m_pathOnMap.push_back(destMapPt);
		}
		return true;
	}

	if (segmentNoForStartPt > segmentNoForDestPt && (BiDir == pTargetLine->m_dir || NegDir == pTargetLine->m_dir))		//此时当前线路必须是双向或者反向，才能沿着当前线路从起点到达终点
	{
		if (nearestPtForStartPt != pTargetLine->m_points[segmentNoForStartPt - 1])
		{
			m_pathOnMap.push_back(pTargetLine->m_points[segmentNoForStartPt - 1]);
		}

		for (int n = segmentNoForStartPt - 1; n > segmentNoForDestPt; --n)
		{
			m_pathOnMap.push_back(pTargetLine->m_points[n - 1]);
		}

		if (nearestPtForDestPt != pTargetLine->m_points[segmentNoForDestPt])
		{
			m_pathOnMap.push_back(nearestPtForDestPt);
		}

		if (destMapPt != nearestPtForDestPt)
		{
			m_pathOnMap.push_back(destMapPt);
		}
		return true;
	}

	//单行道，不可直接沿着线从起点逆行到终点,只能将起点和终点加入路网中，寻找其他可通行的线路
	ID startNodeID, destNodeID;
	m_pathOnMap.clear();
	_insertTwoPointsToLine(pTargetLine, ptsInfo, segmentNoForStartPt, segmentNoForDestPt, startNodeID, destNodeID);
	return _pathSearchOnRoadNetwork(startNodeID, destNodeID);
}


//在线上插入两个点
void TopoMap::_insertTwoPointsToLine(const pLine& pTargetLine, const vector<CVector3>& ptsInfo, const int segmentNoForStartPt, const int segmentNoForDestPt, ID& startNodeID, ID& destNodeID)
{
	CVector3 startMapPt = ptsInfo[0];
	CVector3 nearestPtForStartPt = ptsInfo[1];
	CVector3 nearestPtForDestPt = ptsInfo[2];
	CVector3 destMapPt = ptsInfo[3];
	ID nearestPtForStartID, nearestPtForDestID;
	bool isStartIDEqualNearstPtID = (startMapPt == nearestPtForStartPt);
	bool isDestIDEqualNearstPtID = (destMapPt == nearestPtForDestPt);
	bool isNearestNodeForStartCreated = false;
	bool isNearestNodeForDestCreated = false;
	int partCount = 3;			//需要把线分割成几段

	nearestPtForStartID = _addNewNodeOnLineIntoTopo(nearestPtForStartPt, pTargetLine);
	if (nearestPtForStartID == pTargetLine->m_aNodeId || nearestPtForStartID == pTargetLine->m_bNodeId)
	{
		destNodeID = _addLinePointIntoTopo(destMapPt, pTargetLine->m_id, segmentNoForDestPt, nearestPtForDestPt);
		if (isStartIDEqualNearstPtID)
		{
			startNodeID = nearestPtForStartID;
			return;
		}
		--partCount;
	}

	nearestPtForDestID = _addNewNodeOnLineIntoTopo(nearestPtForDestPt, pTargetLine);
	if (nearestPtForDestID == pTargetLine->m_aNodeId || nearestPtForDestID == pTargetLine->m_bNodeId)
	{
		startNodeID = _addLinePointIntoTopo(startMapPt, pTargetLine->m_id, segmentNoForStartPt, nearestPtForStartPt);
		if (isDestIDEqualNearstPtID)
		{
			destNodeID = nearestPtForDestID;
			return;
		}
		--partCount;
	}

	if (isStartIDEqualNearstPtID)
	{
		startNodeID = nearestPtForStartID;
	}
	else
	{
		//添加点
		startNodeID = (++m_currMaxUsedID);
		pNode pNewNode = new Node(startNodeID, pTargetLine->m_floorNo, pTargetLine->m_grade, startMapPt);
		m_nodes.insert(pair<ID, pNode>(startNodeID, pNewNode));
		//添加线
		pLine pNewLine = new Line((++m_currMaxUsedID), pTargetLine->m_floorNo, pTargetLine->m_grade, startNodeID, nearestPtForStartID, BiDir);
		pNewLine->m_points.push_back(startMapPt);
		pNewLine->m_points.push_back(nearestPtForStartPt);
		pNewLine->calculateLineLength();
		m_lines.insert(pair<ID, pLine>(pNewLine->m_id, pNewLine));
		//添加道路
		_addNewRoadIntoTopo(pNewLine, false);
	}

	if (isDestIDEqualNearstPtID)
	{
		destNodeID = nearestPtForDestID;
	}
	else
	{
		//添加点
		destNodeID = (++m_currMaxUsedID);
		pNode pNewNode = new Node(destNodeID, pTargetLine->m_floorNo, pTargetLine->m_grade, destMapPt);
		m_nodes.insert(pair<ID, pNode>(destNodeID, pNewNode));
		//添加线
		pLine pNewLine = new Line((++m_currMaxUsedID), pTargetLine->m_floorNo, pTargetLine->m_grade, destNodeID, nearestPtForDestID, BiDir);
		pNewLine->m_points.push_back(destMapPt);
		pNewLine->m_points.push_back(nearestPtForDestPt);
		pNewLine->calculateLineLength();
		m_lines.insert(pair<ID, pLine>(pNewLine->m_id, pNewLine));
		//添加道路
		_addNewRoadIntoTopo(pNewLine, false);
	}
	if (3 != partCount)	return;
	_breakLineIntoThreeParts(pTargetLine, nearestPtForStartID, nearestPtForDestID, segmentNoForStartPt, segmentNoForDestPt);
}


void TopoMap::_breakLineIntoThreeParts(const pLine& pTargetLine, const ID startNodeID, const ID destNodeID, const int segmentNoForStartPt, const int segmentNoForDestPt)
{
	ID linePart1ID = (++m_currMaxUsedID);
	ID linePart2ID = (++m_currMaxUsedID);
	ID linePart3ID = (++m_currMaxUsedID);
	pLine pLinePart1, pLinePart2, pLinePart3;
	ID firstNodeID = startNodeID;
	ID secondNodeID = destNodeID;
	CVector3 firstNode = m_nodes[startNodeID]->m_location;
	CVector3 secondNode = m_nodes[destNodeID]->m_location;
	int firstSegNo = segmentNoForStartPt;
	int secondSegNo = segmentNoForDestPt;
	if (segmentNoForStartPt > segmentNoForDestPt || (segmentNoForStartPt == segmentNoForDestPt && distBetweenTwoPoints(pTargetLine->m_points[segmentNoForStartPt - 1], firstNode) > distBetweenTwoPoints(pTargetLine->m_points[segmentNoForStartPt - 1], secondNode)))
	{
		swap(firstNodeID, secondNodeID);
		swap(firstNode, secondNode);
		swap(firstSegNo, secondSegNo);
	}

	pLinePart1 = new Line(linePart1ID, pTargetLine->m_floorNo, pTargetLine->m_grade, pTargetLine->m_aNodeId, firstNodeID, pTargetLine->m_dir);
	pLinePart2 = new Line(linePart2ID, pTargetLine->m_floorNo, pTargetLine->m_grade, firstNodeID, secondNodeID, pTargetLine->m_dir);
	pLinePart3 = new Line(linePart3ID, pTargetLine->m_floorNo, pTargetLine->m_grade, secondNodeID, pTargetLine->m_bNodeId, pTargetLine->m_dir);
	for (int i = 0; i < firstSegNo; ++i)
	{
		pLinePart1->m_points.push_back(pTargetLine->m_points[i]);
	}
	pLinePart1->m_points.push_back(firstNode);

	if (firstNode != pTargetLine->m_points[firstSegNo]) {
		pLinePart2->m_points.push_back(firstNode);
	}
	for (int i = firstSegNo; i < secondSegNo; ++i)
	{
		pLinePart2->m_points.push_back(pTargetLine->m_points[i]);
	}
	pLinePart2->m_points.push_back(secondNode);

	if (secondNode != pTargetLine->m_points[secondSegNo]) {
		pLinePart3->m_points.push_back(secondNode);
	}
	for (size_t i = secondSegNo, size = pTargetLine->m_points.size(); i < size; ++i)
	{
		pLinePart3->m_points.push_back(pTargetLine->m_points[i]);
	}
	//添加线
	pLinePart1->calculateLineLength();
	pLinePart2->calculateLineLength();
	pLinePart3->calculateLineLength();
	m_lines.insert(pair<ID, pLine>(pLinePart1->m_id, pLinePart1));
	m_lines.insert(pair<ID, pLine>(pLinePart2->m_id, pLinePart2));
	m_lines.insert(pair<ID, pLine>(pLinePart3->m_id, pLinePart3));
	//添加道路
	_addNewRoadIntoTopo(pLinePart1, false);
	_addNewRoadIntoTopo(pLinePart2, false);
	_addNewRoadIntoTopo(pLinePart3, false);
	//把原先线对应的roads从m_allRoads中移除并备份
	_removeAndBackupRoads(pTargetLine);
}


void TopoMap::_restoreTopoMap()
{
	//删除为算法新增加的点
	map<ID, pNode>::iterator iterNode;
	for (iterNode = m_nodes.begin(); iterNode != m_nodes.end();)
	{
		pNode pCurrNode = iterNode->second;
		if (pCurrNode->m_id > m_maxUsedIDAfterPreprocess)
		{
			delete pCurrNode;
			pCurrNode = NULL;
			m_nodes.erase(iterNode++);
			continue;
		}
		++iterNode;
	}

	//删除为算法新增的线
	map<ID, pLine>::iterator iterLine;
	for (iterLine = m_lines.begin(); iterLine != m_lines.end();)
	{
		pLine pCurrLine = iterLine->second;
		if (pCurrLine->m_id > m_maxUsedIDAfterPreprocess)
		{
			delete pCurrLine;
			pCurrLine = NULL;
			m_lines.erase(iterLine++);
			continue;
		}
		++iterLine;
	}

	//删除为算法新增的road
	multimap<ID, Road>::iterator iterRoad;
	for (iterRoad = m_allRoads.begin(); iterRoad != m_allRoads.end();)
	{
		if (!iterRoad->second.m_flag)
		{
			m_allRoads.erase(iterRoad++);
			continue;
		}
		++iterRoad;
	}

	//将之前移除的road对象重新添加回去
	for (iterRoad = m_backupRoads.begin(); iterRoad != m_backupRoads.end(); ++iterRoad)
	{
		m_allRoads.insert(pair<ID, Road>(iterRoad->first, iterRoad->second));
	}

	m_backupRoads.clear();
	m_currMaxUsedID = m_maxUsedIDAfterPreprocess;
	m_pathOnMap.clear();
}


//初始化
void TopoMap::initTopoMap()
{
	clock_t startTime1, finishTime1;			//test
	startTime1 = clock();						//test

	cout << "In preprocess..." << endl << "It may take some time. Please wait patiently!" << endl;
	_preprocessAreas();
	cout << "Preprocess completed!" << endl;
	cout << "Begin to constructing road network" << endl;
	_constructRoadNetwork();
	cout << "constructing road network completed!" << endl;

	finishTime1 = clock();				//test
	cout << "初始化共耗时：" << ((double)(finishTime1 - startTime1) / CLOCKS_PER_SEC) << "秒！\n" << endl;			//test
}


//查到地图上连接起点和终点的路线
bool TopoMap::searchPath(const CVector2& startMapPt, const float startFloorNo, const CVector2& destMapPt, const float destFloorNo)
{
	cout << "Searching path is in process..." << endl;
	if (startMapPt == destMapPt && startFloorNo == destFloorNo)	return false;		//如果起点和终点是同一个点，则直接退出

	ID areaIDForStartPt, areaIDForDestPt;
	ID lineIDForStartPt, lineIDForDestPt;
	ID startNodeID, destNodeID;
	CVector3 nearestPtForStartPt, nearestPtForDestPt;
	float distForStartPt, distForDestPt;
	int segmentNoForStartPt, segmentNoForDestPt;
	int result1, result2;

	_restoreTopoMap();

	result1 = _inWhichArea(startMapPt, startFloorNo, areaIDForStartPt);
	if (-1 == result1)
	{
		cout << "起点位于不可通行区域内" << endl;
		return false;
	}

	result2 = _inWhichArea(destMapPt, destFloorNo, areaIDForDestPt);
	if (-1 == result2)
	{
		cout << "终点位于不可通行区域内" << endl;
		return false;
	}

	//起点和终点在同一区域（面状路网）内
	if (1 == result1 && 1 == result2 && areaIDForStartPt == areaIDForDestPt)
	{
		pArea pTargetArea = m_areas[areaIDForStartPt];
		float zCoordOfArea = pTargetArea->m_geo.m_points[0].z;
		return pTargetArea->openSpaceSearchOnMap(CVector3(startMapPt.x, startMapPt.y, zCoordOfArea), CVector3(destMapPt.x, destMapPt.y, zCoordOfArea), m_pathOnMap);
	}

	switch (result1)
	{
	case 0:
		//起点不在任何区域内，则找与点距离最近的线
		if (!_nearWhichLine(startMapPt, startFloorNo, lineIDForStartPt, nearestPtForStartPt, segmentNoForStartPt, distForStartPt))
		{
			cout << "起点不靠近任何线" << endl;
			return false;
		}
		break;
	case 1:
		//将起点与所在区域的出入口两两相连，并将形成的新的线路并入路网中（注意备份，以便后续查找还原原始路网）
		startNodeID = _addAreaPointIntoTopo(startMapPt, areaIDForStartPt);
		break;
	}

	switch (result2)
	{
	case 0:
		//终点不在任何区域内，则找与点距离最近的线
		if (!_nearWhichLine(destMapPt, destFloorNo, lineIDForDestPt, nearestPtForDestPt, segmentNoForDestPt, distForDestPt))
		{
			cout << "终点不靠近任何线" << endl;
			return false;
		}
		break;
	case 1:
		//将终点与所在区域的出入口两两相连，并将形成的新的线路并入路网中（注意备份，以便后续查找还原原始路网）
		destNodeID = _addAreaPointIntoTopo(destMapPt, areaIDForDestPt);
		break;
	}

	//起点和终点位于同一线状路网上
	if (0 == result1 && 0 == result2 && lineIDForStartPt == lineIDForDestPt)
	{
		pLine pTargetLine = m_lines[lineIDForStartPt];
		float z = pTargetLine->m_points[0].z;
		CVector3 startPtV3(startMapPt.x, startMapPt.y, nearestPtForStartPt.z);
		CVector3 destPtV3(destMapPt.x, destMapPt.y, nearestPtForDestPt.z);
		vector<CVector3> ptsInfo;
		ptsInfo.push_back(startPtV3);
		ptsInfo.push_back(nearestPtForStartPt);
		ptsInfo.push_back(nearestPtForDestPt);
		ptsInfo.push_back(destPtV3);
		return _constructPathOnSameLine(pTargetLine, segmentNoForStartPt, segmentNoForDestPt, ptsInfo);
	}

	//将找到的线一分为二，并加入路网中(注意原来的线需要备份，以便下次查找时还原回原始路网)
	if (0 == result1)
	{
		CVector3 startPtV3(startMapPt.x, startMapPt.y, nearestPtForStartPt.z);
		startNodeID = _addLinePointIntoTopo(startPtV3, lineIDForStartPt, segmentNoForStartPt, nearestPtForStartPt);
	}

	if (0 == result2)
	{
		CVector3 destPtV3(destMapPt.x, destMapPt.y, nearestPtForDestPt.z);
		destNodeID = _addLinePointIntoTopo(destPtV3, lineIDForDestPt, segmentNoForDestPt, nearestPtForDestPt);
	}

	return _pathSearchOnRoadNetwork(startNodeID, destNodeID);
}


//退出并释放资源
void TopoMap::exitAndRelease()
{
	map<ID, pNode>::iterator iterNode;
	for (iterNode = m_nodes.begin(); iterNode != m_nodes.end(); ++iterNode)
	{
		pNode pCurrNode = iterNode->second;
		delete pCurrNode;
		pCurrNode = NULL;
	}
	m_nodes.clear();

	map<ID, pLine>::iterator iterLine;
	for (iterLine = m_lines.begin(); iterLine != m_lines.end(); ++iterLine)
	{
		pLine pCurrLine = iterLine->second;
		delete pCurrLine;
		pCurrLine = NULL;
	}
	m_lines.clear();

	map<ID, pArea>::iterator iterArea;
	for (iterArea = m_areas.begin(); iterArea != m_areas.end(); ++iterArea)
	{
		pArea pCurrArea = iterArea->second;
		delete pCurrArea->m_grid.m_passageInfo;
		delete pCurrArea;
		pCurrArea = NULL;
	}
	m_areas.clear();
}


#define MAXBUFSIZE 10000000
static char buf[MAXBUFSIZE];
//获取路径查找结果，以字符串形式返回
const char* TopoMap::getPathPointResult()
{
	stringstream ss;
	for (size_t i = 0; i < m_pathOnMap.size(); i++)
	{
		ss << m_pathOnMap[i].x << "," << m_pathOnMap[i].y << "," << m_pathOnMap[i].z << ";";
	}

	string str = ss.str();
	int n = str.length() + 1;
	if (n >= MAXBUFSIZE)
	{
		return "";
	}

	strncpy_s(buf, n, str.c_str(), n);
	return buf;
}


//获取路径查找结果,结果将以字符串形式赋值给传入的参数
bool TopoMap::getPathPointResultEx(char* result)
{
	stringstream ss;
	for (size_t i = 0; i < m_pathOnMap.size(); i++)
	{
		ss << m_pathOnMap[i].x << "," << m_pathOnMap[i].y << "," << m_pathOnMap[i].z << ";";
	}

	int n = ss.str().length() + 1;
	strncpy_s(result, n, ss.str().c_str(), n);

	return true;
}

//获取点坐标串结果的字节大小
int TopoMap::getByteSizeOfPathPointResult()
{
	stringstream ss;
	for (size_t i = 0; i < m_pathOnMap.size(); i++)

	{
		ss << m_pathOnMap[i].x << "," << m_pathOnMap[i].y << "," << m_pathOnMap[i].z << ";";
	}

	return ss.str().length() + 1;
}



void TopoMap::_addNewRoadIntoTopo(const pLine& pTargetLine, const bool flag)
{
	if (BiDir == pTargetLine->m_dir || PosDir == pTargetLine->m_dir)
	{
		Road newRoad(pTargetLine->m_id, pTargetLine->m_aNodeId, pTargetLine->m_bNodeId, pTargetLine->m_totalLength);
		newRoad.m_flag = flag;
		m_allRoads.insert(pair<ID, Road>(newRoad.m_aNodeId, newRoad));
	}

	if (BiDir == pTargetLine->m_dir || NegDir == pTargetLine->m_dir)
	{
		Road newRoad(pTargetLine->m_id, pTargetLine->m_bNodeId, pTargetLine->m_aNodeId, pTargetLine->m_totalLength);
		newRoad.m_flag = flag;
		m_allRoads.insert(pair<ID, Road>(newRoad.m_aNodeId, newRoad));
	}
}

ID TopoMap::_addNewNodeOnLineIntoTopo(const CVector3& mapPt, const pLine& pTargetLine)
{
	if (mapPt == m_nodes[pTargetLine->m_aNodeId]->m_location)
	{
		return pTargetLine->m_aNodeId;
	}
	if (mapPt == m_nodes[pTargetLine->m_bNodeId]->m_location)
	{
		return pTargetLine->m_bNodeId;
	}

	ID newNodeID = (++m_currMaxUsedID);
	pNode pNewNode = new Node(newNodeID, pTargetLine->m_floorNo, pTargetLine->m_grade, mapPt);
	m_nodes.insert(pair<ID, pNode>(newNodeID, pNewNode));
	return newNodeID;
}


void TopoMap::_removeAndBackupRoads(const pLine& pTargetLine)
{
	if (BiDir == pTargetLine->m_dir || PosDir == pTargetLine->m_dir)
	{
		pair<multimap<ID, Road>::iterator, multimap<ID, Road>::iterator> ret = m_allRoads.equal_range(pTargetLine->m_aNodeId);
		for (multimap<ID, Road>::iterator iter = ret.first; iter != ret.second;)
		{
			if (pTargetLine->m_id == iter->second.m_id)
			{
				m_backupRoads.insert(*iter);
				m_allRoads.erase(iter++);
				break;
			}
			++iter;
		}
	}

	if (BiDir == pTargetLine->m_dir || NegDir == pTargetLine->m_dir)
	{
		pair<multimap<ID, Road>::iterator, multimap<ID, Road>::iterator> ret = m_allRoads.equal_range(pTargetLine->m_bNodeId);
		for (multimap<ID, Road>::iterator iter = ret.first; iter != ret.second;)
		{
			if (pTargetLine->m_id == iter->second.m_id)
			{
				m_backupRoads.insert(*iter);
				m_allRoads.erase(iter++);
				break;
			}
			++iter;
		}
	}
}