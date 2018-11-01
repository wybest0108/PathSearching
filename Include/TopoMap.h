//*********************************************************************************************
//作者:	Wei Ying
//完成日期：2018/9/30
//备注：本室内导航算法基于A*算法，算法参考来源：https://www.redblobgames.com/pathfinding/a-star/
//*********************************************************************************************

#include "helper.h"
#define MINGRIDWIDTH 0.01f

typedef unsigned long long ID;
using namespace std;
//用于描述区域的几何形状
class Geometry
{
public:
	int m_pointCount;										//顶点数量
	int m_polygonCount;										//多边形数量
	vector<CVector3> m_points;								//顶点坐标数组
	vector<int> m_polygonStartIndex;						//多边形起始点索引

	int isPointInsideGeometry(const CVector2& point);					//判断点是否在区域内（一个区域可能包含多个多边形）
	bool calculateClipBox(CVector3& clipMin, CVector3& clipMax);		//计算区域包围盒

private:
	int _isPointInPolygon(const CVector2& pt, const int startIndex, const int endIndex);	//判断点是否在区域的单个多边形内
};
typedef Geometry* pGeometry;


//区域栅格化后网格对象
class SquareGrid
{
public:
	static GridLocation m_DIRS[8];
	int m_width;							//网格宽度，即在x方向的单元格数量
	int m_height;							//网格高度，即在y方向的单元格数量
	int m_gridSize;							//网格总共包含的单元格数量
	unsigned char *m_passageInfo = NULL;	//网格单元格通行信息，一个单元格用一个位来存储：1 - 可通行； 0 - 不可通行

	SquareGrid() {};
	SquareGrid(int width, int height) : m_width(width), m_height(height) {};
	~SquareGrid();
	bool isInBounds(const GridLocation& gridPt);					//判断点是否在网格区域内
	bool isPassable(const GridLocation& gridPt);					//判断单元格是否可通行
	bool openSpaceSearchOnGrid(GridLocation& start, GridLocation& dest, vector<GridLocation>& path);		//在网格区域内查找从起点到终点的通行路径
	void getNeighbors(const GridLocation& gridPt, vector<GridLocation>& neighbors);							//获取与目标单元格相邻且可通行的单元格
	void AStarSearchOnGrid(const GridLocation& start, const GridLocation& dest, map<GridLocation, GridLocation>& cameFrom, map<GridLocation, float>& costSoFar);	//基于网格的A*搜索算法
	void refinePath(vector<GridLocation>& path);			//优化路径，使其更平滑些

private:
	bool _adjustGridLocation(GridLocation& gridPt);		//靠近边界的可通行的实际点，栅格化后，可能会划分到不可通行的单元格上，此时需要寻找离它最近的可通性单元格来代替
};
typedef SquareGrid* pSquareGrid;


//节点、线、面的基类
class BaseElement
{
public:
	ID m_id;								//唯一标识
	float m_floorNo;						//所属楼层
	float m_grade;							//等级，不同的元素权重不一样，例如电梯通道和普通走廊
	//unsigned long m_startTime;			//开放时间（目前用不到，预留）
	//unsigned long m_endTime;				//关闭时间（目前用不到，预留）

	BaseElement() {};
	BaseElement(ID id) : m_id(id) {};
	BaseElement(ID id, float floorNo, float grade) : m_id(id), m_floorNo(floorNo), m_grade(grade) {};
	//BaseElement(ID id, float floorNo, float grade, unsigned long startTime, unsigned long endTime) : m_id(id), m_floorNo(floorNo), m_grade(grade), m_startTime(startTime), m_endTime(endTime) {};
};


//节点元素，继承自BaseElement类，可以用来定义道路两端点、门、电梯等
class Node : public BaseElement
{
public:
	CVector3 m_location;				//节点的位置坐标

	Node() {};
	Node(ID id) : BaseElement(id){};
	Node(ID id, float floorNo, float grade) : BaseElement(id, floorNo, grade) {};
	Node(ID id, float floorNo, float grade, const CVector3&);
};
typedef Node* pNode;


//线状元素(可以是多段线段组成的折线)，继承自BaseElement类，用来定义道路
class Line : public BaseElement
{
public:
	ID m_aNodeId;						//起点ID
	ID m_bNodeId;						//末点ID
	Direction m_dir;					//方向：0 - 双向， 1 - 正向， 2 - 反向
	vector<CVector3> m_points;			//线上的所有点（定义线元素的形状），包括起点、折点、末点
	float m_totalLength;				//总长度

	Line() {};
	Line(ID id, float floorNo, float grade, ID aNodeId, ID bNodeId, Direction dir) : BaseElement(id, floorNo, grade), m_aNodeId(aNodeId), m_bNodeId(bNodeId), m_dir(dir), m_totalLength(0.0f) {};
	void calculateLineLength();				//计算总长度
};
typedef Line* pLine;


//面状元素，继承自BaseElement类
class Area : public BaseElement
{
public:
	Geometry m_geo;											//几何形状对象
	CVector3 m_clipMax;										//区域包围盒最大角点
	CVector3 m_clipMin;										//区域包围盒最小角点
	vector<ID> m_attachNodeIDs;								//跟区域连接的节点ID数组
	float m_gridWidth;										//栅格化网格宽度(即正方形单元格的边长)
	SquareGrid m_grid;										//栅格化后网格对象	

	Area() {};
	Area(ID id) : BaseElement(id) {};
	Area(ID id, float floorNo, float grade, float gridWidth) : BaseElement(id, floorNo, grade), m_gridWidth(gridWidth) {};
	bool calculateClipBox();								//计算区域包围盒
	bool createGrid();										//生成栅格化网格
	bool gridToMap(const GridLocation&, CVector2&);			//栅格点位坐标转换成实际点位坐标
	bool mapToGrid(const CVector2&, GridLocation&);			//实际点位坐标转换成栅格点位坐标
	bool openSpaceSearchOnMap(const CVector3&, const CVector3&, vector<CVector3>&);		//在地图区域内查找从起点到终点的通行路径
};
typedef Area* pArea;


//道路对象，路径查找算法中使用
class Road
{
public:
	ID m_id;				//来源于拓扑线对象或者面对象的ID
	ID m_aNodeId;			//起点ID
	ID m_bNodeId;			//末点ID
	float m_roadLength;		//道路总长
	bool m_flag;			//值为true时，表示为原topo中的Road；为false时，表示为将起点或终点插入topo后新生成的Road

	Road() {};
	Road(ID id, ID aNodeId, ID bNodeId) :m_id(id), m_aNodeId(aNodeId), m_bNodeId(bNodeId), m_roadLength(0.0f), m_flag(true) {};
	Road(ID id, ID aNodeId, ID bNodeId, float len) :m_id(id), m_aNodeId(aNodeId), m_bNodeId(bNodeId), m_roadLength(len), m_flag(true) {};
};


//拓扑地图对象
class TopoMap
{
public:
	map<ID, pNode> m_nodes;						//节点对象数组
	map<ID, pLine> m_lines;						//线对象数组,包括由面对象抽象出来的线对象
	map<ID, pArea> m_areas;						//面对象数组
	ID m_currMaxUsedID;							//当前被使用到的最大ID号(点、线、面等所有元素的ID号不能重复)
	ID m_originalMaxLineID;						//原始拓扑数据中线使用的最大ID号（不包括由面对象抽象出来的线对象）
	multimap<ID, Road> m_allRoads;				//所有的单向道路，将用于A*算法中(以起点ID作为key)
	ID m_maxUsedIDAfterPreprocess;				//对面对象进行预处理结束后使用到的最大ID
	multimap<ID, Road> m_backupRoads;			//存储暂时从m_allRoads移除的Road对象
	vector<CVector3> m_pathOnMap;				//用来保存查找到的路径结果

	TopoMap() {};
	void initTopoMap();																											//初始化
	bool searchPath(const CVector2& startMapPt, const float startFloorNo, const CVector2& destMapPt, const float destFloorNo);	//查找地图上连接起点和终点的路线
	void exitAndRelease();																										//退出并释放资源
	const char* getPathPointResult();																							//获取路径查找结果，以字符串形式返回
	bool getPathPointResultEx(char* result);																					//获取路径查找结果,结果将以字符串形式赋值给传入的参数
	int getByteSizeOfPathPointResult();																							//获取点坐标串结果的字节大小

private:
	void _preprocessAreas();																																//预处理面对象，将面对象栅格化，并将面对象的进口或出口进行两两配对，形成所有可能的线通路的集合
	void _constructRoadNetwork();																															//构造道路拓扑网
	bool _constructPathOnSameLine(const pLine& pTargetLine, const int segmentNoForStartPt, const int segmentNoForDestPt, const vector<CVector3>& ptsInfo);	//当起点和终点在同一线上时，构造从起点到终点的路径
	ID _addAreaPointIntoTopo(const CVector2& mapPt, const ID areaID);																						//将位于面状区域内的起点或者终点添加到道路拓扑中
	ID _addLinePointIntoTopo(const CVector3& mapPt, const ID lineID, const int segmentNo, const CVector3& nearestPt);										//将不位于任何面状区域内的起点或者终点添加到道路拓扑中（将距离最近的线打断成两条线）
	void _roadPthToPointPath(const vector<ID>& roadPath);																									//将道路节点路径转换成点位坐标路径
	void _AStarSearchOnRoadNetwork(const ID& startRoadID, const ID& destRoadID, map<ID, ID>& cameFrom, map<ID, float>& costSoFar);							//基于路网的A*搜索算法
	bool _pathSearchOnRoadNetwork(const ID& startNodeID, const ID& destNodeID);																				//在路网上查找从起点到终点的线路
	int _inWhichArea(const CVector2& mapPt, const float floorNo, ID& areaID);																				//找出指定点在哪个区域内	
	bool _nearWhichLine(const CVector2& mapPt, const float floorNo, ID& lineID, CVector3& nearestPt, int& segmentNo, float& minDist);						//找出指定点距离哪条线最近
	float _heuristicForRoad(const ID nextNodeID, const ID destNodeID);																						//启发式函数
	void _breakLineIntoTwoParts(const pLine& pTargetLine, const int segmentNo, const ID seperatePtID, const CVector3& seperatePt);							//将线打断成两条线
	void _restoreTopoMap();
	ID _addNewNodeOnLineIntoTopo(const CVector3& mapPt, const pLine& pTargetLine);							//新增一个线上的节点
	void _addNewRoadIntoTopo(const pLine& pTargetLine, const bool flag);									//添加与线对应的道路
	void _insertTwoPointsToLine(const pLine& pTargetLine, const vector<CVector3>& ptsInfo, const int segmentNoForStartPt, const int segmentNoForDestPt, ID& startNodeID, ID& endNodeID);
	void _breakLineIntoThreeParts(const pLine& pTargetLine, const ID startNodeID, const ID destNodeID, const int segmentNoForStartPt, const int segmentNoForDestPt);	//将线打断成三段
	void _removeAndBackupRoads(const pLine& pTargetLine);
};


