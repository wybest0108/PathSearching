//*********************************************************************************************
//����:	Wei Ying
//������ڣ�2018/9/30
//��ע�������ڵ����㷨����A*�㷨���㷨�ο���Դ��https://www.redblobgames.com/pathfinding/a-star/
//*********************************************************************************************

#include "helper.h"
#define MINGRIDWIDTH 0.01f

typedef unsigned long long ID;
using namespace std;
//������������ļ�����״
class Geometry
{
public:
	int m_pointCount;										//��������
	int m_polygonCount;										//���������
	vector<CVector3> m_points;								//������������
	vector<int> m_polygonStartIndex;						//�������ʼ������

	int isPointInsideGeometry(const CVector2& point);					//�жϵ��Ƿ��������ڣ�һ��������ܰ����������Σ�
	bool calculateClipBox(CVector3& clipMin, CVector3& clipMax);		//���������Χ��

private:
	int _isPointInPolygon(const CVector2& pt, const int startIndex, const int endIndex);	//�жϵ��Ƿ�������ĵ����������
};
typedef Geometry* pGeometry;


//����դ�񻯺��������
class SquareGrid
{
public:
	static GridLocation m_DIRS[8];
	int m_width;							//�����ȣ�����x����ĵ�Ԫ������
	int m_height;							//����߶ȣ�����y����ĵ�Ԫ������
	int m_gridSize;							//�����ܹ������ĵ�Ԫ������
	unsigned char *m_passageInfo = NULL;	//����Ԫ��ͨ����Ϣ��һ����Ԫ����һ��λ���洢��1 - ��ͨ�У� 0 - ����ͨ��

	SquareGrid() {};
	SquareGrid(int width, int height) : m_width(width), m_height(height) {};
	~SquareGrid();
	bool isInBounds(const GridLocation& gridPt);					//�жϵ��Ƿ�������������
	bool isPassable(const GridLocation& gridPt);					//�жϵ�Ԫ���Ƿ��ͨ��
	bool openSpaceSearchOnGrid(GridLocation& start, GridLocation& dest, vector<GridLocation>& path);		//�����������ڲ��Ҵ���㵽�յ��ͨ��·��
	void getNeighbors(const GridLocation& gridPt, vector<GridLocation>& neighbors);							//��ȡ��Ŀ�굥Ԫ�������ҿ�ͨ�еĵ�Ԫ��
	void AStarSearchOnGrid(const GridLocation& start, const GridLocation& dest, map<GridLocation, GridLocation>& cameFrom, map<GridLocation, float>& costSoFar);	//���������A*�����㷨
	void refinePath(vector<GridLocation>& path);			//�Ż�·����ʹ���ƽ��Щ

private:
	bool _adjustGridLocation(GridLocation& gridPt);		//�����߽�Ŀ�ͨ�е�ʵ�ʵ㣬դ�񻯺󣬿��ܻỮ�ֵ�����ͨ�еĵ�Ԫ���ϣ���ʱ��ҪѰ����������Ŀ�ͨ�Ե�Ԫ��������
};
typedef SquareGrid* pSquareGrid;


//�ڵ㡢�ߡ���Ļ���
class BaseElement
{
public:
	ID m_id;								//Ψһ��ʶ
	float m_floorNo;						//����¥��
	float m_grade;							//�ȼ�����ͬ��Ԫ��Ȩ�ز�һ�����������ͨ������ͨ����
	//unsigned long m_startTime;			//����ʱ�䣨Ŀǰ�ò�����Ԥ����
	//unsigned long m_endTime;				//�ر�ʱ�䣨Ŀǰ�ò�����Ԥ����

	BaseElement() {};
	BaseElement(ID id) : m_id(id) {};
	BaseElement(ID id, float floorNo, float grade) : m_id(id), m_floorNo(floorNo), m_grade(grade) {};
	//BaseElement(ID id, float floorNo, float grade, unsigned long startTime, unsigned long endTime) : m_id(id), m_floorNo(floorNo), m_grade(grade), m_startTime(startTime), m_endTime(endTime) {};
};


//�ڵ�Ԫ�أ��̳���BaseElement�࣬�������������·���˵㡢�š����ݵ�
class Node : public BaseElement
{
public:
	CVector3 m_location;				//�ڵ��λ������

	Node() {};
	Node(ID id) : BaseElement(id){};
	Node(ID id, float floorNo, float grade) : BaseElement(id, floorNo, grade) {};
	Node(ID id, float floorNo, float grade, const CVector3&);
};
typedef Node* pNode;


//��״Ԫ��(�����Ƕ���߶���ɵ�����)���̳���BaseElement�࣬���������·
class Line : public BaseElement
{
public:
	ID m_aNodeId;						//���ID
	ID m_bNodeId;						//ĩ��ID
	Direction m_dir;					//����0 - ˫�� 1 - ���� 2 - ����
	vector<CVector3> m_points;			//���ϵ����е㣨������Ԫ�ص���״����������㡢�۵㡢ĩ��
	float m_totalLength;				//�ܳ���

	Line() {};
	Line(ID id, float floorNo, float grade, ID aNodeId, ID bNodeId, Direction dir) : BaseElement(id, floorNo, grade), m_aNodeId(aNodeId), m_bNodeId(bNodeId), m_dir(dir), m_totalLength(0.0f) {};
	void calculateLineLength();				//�����ܳ���
};
typedef Line* pLine;


//��״Ԫ�أ��̳���BaseElement��
class Area : public BaseElement
{
public:
	Geometry m_geo;											//������״����
	CVector3 m_clipMax;										//�����Χ�����ǵ�
	CVector3 m_clipMin;										//�����Χ����С�ǵ�
	vector<ID> m_attachNodeIDs;								//���������ӵĽڵ�ID����
	float m_gridWidth;										//դ��������(�������ε�Ԫ��ı߳�)
	SquareGrid m_grid;										//դ�񻯺��������	

	Area() {};
	Area(ID id) : BaseElement(id) {};
	Area(ID id, float floorNo, float grade, float gridWidth) : BaseElement(id, floorNo, grade), m_gridWidth(gridWidth) {};
	bool calculateClipBox();								//���������Χ��
	bool createGrid();										//����դ������
	bool gridToMap(const GridLocation&, CVector2&);			//դ���λ����ת����ʵ�ʵ�λ����
	bool mapToGrid(const CVector2&, GridLocation&);			//ʵ�ʵ�λ����ת����դ���λ����
	bool openSpaceSearchOnMap(const CVector3&, const CVector3&, vector<CVector3>&);		//�ڵ�ͼ�����ڲ��Ҵ���㵽�յ��ͨ��·��
};
typedef Area* pArea;


//��·����·�������㷨��ʹ��
class Road
{
public:
	ID m_id;				//��Դ�������߶������������ID
	ID m_aNodeId;			//���ID
	ID m_bNodeId;			//ĩ��ID
	float m_roadLength;		//��·�ܳ�
	bool m_flag;			//ֵΪtrueʱ����ʾΪԭtopo�е�Road��Ϊfalseʱ����ʾΪ�������յ����topo�������ɵ�Road

	Road() {};
	Road(ID id, ID aNodeId, ID bNodeId) :m_id(id), m_aNodeId(aNodeId), m_bNodeId(bNodeId), m_roadLength(0.0f), m_flag(true) {};
	Road(ID id, ID aNodeId, ID bNodeId, float len) :m_id(id), m_aNodeId(aNodeId), m_bNodeId(bNodeId), m_roadLength(len), m_flag(true) {};
};


//���˵�ͼ����
class TopoMap
{
public:
	map<ID, pNode> m_nodes;						//�ڵ��������
	map<ID, pLine> m_lines;						//�߶�������,��������������������߶���
	map<ID, pArea> m_areas;						//���������
	ID m_currMaxUsedID;							//��ǰ��ʹ�õ������ID��(�㡢�ߡ��������Ԫ�ص�ID�Ų����ظ�)
	ID m_originalMaxLineID;						//ԭʼ������������ʹ�õ����ID�ţ�����������������������߶���
	multimap<ID, Road> m_allRoads;				//���еĵ����·��������A*�㷨��(�����ID��Ϊkey)
	ID m_maxUsedIDAfterPreprocess;				//����������Ԥ���������ʹ�õ������ID
	multimap<ID, Road> m_backupRoads;			//�洢��ʱ��m_allRoads�Ƴ���Road����
	vector<CVector3> m_pathOnMap;				//����������ҵ���·�����

	TopoMap() {};
	void initTopoMap();																											//��ʼ��
	bool searchPath(const CVector2& startMapPt, const float startFloorNo, const CVector2& destMapPt, const float destFloorNo);	//���ҵ�ͼ�����������յ��·��
	void exitAndRelease();																										//�˳����ͷ���Դ
	const char* getPathPointResult();																							//��ȡ·�����ҽ�������ַ�����ʽ����
	bool getPathPointResultEx(char* result);																					//��ȡ·�����ҽ��,��������ַ�����ʽ��ֵ������Ĳ���
	int getByteSizeOfPathPointResult();																							//��ȡ�����괮������ֽڴ�С

private:
	void _preprocessAreas();																																//Ԥ��������󣬽������դ�񻯣����������Ľ��ڻ���ڽ���������ԣ��γ����п��ܵ���ͨ·�ļ���
	void _constructRoadNetwork();																															//�����·������
	bool _constructPathOnSameLine(const pLine& pTargetLine, const int segmentNoForStartPt, const int segmentNoForDestPt, const vector<CVector3>& ptsInfo);	//�������յ���ͬһ����ʱ���������㵽�յ��·��
	ID _addAreaPointIntoTopo(const CVector2& mapPt, const ID areaID);																						//��λ����״�����ڵ��������յ���ӵ���·������
	ID _addLinePointIntoTopo(const CVector3& mapPt, const ID lineID, const int segmentNo, const CVector3& nearestPt);										//����λ���κ���״�����ڵ��������յ���ӵ���·�����У�������������ߴ�ϳ������ߣ�
	void _roadPthToPointPath(const vector<ID>& roadPath);																									//����·�ڵ�·��ת���ɵ�λ����·��
	void _AStarSearchOnRoadNetwork(const ID& startRoadID, const ID& destRoadID, map<ID, ID>& cameFrom, map<ID, float>& costSoFar);							//����·����A*�����㷨
	bool _pathSearchOnRoadNetwork(const ID& startNodeID, const ID& destNodeID);																				//��·���ϲ��Ҵ���㵽�յ����·
	int _inWhichArea(const CVector2& mapPt, const float floorNo, ID& areaID);																				//�ҳ�ָ�������ĸ�������	
	bool _nearWhichLine(const CVector2& mapPt, const float floorNo, ID& lineID, CVector3& nearestPt, int& segmentNo, float& minDist);						//�ҳ�ָ����������������
	float _heuristicForRoad(const ID nextNodeID, const ID destNodeID);																						//����ʽ����
	void _breakLineIntoTwoParts(const pLine& pTargetLine, const int segmentNo, const ID seperatePtID, const CVector3& seperatePt);							//���ߴ�ϳ�������
	void _restoreTopoMap();
	ID _addNewNodeOnLineIntoTopo(const CVector3& mapPt, const pLine& pTargetLine);							//����һ�����ϵĽڵ�
	void _addNewRoadIntoTopo(const pLine& pTargetLine, const bool flag);									//������߶�Ӧ�ĵ�·
	void _insertTwoPointsToLine(const pLine& pTargetLine, const vector<CVector3>& ptsInfo, const int segmentNoForStartPt, const int segmentNoForDestPt, ID& startNodeID, ID& endNodeID);
	void _breakLineIntoThreeParts(const pLine& pTargetLine, const ID startNodeID, const ID destNodeID, const int segmentNoForStartPt, const int segmentNoForDestPt);	//���ߴ�ϳ�����
	void _removeAndBackupRoads(const pLine& pTargetLine);
};


