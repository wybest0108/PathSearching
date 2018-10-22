//*********************************
//作者: Wei Ying
//完成日期：2018/9/30
//*********************************

#include <queue>
#include <vector>
#include <map>
#include <algorithm>
#include <functional>

#define PRECISION 1.0E-3f
using namespace std;

enum Direction
{
	BiDir = 0,		//双向
	PosDir = 1,		//正向
	NegDir = 2		//反向
};


//用于表示区域栅格化后点的位置坐标
typedef pair<int, int> GridLocation;


//用于表示一个点的平面位置坐标
class CVector2
{
public:
	float x;
	float y;

	CVector2() {};
	CVector2(float _x, float _y) : x(_x), y(_y) {};
	friend bool operator == (const CVector2&, const CVector2&);
	friend bool operator != (const CVector2&, const CVector2&);
};
typedef CVector2* pCVector2;


//用于表示一个点的空间位置坐标
class CVector3
{
public:
	float x;
	float y;
	float z;

	CVector3() {};
	CVector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {};
	friend bool operator == (const CVector3&, const CVector3&);
	friend bool operator != (const CVector3&, const CVector3&);
};
typedef CVector3* pCVector3;


//优先队列
template<typename T, typename Number = float>
struct PriorityQueue
{
	typedef pair<Number, T> PQElement;
	//priority_queue后两个参数缺省时，优先队列是大顶堆，队头最大。greater表示队列为小顶堆，less表示队列为大顶堆
	priority_queue<PQElement, vector<PQElement>, greater<PQElement>> elements;

	inline bool empty() const
	{
		return elements.empty();
	}

	inline void put(T item, Number priority)
	{
		elements.push(PQElement(priority, item));
	}

	inline T get()
	{
		//取有序集顶元素并删除
		T best_item = elements.top().second;
		elements.pop();
		return best_item;
	}
};


//启发式函数
float heuristic(const GridLocation& pointA, const GridLocation& pointB);

//计算两点间通行成本
float cost(const GridLocation& pointA, const GridLocation& pointB);

//计算平面上点到直线的距离，参数类型为GridLocation
float distFromPointToLine(const GridLocation& aPointOfLine, const GridLocation& bPointOfLine, const GridLocation& point);

//计算空间中两点间的距离
float distBetweenTwoPoints(const CVector3& pointA, const CVector3& pointB);

//计算空间中点到线段的最近距离和最近点.返回值为0时，表示垂足在线段外； 为1时，表示垂足在线段上; 为-1时，异常
int minDistFromPointToLineSegment(const CVector3& aNodeOfLine, const CVector3& bNodeOfLine, const CVector3& point, CVector3& nearestPt, float& distance);

//判断点是否在多边形内
int isPointInPolygon(const CVector2& pt, const vector<CVector2>& path);

//从A*算法结果中提取出从起点到终点的路径
template<typename locate>
bool constructPath(const locate& start, const locate& dest, const map<locate, locate>& cameFrom, vector<locate>& path)
{
	locate current = dest;
	path.push_back(current);
	while (current != start)
	{
		map<locate, locate>::const_iterator iter = cameFrom.find(current);
		if (iter != cameFrom.end())
		{
			current = iter->second;
			path.push_back(current);
		}
		else
		{
			return false;
		}

	}
	reverse(path.begin(), path.end());
	return true;
}
