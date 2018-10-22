//*********************************
//����: Wei Ying
//������ڣ�2018/9/30
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
	BiDir = 0,		//˫��
	PosDir = 1,		//����
	NegDir = 2		//����
};


//���ڱ�ʾ����դ�񻯺���λ������
typedef pair<int, int> GridLocation;


//���ڱ�ʾһ�����ƽ��λ������
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


//���ڱ�ʾһ����Ŀռ�λ������
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


//���ȶ���
template<typename T, typename Number = float>
struct PriorityQueue
{
	typedef pair<Number, T> PQElement;
	//priority_queue����������ȱʡʱ�����ȶ����Ǵ󶥶ѣ���ͷ���greater��ʾ����ΪС���ѣ�less��ʾ����Ϊ�󶥶�
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
		//ȡ���򼯶�Ԫ�ز�ɾ��
		T best_item = elements.top().second;
		elements.pop();
		return best_item;
	}
};


//����ʽ����
float heuristic(const GridLocation& pointA, const GridLocation& pointB);

//���������ͨ�гɱ�
float cost(const GridLocation& pointA, const GridLocation& pointB);

//����ƽ���ϵ㵽ֱ�ߵľ��룬��������ΪGridLocation
float distFromPointToLine(const GridLocation& aPointOfLine, const GridLocation& bPointOfLine, const GridLocation& point);

//����ռ��������ľ���
float distBetweenTwoPoints(const CVector3& pointA, const CVector3& pointB);

//����ռ��е㵽�߶ε��������������.����ֵΪ0ʱ����ʾ�������߶��⣻ Ϊ1ʱ����ʾ�������߶���; Ϊ-1ʱ���쳣
int minDistFromPointToLineSegment(const CVector3& aNodeOfLine, const CVector3& bNodeOfLine, const CVector3& point, CVector3& nearestPt, float& distance);

//�жϵ��Ƿ��ڶ������
int isPointInPolygon(const CVector2& pt, const vector<CVector2>& path);

//��A*�㷨�������ȡ������㵽�յ��·��
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
