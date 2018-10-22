//*********************************
//作者: Wei Ying
//完成日期：2018/9/30
//*********************************

#include "helper.h"
#include <math.h>

bool operator == (const CVector2& cv2_A, const CVector2& cv2_B)
{
	return (fabs(cv2_A.x - cv2_B.x) < PRECISION && fabs(cv2_A.y - cv2_B.y) < PRECISION);
}

bool operator != (const CVector2& cv2_A, const CVector2& cv2_B)
{
	return !(cv2_A == cv2_B);
}


bool operator == (const CVector3& cv3_A, const CVector3& cv3_B)
{
	return (fabs(cv3_A.x - cv3_B.x) < PRECISION && fabs(cv3_A.y - cv3_B.y) < PRECISION && fabs(cv3_A.z - cv3_B.z) < PRECISION);
}

bool operator != (const CVector3& cv3_A, const CVector3& cv3_B)
{
	return !(cv3_A == cv3_B);
}

//启发式函数，这里用的是欧氏距离,不过有障碍物的时候这个并不代表要走的距离
float heuristic(const GridLocation& pointA, const GridLocation& pointB)
{
	float dx, dy;
	dx = (float)abs(pointA.first - pointB.first);
	dy = (float)abs(pointA.second - pointB.second);
	return 1.1f * sqrt(dx * dx + dy * dy);		//1.1为决胜值
}


//计算两点间通行成本（此处按距离计算成本）
float cost(const GridLocation& pointA, const GridLocation& pointB)
{
	//如果是对角线上的，返回斜对角距离,否则返回1
	return (abs(pointA.first - pointB.first) + abs(pointA.second - pointB.second) == 2) ? 1.4142f : 1.0f;
}


//计算平面上点到直线的距离（A、B两点不能为同一个点，否则无法构成一条直线，公式中会出现除以零的情况）
float distFromPointToLine(const GridLocation& aPointOfLine, const GridLocation& bPointOfLine, const GridLocation& point)
{
	float coefA, coefB, coefC;

	coefA = (float)bPointOfLine.second - (float)aPointOfLine.second;
	coefB = (float)aPointOfLine.first - (float)bPointOfLine.first;
	coefC = (float)bPointOfLine.first * (float)aPointOfLine.second - (float)aPointOfLine.first * (float)bPointOfLine.second;

	return fabs(coefA * (float)point.first + coefB * (float)point.second + coefC) / sqrt(coefA * coefA + coefB * coefB);
}


//计算空间中两点间的距离
float distBetweenTwoPoints(const CVector3& pointA, const CVector3& pointB)
{
	float dx = pointB.x - pointA.x;
	float dy = pointB.y - pointA.y;
	float dz = pointB.z - pointA.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}


//计算空间中点到线段的最近距离和最近点（线段的A、B两点不能为同一个点，否则无法构成直线，公式中会出现除以零的情况）.返回值为0时，表示垂足在线段外； 为1时，表示垂足在线段上; 为-1时，异常
int minDistFromPointToLineSegment(const CVector3& aNodeOfLine, const CVector3& bNodeOfLine, const CVector3& point, CVector3& nearestPt, float& distance)
{
	float deltaX = (bNodeOfLine.x - aNodeOfLine.x);
	float deltaY = (bNodeOfLine.y - aNodeOfLine.y);
	float deltaZ = (bNodeOfLine.z - aNodeOfLine.z);
	if (fabs(deltaX) < PRECISION && fabs(deltaY) < PRECISION && fabs(deltaZ) < PRECISION)	return -1;

	float coefK = -(((aNodeOfLine.x - point.x) * deltaX + (aNodeOfLine.y - point.y) * deltaY + (aNodeOfLine.z - point.z) * deltaZ) / (deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ));

	//最近点暂时取垂足
	nearestPt.x = coefK * deltaX + aNodeOfLine.x;
	nearestPt.y = coefK * deltaY + aNodeOfLine.y;
	nearestPt.z = coefK * deltaZ + aNodeOfLine.z;

	//检查垂足是否在线段上
	float distFromFootPtToANode = distBetweenTwoPoints(aNodeOfLine, nearestPt);			//垂足到A端点的距离
	float distFromFootPtToBNode = distBetweenTwoPoints(bNodeOfLine, nearestPt);			//垂足到B端点的距离
	float lengthOfLineSegment = distBetweenTwoPoints(aNodeOfLine, bNodeOfLine);			//线段长度

	//垂足在线段外
	if ((distFromFootPtToANode + distFromFootPtToBNode) > lengthOfLineSegment)
	{
		nearestPt = distFromFootPtToANode > distFromFootPtToBNode ? bNodeOfLine : aNodeOfLine;
		distance = distFromFootPtToANode > distFromFootPtToBNode ? distBetweenTwoPoints(bNodeOfLine, point) : distBetweenTwoPoints(aNodeOfLine, point);
		return 0;
	}

	//垂足在线段上
	distance = distBetweenTwoPoints(point, nearestPt);
	return 1;
}
