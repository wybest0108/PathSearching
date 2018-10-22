//*********************************
//����: Wei Ying
//������ڣ�2018/9/30
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

//����ʽ�����������õ���ŷ�Ͼ���,�������ϰ����ʱ�������������Ҫ�ߵľ���
float heuristic(const GridLocation& pointA, const GridLocation& pointB)
{
	float dx, dy;
	dx = (float)abs(pointA.first - pointB.first);
	dy = (float)abs(pointA.second - pointB.second);
	return 1.1f * sqrt(dx * dx + dy * dy);		//1.1Ϊ��ʤֵ
}


//���������ͨ�гɱ����˴����������ɱ���
float cost(const GridLocation& pointA, const GridLocation& pointB)
{
	//����ǶԽ����ϵģ�����б�ԽǾ���,���򷵻�1
	return (abs(pointA.first - pointB.first) + abs(pointA.second - pointB.second) == 2) ? 1.4142f : 1.0f;
}


//����ƽ���ϵ㵽ֱ�ߵľ��루A��B���㲻��Ϊͬһ���㣬�����޷�����һ��ֱ�ߣ���ʽ�л���ֳ�����������
float distFromPointToLine(const GridLocation& aPointOfLine, const GridLocation& bPointOfLine, const GridLocation& point)
{
	float coefA, coefB, coefC;

	coefA = (float)bPointOfLine.second - (float)aPointOfLine.second;
	coefB = (float)aPointOfLine.first - (float)bPointOfLine.first;
	coefC = (float)bPointOfLine.first * (float)aPointOfLine.second - (float)aPointOfLine.first * (float)bPointOfLine.second;

	return fabs(coefA * (float)point.first + coefB * (float)point.second + coefC) / sqrt(coefA * coefA + coefB * coefB);
}


//����ռ��������ľ���
float distBetweenTwoPoints(const CVector3& pointA, const CVector3& pointB)
{
	float dx = pointB.x - pointA.x;
	float dy = pointB.y - pointA.y;
	float dz = pointB.z - pointA.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}


//����ռ��е㵽�߶ε�������������㣨�߶ε�A��B���㲻��Ϊͬһ���㣬�����޷�����ֱ�ߣ���ʽ�л���ֳ�����������.����ֵΪ0ʱ����ʾ�������߶��⣻ Ϊ1ʱ����ʾ�������߶���; Ϊ-1ʱ���쳣
int minDistFromPointToLineSegment(const CVector3& aNodeOfLine, const CVector3& bNodeOfLine, const CVector3& point, CVector3& nearestPt, float& distance)
{
	float deltaX = (bNodeOfLine.x - aNodeOfLine.x);
	float deltaY = (bNodeOfLine.y - aNodeOfLine.y);
	float deltaZ = (bNodeOfLine.z - aNodeOfLine.z);
	if (fabs(deltaX) < PRECISION && fabs(deltaY) < PRECISION && fabs(deltaZ) < PRECISION)	return -1;

	float coefK = -(((aNodeOfLine.x - point.x) * deltaX + (aNodeOfLine.y - point.y) * deltaY + (aNodeOfLine.z - point.z) * deltaZ) / (deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ));

	//�������ʱȡ����
	nearestPt.x = coefK * deltaX + aNodeOfLine.x;
	nearestPt.y = coefK * deltaY + aNodeOfLine.y;
	nearestPt.z = coefK * deltaZ + aNodeOfLine.z;

	//��鴹���Ƿ����߶���
	float distFromFootPtToANode = distBetweenTwoPoints(aNodeOfLine, nearestPt);			//���㵽A�˵�ľ���
	float distFromFootPtToBNode = distBetweenTwoPoints(bNodeOfLine, nearestPt);			//���㵽B�˵�ľ���
	float lengthOfLineSegment = distBetweenTwoPoints(aNodeOfLine, bNodeOfLine);			//�߶γ���

	//�������߶���
	if ((distFromFootPtToANode + distFromFootPtToBNode) > lengthOfLineSegment)
	{
		nearestPt = distFromFootPtToANode > distFromFootPtToBNode ? bNodeOfLine : aNodeOfLine;
		distance = distFromFootPtToANode > distFromFootPtToBNode ? distBetweenTwoPoints(bNodeOfLine, point) : distBetweenTwoPoints(aNodeOfLine, point);
		return 0;
	}

	//�������߶���
	distance = distBetweenTwoPoints(point, nearestPt);
	return 1;
}
