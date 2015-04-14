#pragma once
#include "Skeleton.h"


class CScene;

class SkeletonSampler
{
public:
	SkeletonSampler();
	SkeletonSampler(CScene *scene);
	~SkeletonSampler();

	void setFloorRange(const std::vector<double> &floorXY);
	void setSkeleton(Skeleton *skel) { m_inputSkeleton = skel; };
	void setSkeletonList(std::vector<Skeleton*> skelList) { m_inputSkeletonList = skelList; };

	void sampleSkeletonAroundModel(int modelID);
	void sampleSkeletonAroundModelFromSkelList(int modelID, const QString &method);

	std::vector<Skeleton*> getSampledSkeletons() { return m_sampledSkeletons; };
	
	std::vector<double> getSampleRange(int modelID) { return m_sampleRegions[modelID]; };
	std::vector<MathLib::Vector3>& getSamplePositions(int modelID) { return m_samplePositions[modelID]; };

	bool isHardConflictWithScene(Skeleton *skel);

private:
	Skeleton *m_inputSkeleton;
	std::vector<Skeleton*> m_inputSkeletonList;

	std::vector<Skeleton*> m_sampledSkeletons;
	CScene *m_scene;

	std::vector<double> m_floorXYRange;
	std::vector<std::vector<double>> m_sampleRegions;
	std::vector<std::vector<MathLib::Vector3>> m_samplePositions;
};

