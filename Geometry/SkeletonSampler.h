#pragma once
#include "Skeleton.h"

class CScene;
class SuppPlane;

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

	std::vector<Skeleton*> getSampledStartSkeletons() { return m_sampledStartSkeletons; };
	std::vector<Skeleton*> getSampledEndSkeletons() { return m_sampledEndSkeletons; };
	
	std::vector<double> getSampleRange(int modelID) { return m_sampleStartRegions[modelID]; };
	std::vector<MathLib::Vector3>& getSamplePositions(int modelID) { return m_sampleStartPositions[modelID]; };

	bool isHardConflictWithScene(Skeleton *skel);

	void sampleArrangeSkeletonsInScene(const QString &method);

private:
	Skeleton *m_inputSkeleton;
	std::vector<Skeleton*> m_inputSkeletonList;

	std::vector<Skeleton*> m_sampledStartSkeletons;
	std::vector<Skeleton*> m_sampledEndSkeletons;

	CScene *m_scene;

	std::vector<double> m_floorXYRange;
	std::vector<std::vector<double>> m_sampleStartRegions;
	std::vector<std::vector<MathLib::Vector3>> m_sampleStartPositions;
};

