#pragma once
#include "Skeleton.h"

class CScene;

class SkeletonSampler
{
public:
	SkeletonSampler();
	SkeletonSampler(CScene *scene);
	~SkeletonSampler();

	void setSkeleton(Skeleton *skel) { m_inputSkeleton = skel; };

	void sampleSkeletonAroundModel(int modelID);
	std::vector<Skeleton*> getSampledSkeletons() { return m_sampledSkeletons; };
	std::vector<double> getSampleRange(int modelID) { return m_sampleRegions[modelID]; };

	bool isHardConflictWithScene(Skeleton *skel);

private:
	Skeleton *m_inputSkeleton;
	std::vector<Skeleton*> m_sampledSkeletons;
	CScene *m_scene;

	std::vector<std::vector<double>> m_sampleRegions;
};

