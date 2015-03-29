#include "SkeletonSampler.h"
#include "Scene.h"

const double DistanceThreshold = 0.8;
const double SampleGridSize = 0.4;

SkeletonSampler::SkeletonSampler()
{

}

SkeletonSampler::SkeletonSampler(CScene *scene):
m_scene(scene)
{
	m_sampleRegions.resize(m_scene->getModelNum());
}

SkeletonSampler::~SkeletonSampler()
{
}

void SkeletonSampler::sampleSkeletonAroundModel(int modelID)
{
	m_sampledSkeletons.clear();

	CModel *m = m_scene->getModel(modelID);

	std::vector<double> sampleRange = m->getAABBXYRange();

	sampleRange[0] -= DistanceThreshold; // xmin
	sampleRange[1] += DistanceThreshold; // xmax
	sampleRange[2] -= DistanceThreshold; // ymin
	sampleRange[3] += DistanceThreshold; // ymax

	m_sampleRegions[modelID] = sampleRange;

	int xGridNum, yGridNum;

	xGridNum = (int)(sampleRange[1] - sampleRange[0]) / SampleGridSize;
	yGridNum = (int)(sampleRange[3] - sampleRange[2]) / SampleGridSize;

	for (int i = 0; i < xGridNum; i++)
	{
		for (int j = 0; j < yGridNum; j++)
		{
			for (int k = 0; k < 8; k++)
			{
				std::vector<MathLib::Vector3> joints = m_inputSkeleton->getTransformedJoints(sampleRange[0] + i*SampleGridSize, sampleRange[2] + j*SampleGridSize, k*MathLib::ML_PI_4, m_scene->getUprightVec());

				Skeleton *sampledSkeleton = new Skeleton(joints);

				if (!isHardConflictWithScene(sampledSkeleton))
				{
					m_sampledSkeletons.push_back(sampledSkeleton);
				}
			}
		}
	}
}

bool SkeletonSampler::isHardConflictWithScene(Skeleton *skel)
{
	for (int m_id = 0; m_id < m_scene->getModelNum(); m_id++)
	{	
		//// test for joints
		//for (int jt_id = 0; jt_id < skel->jointNum(); jt_id++)
		//{
		//	MathLib::Vector3 joint = skel->getJoint(jt_id);

		//	bool isConflict = m_scene->isInsideModel(SurfaceMesh::Vector3(joint.x, joint.y, joint.z), m_id);

		//	if (isConflict)
		//	{
		//		return true;
		//	}
		//}

		// test for bones
		std::vector<SurfaceMesh::Vector3> joints = skel->getSurfaceMeshJoints();
		int boneNum = 19;

		for (int bi = 0; bi < boneNum; bi++)
		{
			bool isConflict = m_scene->isIntersectModel(joints[BoneMap[bi][0]], joints[BoneMap[bi][1]], m_id);

			if (isConflict)
			{
				return true;
			}
		}
	}

	return false;
}
