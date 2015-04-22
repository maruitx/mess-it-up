#include "SkeletonSampler.h"
#include "Scene.h"
#include "SuppPlane.h"
#include "../Action/ActionFeature.h"

SkeletonSampler::SkeletonSampler()
{

}

SkeletonSampler::SkeletonSampler(CScene *scene):
m_scene(scene)
{
	m_sampleStartRegions.resize(m_scene->getModelNum());
	m_sampleStartPositions.resize(m_scene->getModelNum());
}

SkeletonSampler::~SkeletonSampler()
{
}

void SkeletonSampler::sampleSkeletonAroundModel(int modelID)
{
	m_sampledStartSkeletons.clear();

	CModel *m = m_scene->getModel(modelID);

	std::vector<double> sampleRange = m->getAABBXYRange();

	sampleRange[0] -= ReachDistThreshold; // xmin
	sampleRange[1] += ReachDistThreshold; // xmax
	sampleRange[2] -= ReachDistThreshold; // ymin
	sampleRange[3] += ReachDistThreshold; // ymax

	sampleRange[0] = std::max(sampleRange[0], m_floorXYRange[0]);
	sampleRange[1] = std::min(sampleRange[1], m_floorXYRange[1]);
	sampleRange[2] = std::max(sampleRange[2], m_floorXYRange[2]);
	sampleRange[3] = std::min(sampleRange[3], m_floorXYRange[3]);

	m_sampleStartRegions[modelID] = sampleRange;

	int xGridNum, yGridNum;

	xGridNum = (int)((sampleRange[1] - sampleRange[0]) / SampleGridSize);
	yGridNum = (int)((sampleRange[3] - sampleRange[2]) / SampleGridSize);

	for (int i = 0; i < xGridNum + 1; i++)
	{
		for (int j = 0; j < yGridNum + 1; j++)
		{
			MathLib::Vector3 samplePos = MathLib::Vector3(sampleRange[0] + i*SampleGridSize, sampleRange[2] + j*SampleGridSize, 0);

			m_sampleStartPositions[modelID].push_back(samplePos);

			for (int k = 0; k < 8; k++)
			{
				std::vector<MathLib::Vector3> joints = m_inputSkeleton->getTransformedJoints(samplePos.x, samplePos.y, 0, k*MathLib::ML_PI_4, m_scene->getUprightVec());

				Skeleton *sampledSkeleton = new Skeleton(joints);

				if (!isHardConflictWithScene(sampledSkeleton))
				//if (!m_scene->isIntersectModel(SurfaceMesh::Vector3(samplePos.x, samplePos.y, -1), SurfaceMesh::Vector3(samplePos.x, samplePos.y, 3), 0))
				{
					sampledSkeleton->setScene(m_scene);
					m_sampledStartSkeletons.push_back(sampledSkeleton);
				}			
			}
		}
	}
}

void SkeletonSampler::sampleSkeletonAroundModelFromSkelList(int modelID, const QString &method)
{
	m_sampledStartSkeletons.clear();

	CModel *m = m_scene->getModel(modelID);

	std::vector<double> sampleRange = m->getAABBXYRange();

	sampleRange[0] -= ReachDistThreshold; // xmin
	sampleRange[1] += ReachDistThreshold; // xmax
	sampleRange[2] -= ReachDistThreshold; // ymin
	sampleRange[3] += ReachDistThreshold; // ymax

	sampleRange[0] = std::max(sampleRange[0], m_floorXYRange[0]);
	sampleRange[1] = std::min(sampleRange[1], m_floorXYRange[1]);
	sampleRange[2] = std::max(sampleRange[2], m_floorXYRange[2]);
	sampleRange[3] = std::min(sampleRange[3], m_floorXYRange[3]);

	m_sampleStartRegions[modelID] = sampleRange;

	int xGridNum, yGridNum;

	xGridNum = (int)((sampleRange[1] - sampleRange[0]) / SampleGridSize);
	yGridNum = (int)((sampleRange[3] - sampleRange[2]) / SampleGridSize);

	for (int i = 0; i < xGridNum + 1; i++)
	{
		for (int j = 0; j < yGridNum + 1; j++)
		{
			MathLib::Vector3 samplePos = MathLib::Vector3(sampleRange[0] + i*SampleGridSize, sampleRange[2] + j*SampleGridSize, 0);

			m_sampleStartPositions[modelID].push_back(samplePos);

			for (int k = 0; k < 8; k++)
			{
				if (method == "random")
				{
					int id = std::rand() % m_inputSkeletonList.size(); // random select

					std::vector<MathLib::Vector3> joints = m_inputSkeletonList[id]->getTransformedJoints(samplePos.x, samplePos.y, samplePos.z, k*MathLib::ML_PI_4, m_scene->getUprightVec());

					Skeleton *sampledSkeleton = new Skeleton(joints);
					sampleTestForSkeleton(sampledSkeleton, ActionFeature::ActionPhase::StartAction, samplePos);
				}

				if (method == "all")
				{
					for (int id = 0; id < m_inputSkeletonList.size(); id++)
					{
						std::vector<MathLib::Vector3> joints = m_inputSkeletonList[id]->getTransformedJoints(samplePos.x, samplePos.y, samplePos.z, k*MathLib::ML_PI_4, m_scene->getUprightVec());

						Skeleton *sampledSkeleton = new Skeleton(joints);
						sampleTestForSkeleton(sampledSkeleton, ActionFeature::ActionPhase::StartAction, samplePos);
					}
				}
			}
		}
	}
}

// need to improve: ignore too far way objects
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

		if (m_scene->getModelName(m_id) != "floor")
		{
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
	}

	return false;
}

void SkeletonSampler::setFloorRange(const std::vector<double> &floorXY)
{
	m_floorXYRange = floorXY;
}

void SkeletonSampler::sampleArrangeSkeletonsInScene(const QString &method)
{
	m_sampledEndSkeletons.clear();

	for (int i = 0; i < m_scene->getModelNum(); i++)
	{
		QString modelName = m_scene->getModelName(i);
		CModel* m = m_scene->getModel(i);

		if (modelName == "floor" || modelName == "chair" || modelName == "sofa" || modelName == "bed")
		{
			if (!m->getAllSuppPlanes().empty())
			{
				SuppPlane* suppPlaneForSampling = m->getLargestAreaSuppPlane();

				MathLib::Vector3 planeFirstCorner = suppPlaneForSampling->GetCorner(0);
				std::vector<MathLib::Vector3> planeAxis = suppPlaneForSampling->GetAxis();
				MathLib::Vector3 planeCenter = suppPlaneForSampling->GetCenter();

				double planeLength = suppPlaneForSampling->GetLength();
				double planeWidth = suppPlaneForSampling->GetWidth();

				int xGridNum, yGridNum;

				xGridNum = (int)(planeLength / SampleGridSize);
				yGridNum = (int)(planeWidth / SampleGridSize);

				for (int i = 0; i < xGridNum + 1; i++)
				{
					for (int j = 0; j < yGridNum + 1; j++)
					{
						//MathLib::Vector3 samplePos = MathLib::Vector3(planeFirstCorner[0] + i*SampleGridSize*planeAxis[0], planeFirstCorner[2] + j*SampleGridSize, 0);
						MathLib::Vector3 samplePos = planeFirstCorner + planeAxis[0] * i*SampleGridSize + planeAxis[1] * j*SampleGridSize;

						for (int k = 0; k < 8; k++)
						{
							if (method == "random")
							{
								int id = std::rand() % m_inputSkeletonList.size(); // random select

								std::vector<MathLib::Vector3> joints = m_inputSkeletonList[id]->getTransformedJoints(samplePos.x, samplePos.y, samplePos.z, k*MathLib::ML_PI_4, m_scene->getUprightVec());

								Skeleton *sampledSkeleton = new Skeleton(joints);
								sampleTestForSkeleton(sampledSkeleton, ActionFeature::ActionPhase::EndAction,samplePos);
								
							}

							if (method == "all")
							{
								for (int id = 0; id < m_inputSkeletonList.size(); id++)
								{
									std::vector<MathLib::Vector3> joints = m_inputSkeletonList[id]->getTransformedJoints(samplePos.x, samplePos.y, samplePos.z, k*MathLib::ML_PI_4, m_scene->getUprightVec());

									Skeleton *sampledSkeleton = new Skeleton(joints);
									sampleTestForSkeleton(sampledSkeleton, ActionFeature::ActionPhase::EndAction, samplePos);
								}
							}
						}
					}
				}
			}
		}
	}
}

void SkeletonSampler::sampleTestForSkeleton(Skeleton *skel, int phaseID, const MathLib::Vector3 &samplePos)
{

	if (!isHardConflictWithScene(skel))
		//if (!m_scene->isIntersectModel(SurfaceMesh::Vector3(samplePos.x, samplePos.y, -1), SurfaceMesh::Vector3(samplePos.x, samplePos.y, 3), 0))
	{
		skel->setSampledPos(samplePos);

		if (phaseID == ActionFeature::ActionPhase::StartAction)
		{
			m_sampledStartSkeletons.push_back(skel);
		}

		if (phaseID == ActionFeature::ActionPhase::EndAction)
		{
			skel->setScene(m_scene);

			// compute all potential planes to place object			
			skel->computePlacementRegion();
			m_sampledEndSkeletons.push_back(skel);

			// filter out useless skeleton, e.g no place to place object around (may not happen since there is always floor, but still possible)
			if (!skel->hasAccessiblePlanes())
			{
				m_sampledEndSkeletons.pop_back();
			}
		}
	}
}
