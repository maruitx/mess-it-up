#pragma once

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/features/integral_image_normal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <pcl/registration/correspondence_estimation.h>
//#include <pcl/registration/icp.h>

//#include <boost/format.hpp>

#define FPS_CALC_BEGIN                          \
    static double duration = 0;                 \
    double start_time = pcl::getTime ();        \

#define FPS_CALC_END(_WHAT_)                    \
	    {                                             \
    double end_time = pcl::getTime ();          \
    static unsigned count = 0;                  \
    if (++count == 10)                          \
		    {                                           \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(duration) << " Hz" <<  std::endl; \
      count = 0;                                                        \
      duration = 0.0;                                                   \
		    }                                           \
			    else                                        \
			    {                                           \
      duration += end_time - start_time;        \
			    }                                           \
	    }

const int CounterNum = 10;

using namespace pcl::tracking;

template < typename PointType >
class DepthCloudTracker
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	DepthCloudTracker();

	//typedef pcl::PointXYZRGBANormal RefPointType;
	typedef pcl::PointXYZRGB RefPointType;
	//typedef pcl::PointXYZ RefPointType;
	typedef ParticleXYZRPY ParticleT;

	typedef pcl::PointCloud<PointType> Cloud;
	typedef pcl::PointCloud<RefPointType> RefCloud;
	typedef typename RefCloud::Ptr RefCloudPtr;
	typedef typename RefCloud::ConstPtr RefCloudConstPtr;
	typedef typename Cloud::Ptr CloudPtr;
	typedef typename Cloud::ConstPtr CloudConstPtr;
	//typedef KLDAdaptiveParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
	//typedef KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> ParticleFilter;
	//typedef ParticleFilterOMPTracker<RefPointType, ParticleT> ParticleFilter;
	typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;
	typedef typename ParticleFilter::CoherencePtr CoherencePtr;
	typedef typename pcl::search::KdTree<PointType> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;

	void init(/*pcl::visualization::CloudViewer *viewer,*/ int thread_nr, double downsampling_grid_size,
		bool use_convex_hull,
		bool visualize_non_downsample, bool visualize_particles,
		bool use_fixed);

	bool drawParticles(pcl::visualization::PCLVisualizer& viz);

	void drawResult(pcl::visualization::PCLVisualizer& viz);

	void drawRefCloud(pcl::visualization::PCLVisualizer& viz);

	void drawOfflineResult(pcl::visualization::PCLVisualizer& viz);

	void viz_cb(pcl::visualization::PCLVisualizer& viz);

	void filterPassThrough(const CloudConstPtr &cloud, Cloud &result);

	void euclideanSegment(const CloudConstPtr &cloud,
		std::vector<pcl::PointIndices> &cluster_indices);

	void gridSample(const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01);

	void gridSampleApprox(const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01);

	void planeSegmentation(const CloudConstPtr &cloud,
		pcl::ModelCoefficients &coefficients,
		pcl::PointIndices &inliers);

	void planeProjection(const CloudConstPtr &cloud,
		Cloud &result,
		const pcl::ModelCoefficients::ConstPtr &coefficients);

	void convexHull(const CloudConstPtr &cloud,
		Cloud &,
		std::vector<pcl::Vertices> &hull_vertices);

	void normalEstimation(const CloudConstPtr &cloud,
		pcl::PointCloud<pcl::Normal> &result);

	void tracking(const RefCloudConstPtr &cloud);

	void addNormalToCloud(const CloudConstPtr &cloud,
		const pcl::PointCloud<pcl::Normal>::ConstPtr &normals,
		RefCloud &result);

	void extractNonPlanePoints(const CloudConstPtr &cloud,
		const CloudConstPtr &cloud_hull,
		Cloud &result);

	void removeZeroPoints(const CloudConstPtr &cloud,
		Cloud &result);

	void extractSegmentCluster(const CloudConstPtr &cloud,
		const std::vector<pcl::PointIndices> cluster_indices,
		const int segment_index,
		Cloud &result);

	void cloud_cb(const CloudConstPtr &cloud);
	void cloud_cb_ref(const CloudConstPtr &cloud);

	void run();

	void setRefCloud(const RefCloudPtr &cloud);
	void matchRefCloudToDepthCloud(const CloudConstPtr &cloud);

	void setResultCloud(const RefCloudPtr &cloud);

	Eigen::Matrix4d getTrackingResultMat();

	//pcl::visualization::CloudViewer *viewer_;
	pcl::PointCloud<pcl::Normal>::Ptr normals_;
	CloudPtr cloud_pass_;
	CloudPtr cloud_pass_downsampled_;
	CloudPtr plane_cloud_;
	CloudPtr nonplane_cloud_;
	CloudPtr cloud_hull_;
	CloudPtr segmented_cloud_;
	CloudPtr reference_;
	std::vector<pcl::Vertices> hull_vertices_;

	boost::mutex mtx_;
	bool new_cloud_;
	pcl::NormalEstimationOMP<PointType, pcl::Normal> ne_; // to store threadpool
	//pcl::NormalEstimation<PointType, pcl::Normal> ne_; // to store threadpool
	boost::shared_ptr<ParticleFilter> tracker_;
	int counter_;
	bool use_convex_hull_;
	bool visualize_non_downsample_;
	bool visualize_particles_;
	double tracking_time_;
	double computation_time_;
	double downsampling_time_;
	double downsampling_grid_size_;

	CloudPtr m_refCloud;
	CloudPtr m_resultCloud;

	Eigen::Affine3f m_CoordTransMat;
};

template < typename PointType >
void DepthCloudTracker<PointType>::setResultCloud(const RefCloudPtr &cloud)
{
	m_resultCloud = cloud;
}


template < typename PointType >
Eigen::Matrix4d DepthCloudTracker<PointType>::getTrackingResultMat()
{
	Eigen::Matrix4d resultMat;

	ParticleXYZRPY result = tracker_->getResult();
	Eigen::Affine3f transformation = tracker_->toEigenMatrix(result);

	// first frame at CounterNum cannot track successfully 
	if (counter_ <= CounterNum + 1) 
	{
		resultMat = Eigen::Matrix4d::Identity();
	}

	else
	{
		resultMat = (transformation*m_CoordTransMat.inverse()).matrix().cast<double>();
	}

	// quantization to make tracking result stable
	if ((resultMat - Eigen::Matrix4d::Identity()).norm() < 1e-3)
	{
		resultMat = Eigen::Matrix4d::Identity();
	}

	return resultMat;
}

template < typename PointType >
void DepthCloudTracker<PointType>::matchRefCloudToDepthCloud(const CloudConstPtr &cloud)
{
	pcl::registration::CorrespondenceEstimation<PointType, RefPointType> corr_est;
	corr_est.setInputSource(m_refCloud);
	corr_est.setInputTarget(cloud);

	if (corr_est.requiresSourceNormals() && corr_est.requiresTargetNormals())
	{
		pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
		corr_est.determineCorrespondences(*correspondences, 0.05);

		pcl::ConstCloudIterator<RefPointType> source_it(*m_refCloud, *correspondences, true);
		pcl::ConstCloudIterator<PointType> target_it(*cloud, *correspondences, false);

		const int npts = static_cast <int> (source_it.size());

		for (int i = 0; i < npts; ++i)
		{
			m_refCloud->points[i] = *target_it;
			++target_it;
		}
	}
}

template < typename PointType >
void DepthCloudTracker<PointType>::setRefCloud(const RefCloudPtr &cloud)
{
	m_refCloud = cloud;
}

template < typename PointType >
DepthCloudTracker<PointType>::DepthCloudTracker()
{

}

template < typename PointType >
void DepthCloudTracker<PointType>::init(int thread_nr, double downsampling_grid_size,
	bool use_convex_hull,
	bool visualize_non_downsample, bool visualize_particles,
	bool use_fixed)
{
	//viewer_(viewer)
	new_cloud_ = false;
	ne_.setNumberOfThreads(thread_nr);

	counter_ = 0;
	use_convex_hull_ = use_convex_hull;
	visualize_non_downsample_ = visualize_non_downsample;
	visualize_particles_ = visualize_particles;
	downsampling_grid_size_ = downsampling_grid_size;

	KdTreePtr tree(new KdTree(false));
	ne_.setSearchMethod(tree);
	ne_.setRadiusSearch(0.03);

	std::vector<double> default_step_covariance = std::vector<double>(6, 0.015 * 0.015);
	default_step_covariance[3] *= 40.0;
	default_step_covariance[4] *= 40.0;
	default_step_covariance[5] *= 40.0;

	std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
	std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);
	if (use_fixed)
	{
		boost::shared_ptr<ParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
			(new ParticleFilterOMPTracker<RefPointType, ParticleT>(thread_nr));
		tracker_ = tracker;
	}
	else
	{
		boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
			(new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT>(thread_nr));
		tracker->setMaximumParticleNum(500);
		tracker->setDelta(0.99);
		tracker->setEpsilon(0.2);
		ParticleT bin_size;
		bin_size.x = 0.1f;
		bin_size.y = 0.1f;
		bin_size.z = 0.1f;
		bin_size.roll = 0.1f;
		bin_size.pitch = 0.1f;
		bin_size.yaw = 0.1f;
		tracker->setBinSize(bin_size);
		tracker_ = tracker;
	}

	tracker_->setTrans(Eigen::Affine3f::Identity());
	tracker_->setStepNoiseCovariance(default_step_covariance);
	tracker_->setInitialNoiseCovariance(initial_noise_covariance);
	tracker_->setInitialNoiseMean(default_initial_mean);
	tracker_->setIterationNum(1);

	tracker_->setParticleNum(400);
	tracker_->setResampleLikelihoodThr(0.00);
	tracker_->setUseNormal(false);
	// setup coherences
	ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
		(new ApproxNearestPairPointCloudCoherence<RefPointType>());
	// NearestPairPointCloudCoherence<RefPointType>::Ptr coherence = NearestPairPointCloudCoherence<RefPointType>::Ptr
	//   (new NearestPairPointCloudCoherence<RefPointType> ());

	boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
		= boost::shared_ptr<DistanceCoherence<RefPointType> >(new DistanceCoherence<RefPointType>());
	coherence->addPointCoherence(distance_coherence);

	boost::shared_ptr<HSVColorCoherence<RefPointType> > color_coherence
		= boost::shared_ptr<HSVColorCoherence<RefPointType> >(new HSVColorCoherence<RefPointType>());
	color_coherence->setWeight(0.1);
	coherence->addPointCoherence(color_coherence);

	//boost::shared_ptr<pcl::search::KdTree<RefPointType> > search (new pcl::search::KdTree<RefPointType> (false));
	boost::shared_ptr<pcl::search::Octree<RefPointType> > search(new pcl::search::Octree<RefPointType>(0.01));
	//boost::shared_ptr<pcl::search::OrganizedNeighbor<RefPointType> > search (new pcl::search::OrganizedNeighbor<RefPointType>);
	coherence->setSearchMethod(search);
	coherence->setMaximumDistance(0.01);
	tracker_->setCloudCoherence(coherence);
}

template < typename PointType >
bool DepthCloudTracker<PointType>::drawParticles(pcl::visualization::PCLVisualizer& viz)
{
	ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles();
	if (particles)
	{
		if (visualize_particles_)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
			for (size_t i = 0; i < particles->points.size(); i++)
			{
				pcl::PointXYZ point;

				point.x = particles->points[i].x;
				point.y = particles->points[i].y;
				point.z = particles->points[i].z;
				particle_cloud->points.push_back(point);
			}

		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(particle_cloud, 250, 99, 71);
			if (!viz.updatePointCloud(particle_cloud, red_color, "particle cloud"))
				viz.addPointCloud(particle_cloud, red_color, "particle cloud");
		}
		}
		return true;
	}
	else
	{
		PCL_WARN("no particles\n");
		return false;
	}
}

template < typename PointType >
void DepthCloudTracker<PointType>::drawResult(pcl::visualization::PCLVisualizer& viz)
{
	ParticleXYZRPY result = tracker_->getResult();
	Eigen::Affine3f transformation = tracker_->toEigenMatrix(result);
	// move a little bit for better visualization
	transformation.translation() += Eigen::Vector3f(0.0f, 0.0f, -0.005f);
	RefCloudPtr result_cloud(new RefCloud());

	if (!visualize_non_downsample_)
		pcl::transformPointCloud<RefPointType>(*(tracker_->getReferenceCloud()), *result_cloud, transformation);
	else
		pcl::transformPointCloud<RefPointType>(*reference_, *result_cloud, transformation);

	{
		pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color(result_cloud, 0, 0, 255);
		if (!viz.updatePointCloud(result_cloud, blue_color, "resultcloud"))
			viz.addPointCloud(result_cloud, blue_color, "resultcloud");
	}

	Eigen::Matrix4d transMat = transformation.matrix().cast<double>();
}

template < typename PointType >
void DepthCloudTracker<PointType>::drawOfflineResult(pcl::visualization::PCLVisualizer& viz)
{
	if (!viz.updatePointCloud(m_resultCloud, "resultcloud"))
		viz.addPointCloud(m_resultCloud, "resultcloud");
}

template < typename PointType >
void DepthCloudTracker<PointType>::drawRefCloud(pcl::visualization::PCLVisualizer& viz)
{
	if (m_refCloud)
	{
		pcl::visualization::PointCloudColorHandlerCustom<RefPointType> green_color(m_refCloud, 0, 250, 0);
		if (!viz.updatePointCloud(m_refCloud, green_color, "refcloud"))
			viz.addPointCloud(m_refCloud, green_color, "refcloud");
	}
}

template < typename PointType >
void DepthCloudTracker<PointType>::viz_cb(pcl::visualization::PCLVisualizer& viz)
{
	boost::mutex::scoped_lock lock(mtx_);

	if (!cloud_pass_)
	{
		boost::this_thread::sleep(boost::posix_time::seconds(1));
		return;
	}

	if (new_cloud_ && cloud_pass_downsampled_)
	{
		CloudPtr cloud_pass;
		if (!visualize_non_downsample_)
			cloud_pass = cloud_pass_downsampled_;
		else
			cloud_pass = cloud_pass_;

		if (!viz.updatePointCloud(cloud_pass, "cloudpass"))
		{
			viz.addPointCloud(cloud_pass, "cloudpass");
			//viz.resetCameraViewpoint("cloudpass");

			//drawRefCloud(viz);
		}
	}

	

	if (new_cloud_ && reference_)
	{
		bool ret = drawParticles(viz);
		if (ret)
		{
			drawResult(viz);

			// draw some texts
			viz.removeShape("N");
			viz.addText(QString("number of Reference PointClouds: %1").arg(tracker_->getReferenceCloud()->points.size()).toStdString(),
				10, 20, 20, 1.0, 1.0, 1.0, "N");

			viz.removeShape("M");
			viz.addText(QString("number of Measured PointClouds:  %1").arg(cloud_pass_downsampled_->points.size()).toStdString(),
				10, 40, 20, 1.0, 1.0, 1.0, "M");

			viz.removeShape("tracking");
			viz.addText(QString("tracking:        %1 fps").arg(1.0 / tracking_time_).toStdString(),
				10, 60, 20, 1.0, 1.0, 1.0, "tracking");

			viz.removeShape("downsampling");
			viz.addText(QString("downsampling:    %1 fps").arg(1.0 / downsampling_time_).toStdString(),
				10, 80, 20, 1.0, 1.0, 1.0, "downsampling");

			viz.removeShape("computation");
			viz.addText(QString("computation:     %1 fps").arg(1.0 / computation_time_).toStdString(),
				10, 100, 20, 1.0, 1.0, 1.0, "computation");

			viz.removeShape("particles");
			viz.addText(QString("particles:     %1").arg(tracker_->getParticles()->points.size()).toStdString(),
				10, 120, 20, 1.0, 1.0, 1.0, "particles");
		}
	}
	new_cloud_ = false;
}

template < typename PointType >
void DepthCloudTracker<PointType>::filterPassThrough(const CloudConstPtr &cloud, Cloud &result)
{
	FPS_CALC_BEGIN;
	pcl::PassThrough<PointType> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 10.0);
	//pass.setFilterLimits (0.0, 1.5);
	//pass.setFilterLimits (0.0, 0.6);
	pass.setKeepOrganized(false);
	pass.setInputCloud(cloud);
	pass.filter(result);
	FPS_CALC_END("filterPassThrough");
}

template < typename PointType >
void DepthCloudTracker<PointType>::euclideanSegment(const CloudConstPtr &cloud,
	std::vector<pcl::PointIndices> &cluster_indices)
{
	FPS_CALC_BEGIN;
	pcl::EuclideanClusterExtraction<PointType> ec;
	KdTreePtr tree(new KdTree());

	ec.setClusterTolerance(0.05); // 2cm
	ec.setMinClusterSize(50);
	ec.setMaxClusterSize(25000);
	//ec.setMaxClusterSize (400);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);
	FPS_CALC_END("euclideanSegmentation");
}

template < typename PointType >
void DepthCloudTracker<PointType>::gridSample(const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
{
	FPS_CALC_BEGIN;
	double start = pcl::getTime();
	pcl::VoxelGrid<PointType> grid;
	//pcl::ApproximateVoxelGrid<PointType> grid;
	grid.setLeafSize(float(leaf_size), float(leaf_size), float(leaf_size));
	grid.setInputCloud(cloud);
	grid.filter(result);
	//result = *cloud;
	double end = pcl::getTime();
	downsampling_time_ = end - start;
	FPS_CALC_END("gridSample");
}

template < typename PointType >
void DepthCloudTracker<PointType>::gridSampleApprox(const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
{
	FPS_CALC_BEGIN;
	double start = pcl::getTime();
	//pcl::VoxelGrid<PointType> grid;
	pcl::ApproximateVoxelGrid<PointType> grid;
	grid.setLeafSize(static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
	grid.setInputCloud(cloud);
	grid.filter(result);
	//result = *cloud;
	double end = pcl::getTime();
	downsampling_time_ = end - start;
	FPS_CALC_END("gridSample");
}

template < typename PointType >
void DepthCloudTracker<PointType>::planeSegmentation(const CloudConstPtr &cloud,
	pcl::ModelCoefficients &coefficients,
	pcl::PointIndices &inliers)
{
	FPS_CALC_BEGIN;
	pcl::SACSegmentation<PointType> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud);
	seg.segment(inliers, coefficients);
	FPS_CALC_END("planeSegmentation");
}

template < typename PointType >
void DepthCloudTracker<PointType>::planeProjection(const CloudConstPtr &cloud,
	Cloud &result,
	const pcl::ModelCoefficients::ConstPtr &coefficients)
{
	FPS_CALC_BEGIN;
	pcl::ProjectInliers<PointType> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(result);
	FPS_CALC_END("planeProjection");
}

template < typename PointType >
void DepthCloudTracker<PointType>::convexHull(const CloudConstPtr &cloud,
	Cloud &,
	std::vector<pcl::Vertices> &hull_vertices)
{
	FPS_CALC_BEGIN;
	pcl::ConvexHull<PointType> chull;
	chull.setInputCloud(cloud);
	chull.reconstruct(*cloud_hull_, hull_vertices);
	FPS_CALC_END("convexHull");
}

template < typename PointType >
void DepthCloudTracker<PointType>::normalEstimation(const CloudConstPtr &cloud,
	pcl::PointCloud<pcl::Normal> &result)
{
	FPS_CALC_BEGIN;
	ne_.setInputCloud(cloud);
	ne_.compute(result);
	FPS_CALC_END("normalEstimation");
}

template < typename PointType >
void DepthCloudTracker<PointType>::tracking(const RefCloudConstPtr &cloud)
{
	double start = pcl::getTime();
	FPS_CALC_BEGIN;
	tracker_->setInputCloud(cloud);
	tracker_->compute();
	double end = pcl::getTime();
	FPS_CALC_END("tracking");
	tracking_time_ = end - start;
}

template < typename PointType >
void DepthCloudTracker<PointType>::addNormalToCloud(const CloudConstPtr &cloud,
	const pcl::PointCloud<pcl::Normal>::ConstPtr &normals,
	RefCloud &result)
{
	result.width = cloud->width;
	result.height = cloud->height;
	result.is_dense = cloud->is_dense;
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		RefPointType point;
		point.x = cloud->points[i].x;
		point.y = cloud->points[i].y;
		point.z = cloud->points[i].z;
		point.rgba = cloud->points[i].rgba;
		// point.normal[0] = normals->points[i].normal[0];
		// point.normal[1] = normals->points[i].normal[1];
		// point.normal[2] = normals->points[i].normal[2];
		result.points.push_back(point);
	}
}

template < typename PointType >
void DepthCloudTracker<PointType>::extractNonPlanePoints(const CloudConstPtr &cloud,
	const CloudConstPtr &cloud_hull,
	Cloud &result)
{
	pcl::ExtractPolygonalPrismData<PointType> polygon_extract;
	pcl::PointIndices::Ptr inliers_polygon(new pcl::PointIndices());
	polygon_extract.setHeightLimits(0.01, 10.0);
	polygon_extract.setInputPlanarHull(cloud_hull);
	polygon_extract.setInputCloud(cloud);
	polygon_extract.segment(*inliers_polygon);
	{
		pcl::ExtractIndices<PointType> extract_positive;
		extract_positive.setNegative(false);
		extract_positive.setInputCloud(cloud);
		extract_positive.setIndices(inliers_polygon);
		extract_positive.filter(result);
	}
}

template < typename PointType >
void DepthCloudTracker<PointType>::removeZeroPoints(const CloudConstPtr &cloud,
	Cloud &result)
{
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		PointType point = cloud->points[i];
		if (!(fabs(point.x) < 0.01 &&
			fabs(point.y) < 0.01 &&
			fabs(point.z) < 0.01) &&
			!pcl_isnan(point.x) &&
			!pcl_isnan(point.y) &&
			!pcl_isnan(point.z))
			result.points.push_back(point);
	}

	result.width = static_cast<pcl::uint32_t> (result.points.size());
	result.height = 1;
	result.is_dense = true;
}

template < typename PointType >
void DepthCloudTracker<PointType>::extractSegmentCluster(const CloudConstPtr &cloud,
	const std::vector<pcl::PointIndices> cluster_indices,
	const int segment_index,
	Cloud &result)
{
	pcl::PointIndices segmented_indices = cluster_indices[segment_index];
	for (size_t i = 0; i < segmented_indices.indices.size(); i++)
	{
		PointType point = cloud->points[segmented_indices.indices[i]];
		result.points.push_back(point);
	}
	result.width = pcl::uint32_t(result.points.size());
	result.height = 1;
	result.is_dense = true;
}

template < typename PointType >
void DepthCloudTracker<PointType>::cloud_cb(const CloudConstPtr &cloud)
{
	boost::mutex::scoped_lock lock(mtx_);
	double start = pcl::getTime();
	FPS_CALC_BEGIN;
	cloud_pass_.reset(new Cloud);
	cloud_pass_downsampled_.reset(new Cloud);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	filterPassThrough(cloud, *cloud_pass_);
	if (counter_ < 10)
	{
		gridSample(cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
	}
	else if (counter_ == 10)
	{
		//gridSample (cloud_pass_, *cloud_pass_downsampled_, 0.01);
		cloud_pass_downsampled_ = cloud_pass_;
		CloudPtr target_cloud;
		if (use_convex_hull_)
		{
			planeSegmentation(cloud_pass_downsampled_, *coefficients, *inliers);
			if (inliers->indices.size() > 3)
			{
				CloudPtr cloud_projected(new Cloud);
				cloud_hull_.reset(new Cloud);
				nonplane_cloud_.reset(new Cloud);

				planeProjection(cloud_pass_downsampled_, *cloud_projected, coefficients);
				convexHull(cloud_projected, *cloud_hull_, hull_vertices_);

				extractNonPlanePoints(cloud_pass_downsampled_, cloud_hull_, *nonplane_cloud_);
				target_cloud = nonplane_cloud_;
			}
			else
			{
				PCL_WARN("cannot segment plane\n");
			}
		}
		else
		{
			PCL_WARN("without plane segmentation\n");
			target_cloud = cloud_pass_downsampled_;
		}

		if (target_cloud != NULL)
		{
			PCL_INFO("segmentation, please wait...\n");
			std::vector<pcl::PointIndices> cluster_indices;
			euclideanSegment(target_cloud, cluster_indices);
			if (cluster_indices.size() > 0)
			{
				// select the cluster to track
				CloudPtr temp_cloud(new Cloud);
				extractSegmentCluster(target_cloud, cluster_indices, 0, *temp_cloud);
				Eigen::Vector4f c;
				pcl::compute3DCentroid<RefPointType>(*temp_cloud, c);
				int segment_index = 0;
				double segment_distance = c[0] * c[0] + c[1] * c[1];
				for (size_t i = 1; i < cluster_indices.size(); i++)
				{
					temp_cloud.reset(new Cloud);
					extractSegmentCluster(target_cloud, cluster_indices, int(i), *temp_cloud);
					pcl::compute3DCentroid<RefPointType>(*temp_cloud, c);
					double distance = c[0] * c[0] + c[1] * c[1];
					if (distance < segment_distance)
					{
						segment_index = int(i);
						segment_distance = distance;
					}
				}

				segmented_cloud_.reset(new Cloud);
				extractSegmentCluster(target_cloud, cluster_indices, segment_index, *segmented_cloud_);
				//pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
				//normalEstimation (segmented_cloud_, *normals);
				RefCloudPtr ref_cloud(new RefCloud);
				//addNormalToCloud (segmented_cloud_, normals, *ref_cloud);
				ref_cloud = segmented_cloud_;
				RefCloudPtr nonzero_ref(new RefCloud);
				removeZeroPoints(ref_cloud, *nonzero_ref);

				PCL_INFO("calculating cog\n");

				RefCloudPtr transed_ref(new RefCloud);
				pcl::compute3DCentroid<RefPointType>(*nonzero_ref, c);
				m_CoordTransMat = Eigen::Affine3f::Identity();
				m_CoordTransMat.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);
				//pcl::transformPointCloudWithNormals<RefPointType> (*ref_cloud, *transed_ref, trans.inverse());
				pcl::transformPointCloud<RefPointType>(*nonzero_ref, *transed_ref, m_CoordTransMat.inverse());
				CloudPtr transed_ref_downsampled(new Cloud);
				gridSample(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
				tracker_->setReferenceCloud(transed_ref_downsampled);
				tracker_->setTrans(m_CoordTransMat);
				reference_ = transed_ref;
				tracker_->setMinIndices(int(ref_cloud->points.size()) / 2);
			}
			else
			{
				PCL_WARN("euclidean segmentation failed\n");
			}
		}
	}
	else
	{
		//normals_.reset (new pcl::PointCloud<pcl::Normal>);
		//normalEstimation (cloud_pass_downsampled_, *normals_);
		//RefCloudPtr tracking_cloud (new RefCloud ());
		//addNormalToCloud (cloud_pass_downsampled_, normals_, *tracking_cloud);
		//tracking_cloud = cloud_pass_downsampled_;

		//*cloud_pass_downsampled_ = *cloud_pass_;
		//cloud_pass_downsampled_ = cloud_pass_;
		gridSampleApprox(cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
		tracking(cloud_pass_downsampled_);
	}

	new_cloud_ = true;
	double end = pcl::getTime();
	computation_time_ = end - start;
	FPS_CALC_END("computation");
	counter_++;
}


template < typename PointType >
void DepthCloudTracker<PointType>::cloud_cb_ref(const CloudConstPtr &cloud)
{
	boost::mutex::scoped_lock lock(mtx_);
	double start = pcl::getTime();
	FPS_CALC_BEGIN;
	cloud_pass_.reset(new Cloud);
	cloud_pass_downsampled_.reset(new Cloud);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	filterPassThrough(cloud, *cloud_pass_);

	if (counter_ < CounterNum)
	{
		gridSample(cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
	}
	else if (counter_ == CounterNum)
	{
		//gridSample (cloud_pass_, *cloud_pass_downsampled_, 0.01);
		cloud_pass_downsampled_ = cloud_pass_;
		CloudPtr target_cloud;
		if (use_convex_hull_)
		{
			planeSegmentation(cloud_pass_downsampled_, *coefficients, *inliers);
			if (inliers->indices.size() > 3)
			{
				CloudPtr cloud_projected(new Cloud);
				cloud_hull_.reset(new Cloud);
				nonplane_cloud_.reset(new Cloud);

				planeProjection(cloud_pass_downsampled_, *cloud_projected, coefficients);
				convexHull(cloud_projected, *cloud_hull_, hull_vertices_);

				extractNonPlanePoints(cloud_pass_downsampled_, cloud_hull_, *nonplane_cloud_);
				target_cloud = nonplane_cloud_;
			}
			else
			{
				PCL_WARN("cannot segment plane\n");
			}
		}
		else
		{
			PCL_WARN("without plane segmentation\n");
			target_cloud = cloud_pass_downsampled_;
		}

		if (target_cloud != NULL)
		{
			RefCloudPtr nonzero_ref(new RefCloud);

			removeZeroPoints(m_refCloud, *nonzero_ref);

			PCL_INFO("calculating cog\n");

			Eigen::Vector4f c;
			RefCloudPtr transed_ref(new RefCloud);
			pcl::compute3DCentroid<RefPointType>(*nonzero_ref, c);
			m_CoordTransMat = Eigen::Affine3f::Identity();
			m_CoordTransMat.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);
			//pcl::transformPointCloudWithNormals<RefPointType> (*ref_cloud, *transed_ref, trans.inverse());
			pcl::transformPointCloud<RefPointType>(*nonzero_ref, *transed_ref, m_CoordTransMat.inverse());
			CloudPtr transed_ref_downsampled(new Cloud);
			gridSample(transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
			tracker_->setReferenceCloud(transed_ref_downsampled);
			tracker_->setTrans(m_CoordTransMat);
			reference_ = transed_ref;
			tracker_->setMinIndices(int(m_refCloud->points.size()) / 2);
		}
	}

	else
	{
		//normals_.reset (new pcl::PointCloud<pcl::Normal>);
		//normalEstimation (cloud_pass_downsampled_, *normals_);
		//RefCloudPtr tracking_cloud (new RefCloud ());
		//addNormalToCloud (cloud_pass_downsampled_, normals_, *tracking_cloud);
		//tracking_cloud = cloud_pass_downsampled_;

		//*cloud_pass_downsampled_ = *cloud_pass_;
		//cloud_pass_downsampled_ = cloud_pass_;
		gridSampleApprox(cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
		tracking(cloud_pass_downsampled_);
	}

	new_cloud_ = true;
	double end = pcl::getTime();
	computation_time_ = end - start;
	FPS_CALC_END("computation");
	counter_++;
}

template < typename PointType >
void DepthCloudTracker<PointType>::run()
{
	//viewer_->runOnVisualizationThread(boost::bind(&DepthCloudTracker<PointType>::viz_cb, this, _1), "viz_cb");
}