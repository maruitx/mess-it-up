include($$[STARLAB])
include($$[SURFACEMESH])
include($$[CHOLMOD])
include($$[NANOFLANN])

StarlabTemplate(plugin)

 QMAKE_CXXFLAGS += -openmp -arch:AVX -D "_CRT_SECURE_NO_WARNINGS"
 QMAKE_CXXFLAGS_RELEASE *= -O2

FORMS += \
    mess_widget.ui \
	Kinect/RgbdViewerWidget.ui \
	Action/ActionLabelerWidget.ui \
	Action/ActionViewerWidget.ui
	
RESOURCES += scene_mess.qrc

HEADERS += \
    mess_widget.h \
	mess_mode.h \
	Geometry/Scene.h \
	Geometry/CModel.h \
	Geometry/SimplePointCloud.h \
	Geometry/UDGraph.h \
	Geometry/SceneRG.h \
	Geometry/QuickMeshDraw.h \
	Geometry/AABB.h \
	Geometry/OBB.h \
	Geometry/OBBEstimator.h \
	Geometry/OBBOBBIntersect.h \
	Geometry/BestFit.h \
	Geometry/ShapeLib.h \
	Geometry/TriTriIntersect.h \
	Geometry/SuppPlane.h\
	Geometry/SuppPlaneBuilder.h\
	Geometry/Skeleton.h\
	Geometry/SkeletonSampler.h\
	Geometry/Voxel.h\
	Geometry/Voxeler.h\
	Geometry/voxel_wed.h\
	Geometry/VoxelOctree.h\
	Geometry/BoundingBox.h\
	Geometry/font.inl\
	Kinect/RgbdViewerWidget.h \
	Kinect/RgbdViewer.h \
	Kinect/MS_KinectHelper.h \
	Kinect/MS_OpenCVFrameHelper.h \
	Kinect/OpenCVHelper.h \
	Kinect/DepthCloud.h \
	Kinect/DepthCloudTracker.h \
	Kinect/DepthSensor.h \
	Kinect/FrameRateTracker.h \
	Kinect/kinect_grabber.h \
	Action/ActionLearner.h \
	Action/ActionLabelerWidget.h \
	Action/ActionLabeler.h \
	Action/ActionFeature.h \
	Action/ActionViewerWidget.h \
	Action/ActionViewer.h \
	Utilities/utility.h \
	Utilities/CustomDrawObjects.h \
	Math/MatrixType.h \
	Math/mathlib.h \
	Math/Eigen3x3.h 

SOURCES += \
    mess_widget.cpp \
	mess_mode.cpp \
	Geometry/Scene.cpp \
	Geometry/CModel.cpp \
	Geometry/SimplePointCloud.cpp \
	Geometry/UDGraph.cpp \
	Geometry/SceneRG.cpp \
	Geometry/AABB.cpp \
	Geometry/OBB.cpp \
	Geometry/OBBEstimator.cpp \
	Geometry/OBBOBBIntersect.cpp \
	Geometry/BestFit.cpp \
	Geometry/TriTriIntersect.cpp \
	Geometry/SuppPlane.cpp\
	Geometry/SuppPlaneBuilder.cpp\
	Geometry/Skeleton.cpp\
	Geometry/SkeletonSampler.cpp\
	Geometry/Voxeler.cpp\
	Geometry/VoxelOctree.cpp\
	Geometry/BoundingBox.cpp\	
	Kinect/RgbdViewerWidget.cpp \
	Kinect/RgbdViewer.cpp \
	Kinect/MS_OpenCVFrameHelper.cpp \
	Kinect/OpenCVHelper.cpp \
	Kinect/DepthCloud.cpp \
	Kinect/DepthSensor.cpp \
	Kinect/FrameRateTracker.cpp \
	Action/ActionLearner.cpp \
	Action/ActionLabelerWidget.cpp \
	Action/ActionLabeler.cpp \
	Action/ActionFeature.cpp \
	Action/ActionViewerWidget.cpp \
	Action/ActionViewer.cpp \
	Math/mathlib.cpp 


# OpenCV
	win32{
    INCLUDEPATH *= C:\Graphics\opencv\opencv-3.0.0\opencv\build\install\include
    LIBS *= -L"C:\Graphics\opencv\opencv-3.0.0\opencv\build\install\x86\vc12\lib"
    OpenCV_VERSION = 300
    Debug:LIBS *= -lopencv_core$${OpenCV_VERSION}d -lopencv_highgui$${OpenCV_VERSION}d -lopencv_features2d$${OpenCV_VERSION}d  -lopencv_flann$${OpenCV_VERSION}d -lopencv_imgproc$${OpenCV_VERSION}d -lopencv_video$${OpenCV_VERSION}d -lopencv_imgcodecs$${OpenCV_VERSION}d -lopencv_videoio$${OpenCV_VERSION}d -lopencv_surface_matching$${OpenCV_VERSION}d
    Release:LIBS *= -lopencv_core$${OpenCV_VERSION} -lopencv_highgui$${OpenCV_VERSION} -lopencv_features2d$${OpenCV_VERSION}  -lopencv_flann$${OpenCV_VERSION} -lopencv_imgproc$${OpenCV_VERSION} -lopencv_video$${OpenCV_VERSION} -lopencv_imgcodecs$${OpenCV_VERSION} -lopencv_videoio$${OpenCV_VERSION} -lopencv_surface_matching$${OpenCV_VERSION}
}

# Kinect SDK
INCLUDEPATH *= $(KINECTSDK10_DIR)\inc
win32:LIBS *= -L"$(KINECTSDK10_DIR)\lib\x86"
LIBS *= -lKinect10

# CGAL and Boost
win32{
	CGAL_DIR = $$(CGAL_DIR)
	DEFINES += CGAL_CFG_NO_CPP0X_VARIADIC_TEMPLATES
	DEFINES += CGAL_CFG_NO_NEXTAFTER
	DEFINES += CGAL_CFG_NO_TR1_ARRAY
	DEFINES += CGAL_CFG_NO_TR1_TUPLE
	INCLUDEPATH *= $(CGAL_DIR)\include
	INCLUDEPATH *= $(CGAL_DIR)\build\include
	INCLUDEPATH *= $(CGAL_DIR)\auxiliary\gmp\include
	LIBS *= -L""$$CGAL_DIR"/lib"
	LIBS *= -lCGAL-vc120-mt-gd-4.5 -lCGAL-vc120-mt-4.5 -lCGAL_Core-vc120-mt-gd-4.5 -lCGAL_Core-vc120-mt-4.5
	LIBS *= -L""$$CGAL_DIR"/auxiliary/gmp/lib"
	LIBS *= -llibgmp-10
	INCLUDEPATH *= C:\Graphics\boost_1_57_0
	LIBS *= -L"C:\Graphics\boost_1_57_0\lib32-msvc-12.0"
}

# PCL
INCLUDEPATH += C:\Graphics\pcl\PCL_install\include\pcl-1.8
LIBS += -LC:/Graphics/pcl/PCL_install/lib
Debug: LIBS += -lpcl_visualization_debug -lpcl_tracking_debug  -lpcl_common_debug -lpcl_octree_debug -lpcl_search_debug -lpcl_features_debug -lpcl_filters_debug -lpcl_surface_debug -lpcl_kdtree_debug -lpcl_segmentation_debug -lpcl_keypoints_debug -lpcl_sample_consensus_debug -lpcl_registration_debug
Release: LIBS += -lpcl_visualization_release -lpcl_tracking_release -lpcl_common_release -lpcl_octree_release -lpcl_search_release -lpcl_features_release -lpcl_filters_release -lpcl_surface_release -lpcl_kdtree_release -lpcl_segmentation_release -lpcl_keypoints_release -lpcl_sample_consensus_release -lpcl_registration_release
INCLUDEPATH += C:\Graphics\flann-1.8.4-src\flann-1.8.4-src\src\cpp
Debug:LIBS += -LC:/Graphics/flann-1.8.4-src/flann-1.8.4-src/build/lib/Debug -lflann_cpp_s -lflann
Release: LIBS += -LC:/Graphics/flann-1.8.4-src/flann-1.8.4-src/build/lib/Release -lflann_cpp_s -lflann
INCLUDEPATH += C:\Graphics\VTK-6.1.0\VTK_install\include\vtk-6.1
LIBS += -LC:/Graphics/VTK-6.1.0/VTK_install/lib -lvtkalglib-6.1 -lvtkChartsCore-6.1 -lvtkCommonColor-6.1 -lvtkCommonComputationalGeometry-6.1 -lvtkCommonCore-6.1 -lvtkCommonDataModel-6.1 -lvtkCommonExecutionModel-6.1 -lvtkCommonMath-6.1 -lvtkCommonMisc-6.1 -lvtkCommonSystem-6.1 -lvtkCommonTransforms-6.1 -lvtkDICOMParser-6.1 -lvtkDomainsChemistry-6.1 -lvtkexoIIc-6.1 -lvtkexpat-6.1 -lvtkFiltersAMR-6.1 -lvtkFiltersCore-6.1 -lvtkFiltersExtraction-6.1 -lvtkFiltersFlowPaths-6.1 -lvtkFiltersGeneral-6.1 -lvtkFiltersGeneric-6.1 -lvtkFiltersGeometry-6.1 -lvtkFiltersHybrid-6.1 -lvtkFiltersHyperTree-6.1 -lvtkFiltersImaging-6.1 -lvtkFiltersModeling-6.1 -lvtkFiltersParallel-6.1 -lvtkFiltersParallelImaging-6.1 -lvtkFiltersProgrammable-6.1 -lvtkFiltersSelection-6.1 -lvtkFiltersSMP-6.1 -lvtkFiltersSources-6.1 -lvtkFiltersStatistics-6.1 -lvtkFiltersTexture-6.1 -lvtkFiltersVerdict-6.1 -lvtkfreetype-6.1 -lvtkftgl-6.1 -lvtkGeovisCore-6.1 -lvtkgl2ps-6.1 -lvtkhdf5-6.1 -lvtkhdf5_hl-6.1 -lvtkImagingColor-6.1 -lvtkImagingCore-6.1 -lvtkImagingFourier-6.1 -lvtkImagingGeneral-6.1 -lvtkImagingHybrid-6.1 -lvtkImagingMath-6.1 -lvtkImagingMorphological-6.1 -lvtkImagingSources-6.1 -lvtkImagingStatistics-6.1 -lvtkImagingStencil-6.1 -lvtkInfovisCore-6.1 -lvtkInfovisLayout-6.1 -lvtkInteractionImage-6.1 -lvtkInteractionStyle-6.1 -lvtkInteractionWidgets-6.1 -lvtkIOAMR-6.1 -lvtkIOCore-6.1 -lvtkIOEnSight-6.1 -lvtkIOExodus-6.1 -lvtkIOExport-6.1 -lvtkIOGeometry-6.1 -lvtkIOImage-6.1 -lvtkIOImport-6.1 -lvtkIOInfovis-6.1 -lvtkIOLegacy-6.1 -lvtkIOLSDyna-6.1 -lvtkIOMINC-6.1 -lvtkIOMovie-6.1 -lvtkIONetCDF-6.1 -lvtkIOParallel-6.1 -lvtkIOPLY-6.1 -lvtkIOSQL-6.1 -lvtkIOVideo-6.1 -lvtkIOXML-6.1 -lvtkIOXMLParser-6.1 -lvtkjpeg-6.1 -lvtkjsoncpp-6.1 -lvtklibxml2-6.1 -lvtkmetaio-6.1 -lvtkNetCDF-6.1 -lvtkNetCDF_cxx-6.1 -lvtkoggtheora-6.1 -lvtkParallelCore-6.1 -lvtkpng-6.1 -lvtkproj4-6.1 -lvtkRenderingAnnotation-6.1 -lvtkRenderingContext2D-6.1 -lvtkRenderingCore-6.1 -lvtkRenderingFreeType-6.1 -lvtkRenderingFreeTypeOpenGL-6.1 -lvtkRenderingGL2PS-6.1 -lvtkRenderingImage-6.1 -lvtkRenderingLabel-6.1 -lvtkRenderingLIC-6.1 -lvtkRenderingLOD-6.1 -lvtkRenderingOpenGL-6.1 -lvtkRenderingVolume-6.1 -lvtkRenderingVolumeAMR-6.1 -lvtkRenderingVolumeOpenGL-6.1 -lvtksqlite-6.1 -lvtksys-6.1 -lvtktiff-6.1 -lvtkverdict-6.1 -lvtkViewsContext2D-6.1 -lvtkViewsCore-6.1 -lvtkViewsGeovis-6.1 -lvtkViewsInfovis-6.1 -lvtkzlib-6.1


