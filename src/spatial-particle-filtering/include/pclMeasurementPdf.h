// Header file which defines conditional probability density function (measurement model)
// related to a point-shaped mobile robot defined as P(z(k)|x(k))
// This relies on a map given as point cloud, the confrontation is done 
// between measured distance from nearest neighbor and predicted distance from
// each particle to its nearest neighbor

#ifndef __PCL_MEAS_MOBILE__
#define __PCL_MEAS_MOBILE__

#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace BFL
{
    // Non linear conditional gaussian, inherited from general conditional pdf
    class pclMeasurementPdf : public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
        {
            public:
                pcl::PointCloud<pcl::PointXYZ>::Ptr _map;
                // Constructor
                pclMeasurementPdf(const Gaussian& measNoise, const PointCloud::ConstPtr& map);

                // Destructor 
                virtual ~pclMeasurementPdf();

                // This gets the measurement model, the P(z(k)|x(k))
                virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

            private:
                Gaussian _measNoise;

        };
}

#endif