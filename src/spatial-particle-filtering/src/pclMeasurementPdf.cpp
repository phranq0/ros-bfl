// Definition of Measurement Model
// Makes a confrontation with a map expressed as point cloud

#include "pclMeasurementPdf.h"
#include <wrappers/rng/rng.h>          // Random number generator
#include <cmath>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#define MEASMODEL_NUMCONDARGUMENTS_MOBILE 1       // Sensor array size
#define MEASMODEL_DIMENSION_MOBILE 3            // Measured state size

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace BFL
{
    using namespace MatrixWrapper;

    // The constructor gets its own copy of the map and stores it
    pclMeasurementPdf::pclMeasurementPdf(const Gaussian& measNoise, const PointCloud::ConstPtr& map)
        : ConditionalPdf<ColumnVector,ColumnVector>(MEASMODEL_DIMENSION_MOBILE,MEASMODEL_NUMCONDARGUMENTS_MOBILE)
    {
        _map = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        _measNoise = measNoise;
        _map->width = map->width;
        _map->height = map->height;
        _map->points = map->points;
        // DEBUG (check if the map is stored correctly)
        cerr << "Map filter - size: " << endl;
        cerr << map->width << endl;
        cerr << map->height << endl;
        cerr << "Map filter - points: " << endl;
        for (pcl::PointXYZ pt : _map->points)
        {
            cerr << "Map filter - points: " << pt.x << " " << pt.y << " " << pt.z << endl;
        }
    }

    pclMeasurementPdf::~pclMeasurementPdf(){};

    // Get conditional probability, this is called for each particle when running an update with measurements
    // The call is implicit, it's handled by the 'Update' method of the filter
    Probability pclMeasurementPdf::ProbabilityGet(const ColumnVector& measurement) const
    {
        // Here z(k) is only conditioned by x(k), not u(k) like in more general model
        ColumnVector state = ConditionalArgumentGet(0);

        // ------------------------ Predicted measurement
        // Compute nearest neighbor on map and its distance
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(_map);
        pcl::PointXYZ target;
        target.x = state(1);
        target.y = state(2);
        target.z = state(3);
        
        // K nearest neighbors search (just one nn to find)
        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        ColumnVector nn_point(3);
        ColumnVector expected_measurement(3);
        if ( kdtree.nearestKSearch (target, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
          // For now measurement is just the distance between point
          for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            {
                //exp_meas(i+1) = pointNKNSquaredDistance[i];
                nn_point(1) = (*_map)[ pointIdxNKNSearch[i] ].x;
                nn_point(2) = (*_map)[ pointIdxNKNSearch[i] ].y;
                nn_point(3) = (*_map)[ pointIdxNKNSearch[i] ].z;
            }
        }
//
        ColumnVector lin_state(3); lin_state(1) = state(1); lin_state(2) = state(2); lin_state(3) = state(3);
        expected_measurement = lin_state - nn_point;
//
        // Compute expected measurement for the current particle
        ColumnVector distance(3);

        // Compute distance between simulated and real measurements
        distance = expected_measurement - measurement;
        cerr << "Predicted Measurement: " << distance << endl;
        // ----------------------------------------- DEBUG -------------------------------------------------
        // OK
        // ColumnVector contact_point(3); contact_point(1) = 0.1; contact_point(2) = 0.15; contact_point(3) = 0.18;
        // ColumnVector nn_point(3); nn_point(1) = 0.4; nn_point(2) = 0.3; nn_point(3) = 0.22; 
        // ColumnVector real_meas(3); 
        // real_meas(1) = abs(contact_point(1) - nn_point(1)); 
        // real_meas(2) = abs(contact_point(2) - nn_point(2));  
        // real_meas(3) = abs(contact_point(3) - nn_point(3));
        // ColumnVector expected_meas(3); expected_meas(1) = state(1); expected_meas(2) = state(2); expected_meas(3) = state(3); 
        // ColumnVector distance(3);
        // distance = expected_meas - real_meas;
        // Scalar distance 
        //ColumnVector real_meas(1);  real_meas = 0.07;
        //ColumnVector expected_meas(1); expected_meas = state(3);
        //ColumnVector distance(1);
        //distance = expected_meas - real_meas;

        // -------------------------------------------------------------------------------------------------
        
        // Computes the likelihood of the measurement by sampling
        // the probability of (em-m) in the noise distribution
        // The weight of the particle will be proportional to this distance
        return _measNoise.ProbabilityGet(distance);
    }
}