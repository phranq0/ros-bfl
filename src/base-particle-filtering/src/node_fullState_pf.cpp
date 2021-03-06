// Test node for basic particle filter estimation
// The model used here is the simple nonlinear unicycle, it's simulated using 
// Bfl library 
// This node simply runs the example within the ROS framework, writing on topics its parameters

// MOTION - Unicycle (x,y,theta)
// MEASUREMENTS - Emulate a GPS, takes noisy readings of the (x,y) position)

#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/GetMap.h"
//#include "map/map.h"
#include "customParticleFilter.h"

// Bayesian Filtering Library
#include <filter/bootstrapfilter.h>
#include <model/systemmodel.h>
#include <model/measurementmodel.h>

// Custom Bfl components
#include "mobile_robot_wall_cts.h"
#include "nonlinearSystemPdf.h"
#include "stateMeasurementPdf.h"
#include "mobile_robot.h"

// Angle conversions ecc...
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;

// For measurement noise emulation
double RandomNumber(double Min, double Max)
{
    return ((double(rand()) / double(RAND_MAX)) * (Max - Min)) + Min;
}

// ----------------------------- ROS node class

class ParticleFilterNode
{
    // Pubs and Subs 
    ros::NodeHandle nh_;
    ros::Subscriber navi_sub;
    ros::Subscriber meas_sub;
    ros::Publisher pose_pub;
    ros::Publisher particle_pub;
    ros::Publisher ground_truth_pub; 
    // Filter components
    NonlinearSystemPdf *sys_pdf;
    SystemModel<ColumnVector> *sys_model;
    stateMeasurementPdf *meas_pdf;
    MeasurementModel<ColumnVector,ColumnVector> *meas_model;
    // Particles
    MCPdf<ColumnVector> *prior_discr;
    // Filter instance
    CustomParticleFilter *filter;   // Custom for samples publishing
    // Timing
    ros::Time prevNavDataTime;
    double dt;

  public:
    // Constructor
    ParticleFilterNode()
    {
       //navi_sub = nh_.subscribe("/dumbTopic", 1, &ParticleFilterNode::InputCb, this);
       //ranges_sub = nh_.subscribe("/dumbTopic", 1, &ParticleFilterNode::MeasurementCb, this);
       pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/pose_pf",1);
       particle_pub = nh_.advertise<geometry_msgs::PoseArray>("/particle_cloud",1);
       ground_truth_pub = nh_.advertise<geometry_msgs::PoseStamped>("/robot_pose",1);
       dt = 0.0;
       // Init filter components
       sys_model = NULL;
       meas_model = NULL;
       filter = NULL;
       // Filter initialization
       CreateParticleFilter();
   }

   // Destructor
   ~ParticleFilterNode()
   {
      delete sys_model;
      delete meas_model;
      delete filter;
   }

   // ----------------------------------------------------- Initialization
   void CreateParticleFilter()
   {
      // -------------------------- Motion Model
      // Nonlinear model of unicycle robot (x,y,theta)

      // -------------- Gaussian noise for the motion model
      // Mean values
      ColumnVector sys_noise_Mu(STATE_SIZE);
      sys_noise_Mu(1) = MU_SYSTEM_NOISE_X;
      sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y;
      sys_noise_Mu(3) = MU_SYSTEM_NOISE_THETA;

      // Covariance matrix
      SymmetricMatrix sys_noise_Cov(STATE_SIZE);
      sys_noise_Cov = 0.0;
      sys_noise_Cov(1,1) = SIGMA_SYSTEM_NOISE_X;
      sys_noise_Cov(1,2) = 0.0;
      sys_noise_Cov(1,3) = 0.0;
      sys_noise_Cov(2,1) = 0.0;
      sys_noise_Cov(2,2) = SIGMA_SYSTEM_NOISE_Y;
      sys_noise_Cov(2,3) = 0.0;
      sys_noise_Cov(3,1) = 0.0;
      sys_noise_Cov(3,2) = 0.0;
      sys_noise_Cov(3,3) = SIGMA_SYSTEM_NOISE_THETA;

      // Create the gaussian distribution
      Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

      // Create the nonlinear motion model 
      // P(x(k+1)|x(k),u(k)) (in this case u is constant)
      sys_pdf = new NonlinearSystemPdf(system_Uncertainty);
      sys_model = new SystemModel<ColumnVector> (sys_pdf);

      // ----------------------------------------------------- Measurement Model

      // Construct the measurement noise
      ColumnVector meas_noise_Mu(MEAS_SIZE);
      meas_noise_Mu(1) = MU_MEAS_NOISE;
      meas_noise_Mu(1) = MU_MEAS_NOISE;
      SymmetricMatrix meas_noise_Cov(MEAS_SIZE);
      meas_noise_Cov(1,1) = SIGMA_MEAS_NOISE;
      meas_noise_Cov(2,2) = SIGMA_MEAS_NOISE;

      // Create the gaussian distribution
      Gaussian measurement_Uncertainty(meas_noise_Mu, meas_noise_Cov);

      // Create the measurement model 
      // P(z(k)|x(k)) 
      meas_pdf = new stateMeasurementPdf(measurement_Uncertainty);
      meas_model = new MeasurementModel<ColumnVector,ColumnVector>(meas_pdf);

      // ------------------------------------------------ Prior Density
      // Continuous Gaussian prior 
      ColumnVector prior_Mu(STATE_SIZE);
      prior_Mu(1) = PRIOR_MU_X;
      prior_Mu(2) = PRIOR_MU_Y;
      prior_Mu(3) = PRIOR_MU_THETA;
      SymmetricMatrix prior_Cov(STATE_SIZE);
      prior_Cov(1,1) = PRIOR_COV_X;
      prior_Cov(1,2) = 0.0;
      prior_Cov(1,3) = 0.0;
      prior_Cov(2,1) = 0.0;
      prior_Cov(2,2) = PRIOR_COV_Y;
      prior_Cov(2,3) = 0.0;
      prior_Cov(3,1) = 0.0;
      prior_Cov(3,2) = 0.0;
      prior_Cov(3,3) = PRIOR_COV_THETA;
      Gaussian prior_cont(prior_Mu,prior_Cov);

      // Discrete prior for Particle filter (using the continuous Gaussian prior)
      vector<Sample<ColumnVector>> prior_samples(NUM_SAMPLES);
      prior_discr = new MCPdf<ColumnVector>(NUM_SAMPLES,STATE_SIZE);
      prior_cont.SampleFrom(prior_samples,NUM_SAMPLES,CHOLESKY,NULL);
      // Particles
      prior_discr->ListOfSamplesSet(prior_samples);

      // ------------------------------ Instance of the filter
      filter = new CustomParticleFilter(prior_discr, 0.75, NUM_SAMPLES/4.0);
      
      // Start simulation loop
      RunSimulation();
    }
    // ----------------------------------------------------------------------

    // --------------------------------------------------- External Interface
    // For interfacing with motion commands and sensors

    // Motion callback
    void InputCb(std_msgs::Float64 msg)
    {
      // Here do input command processing, e.g. store inputs from the odometry
      // sensing after a motion step, to pass them to motion model
      // This should run:
      // filter->Update(sys_model,input);
    }

    // Measurement callback
    void MeasurementCb(std_msgs::Float64 msg)
    {
      // Here do measurement processing, like feature extraction from 
      // raw data, each time new data is received
      // This should run :
      // filter->Update(meas_model, measurement);
      // PublishParticles();
      // PublishPose();
    }

    // -----------------------------------------------------------------------

    // -------------------------------------------------- Simulation Interface
    void RunSimulation()
    {
        // Initialize mobile robot simulation
        MobileRobot mobile_robot;
        ColumnVector input(2);
        input(1) = 0.005;
        input(2) = 0.002;
        //input(1) = 0.0;
        //input(2) = 0.0;
        unsigned int useconds = 10000;  // For pause

        // Saving for plots
        std::ofstream file_robPos, file_robMeas, file_robEst, file_robCov;
        file_robPos.open ("robPos.csv");
        file_robMeas.open ("robMeas.csv");
        file_robEst.open ("robEst.csv");
        file_robCov.open ("robCov.csv");

        // ------------------------- Main Estimation Loop
        // Publish data inside loop
        cout << "Starting estimation" << endl;
        unsigned int time_step;
        for (time_step = 0; time_step < 800; time_step++)
          {
            // Move the simulated robot
            mobile_robot.Move(input);
            ColumnVector realState = mobile_robot.GetState();
            
            // Send real robot pose
            PublishGroundTruthPose(&mobile_robot);

            // Take a measurement (just use corrupted state, problems in using mobile_robot.measure())
            //ColumnVector measurement = mobile_robot.Measure();
            ColumnVector measNoise(3); measNoise(1) = RandomNumber(-0.2, 0.2);
            measNoise(2) = RandomNumber(-0.2, 0.2); measNoise(3) = RandomNumber(-0.2, 0.2);
            ColumnVector measurement = realState + measNoise;
            
            // ---------------- Update the filter 
            // Simulate missing measurements
            if(time_step > 350 && time_step < 550)
            {
                filter->Update(sys_model,input);
            } else {
                filter->Update(sys_model,input,meas_model,measurement);
            }
            
            // Save data for plots
            Pdf<ColumnVector> * posterior = filter->PostGet();
            ColumnVector pose = posterior->ExpectedValueGet();
            SymmetricMatrix poseCov = posterior->CovarianceGet();
            file_robEst << pose(1) << "," << pose(2) << "," << pose(3) << endl;
            file_robPos << realState(1) << "," << realState(2) << "," << realState(3) << endl;
            file_robMeas << measurement(1) << "," << measurement(2) << endl;
            file_robCov << poseCov(1,1) << "," << poseCov(2,2) << "," << poseCov(2,2) << endl;

            // Send estimated pose
            PublishPose();

            // Send particles
            PublishParticles();

            usleep(useconds); // 0.1 seconds sleep
          } 

          Pdf<ColumnVector> * posterior_final = filter->PostGet();
          cout << "After " << time_step+1 << " timesteps " << endl;
          cout << " Posterior Mean = " << endl << posterior_final->ExpectedValueGet() << endl
               << " Covariance = " << endl << posterior_final->CovarianceGet() << "" << endl;

          cout << "Finished simulation" << endl;

          file_robPos.close();
          file_robMeas.close();
          file_robEst.close();
          file_robCov.close();

          return;
    }
    // -----------------------------------------------------------------------

    // --------------------------------------------------------- Visualization 

    // Publishing particles over ROS
    void PublishParticles()
    {
      geometry_msgs::PoseArray particles_msg;
      particles_msg.header.stamp = ros::Time::now();
      particles_msg.header.frame_id = "/map";         // This should be defined 

      vector<WeightedSample<ColumnVector>>::iterator sample_it;
      vector<WeightedSample<ColumnVector>> samples;

      samples = filter->getNewSamples();    // Calling custom get method
      
      // Loading the message with each sample
      for(sample_it = samples.begin(); sample_it<samples.end(); sample_it++)
      {
        geometry_msgs::Pose pose;
        ColumnVector sample = (*sample_it).ValueGet();

        // Euler to Quaternion
        float roll = 0, pitch = 0, yaw = sample(3);
        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
        //cout << "Quaternion" << " " << q.coeffs() << endl;

        pose.position.x = sample(1);
        pose.position.y = sample(2);
        pose.position.z = 0;
        pose.orientation.x = q.coeffs()[0];
        pose.orientation.y = q.coeffs()[1];
        pose.orientation.z = q.coeffs()[2];
        pose.orientation.w = q.coeffs()[3];

        particles_msg.poses.insert(particles_msg.poses.begin(), pose);
      }
      // Sending the full message
      particle_pub.publish(particles_msg);
    }

    // Publishing estimated pose over ROS
    void PublishPose()
    {
        Pdf<ColumnVector> * posterior = filter->PostGet();
        ColumnVector pose = posterior->ExpectedValueGet();
        SymmetricMatrix pose_cov = posterior->CovarianceGet();

        // Euler to Quaternion
        float roll = 0, pitch = 0, yaw = pose(3);
        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
        //cout << "Quaternion" << " " << q.coeffs() << endl;

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "/map";

        pose_msg.pose.position.x = pose(1);
        pose_msg.pose.position.y = pose(2);
        pose_msg.pose.position.z = 0;
        pose_msg.pose.orientation.x = q.coeffs()[0];
        pose_msg.pose.orientation.y = q.coeffs()[1];
        pose_msg.pose.orientation.z = q.coeffs()[2];
        pose_msg.pose.orientation.w = q.coeffs()[3];

        pose_pub.publish(pose_msg);
    }

    // Publishing ground truth pose over ROS
    // Needed only for simulation
    void PublishGroundTruthPose(MobileRobot *mobile_robot)
    {
        geometry_msgs::PoseStamped real_pose_msg;
        real_pose_msg.header.stamp = ros::Time::now();
        real_pose_msg.header.frame_id = "/map";

        // Euler to Quaternion
        float roll = 0, pitch = 0, yaw = mobile_robot->GetState()[2];
        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
        //cout << "Quaternion" << " " << q.coeffs() << endl;

        real_pose_msg.pose.position.x = mobile_robot->GetState()[0];
        real_pose_msg.pose.position.y = mobile_robot->GetState()[1];
        real_pose_msg.pose.position.z = 0;
        real_pose_msg.pose.orientation.x = q.coeffs()[0];
        real_pose_msg.pose.orientation.y = q.coeffs()[1];
        real_pose_msg.pose.orientation.z = q.coeffs()[2];
        real_pose_msg.pose.orientation.w = q.coeffs()[3];

        ground_truth_pub.publish(real_pose_msg);
    }
    // -----------------------------------------------------------------------
};
    
    
int main(int argc, char** argv)
{
  cerr << "ROS - Basic Particle Filter testing" << endl;
  ros::init(argc, argv, "ParticleFilterNode");
  ParticleFilterNode pfNode;
  ros::spin();
  return 0;
}