#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <std_msgs/Float32.h>

using namespace std;

  
class StateEstimation {
  ros::Subscriber cloudSubs;
  ros::Subscriber ballPoseSubs;
  ros::NodeHandle nh;

  tf::TransformListener tfListener;

  bool isAdded;
  pcl::visualization::PCLVisualizer* viewer;
	
	int state ;
	vector< vector<float> > ball_state;  //[time] //x y xdot ydot 

  vector<float> new_state ;  //x y xdot ydot 4x1
	vector< vector<float> > Ball_cov_diag ; //P 4x4
	vector< vector<float> > F_dyn ; //F 4x4
	vector< vector<float> > F_dyn_t ; //F_transpose 4x4
	vector< vector<float> > temp ; //temp 4x4
	vector< vector<float> > H ; //temp 2x4
	vector< vector<float> > H_t ; //H_t 4x2
  
	vector<float>  R_diag ; //R 1_1 2_2
	vector< vector<float> >  S ; //S 2x2
	vector< vector<float> >  Kalman_gain ; //W 4x2

  vector<float> measurement_error ; //v 2x1
	
    ros::Publisher estimatedPub_x;
    ros::Publisher measuredPub_x;
    ros::Publisher realPub_x;
    ros::Publisher estimatedPub_y;
    ros::Publisher measuredPub_y;
    ros::Publisher realPub_y;
    std_msgs::Float32 data_msg;

 public:
  StateEstimation() {//initalize matrix variables
		state = 0 ;
		new_state.push_back(0); //x
    new_state.push_back(0);//y
		//init 2 column matrix
		S.push_back(new_state);//1st row
	  S.push_back(new_state);//2nd row	

    Kalman_gain.push_back(new_state);//1st row
	  Kalman_gain.push_back(new_state);//2nd row
	  Kalman_gain.push_back(new_state);//3rd row
	  Kalman_gain.push_back(new_state);//4th row

		H_t.push_back(new_state);H_t[0][0] = 1;//1st row 
	  H_t.push_back(new_state);H_t[1][1] = 1;//2nd row
		H_t.push_back(new_state);//3rd row 
	  H_t.push_back(new_state);//4th row

    new_state.push_back(0);//xdot
    new_state.push_back(0);//ydot
		//init 4 column matrix
	  Ball_cov_diag.push_back(new_state);//1st row
	  Ball_cov_diag.push_back(new_state);//2nd row
	  Ball_cov_diag.push_back(new_state);//3rd row
	  Ball_cov_diag.push_back(new_state);//4th row

		H.push_back(new_state);H[0][0] = 1;//1st row 
	  H.push_back(new_state);H[1][1] = 1;//2nd row

	  temp.push_back(new_state);//1st row
	  temp.push_back(new_state);//2nd row
	  temp.push_back(new_state);//3rd row
	  temp.push_back(new_state);//4th row

	  F_dyn.push_back(new_state);F_dyn[0][0] = 1;F_dyn[0][2] = 1;//1st row
	  F_dyn.push_back(new_state);F_dyn[1][1] = 1;F_dyn[1][3] = 1;//2nd row
	  F_dyn.push_back(new_state);F_dyn[2][2] = 1;//3rd row
	  F_dyn.push_back(new_state);F_dyn[3][3] = 1;//4th row

	  F_dyn_t.push_back(new_state);F_dyn_t[0][0] = 1;//1st row
	  F_dyn_t.push_back(new_state);F_dyn_t[1][1] = 1;//2nd row
	  F_dyn_t.push_back(new_state);F_dyn_t[2][0] = 1;F_dyn_t[2][2] = 1;//3rd row
	  F_dyn_t.push_back(new_state);F_dyn_t[3][1] = 1;F_dyn_t[3][3] = 1;//4th row
		
		Ball_cov_diag[0][0] = 1 ;Ball_cov_diag[1][1] = 1 ;Ball_cov_diag[2][2] = 1 ;Ball_cov_diag[3][3] = 1 ;

	  R_diag.push_back(0.1);//1_1
	  R_diag.push_back(0.1);//2_2

	  ball_state.push_back(new_state);
		
	  measurement_error.push_back(0.0);//v1
	  measurement_error.push_back(0.0);//v2
		
    cloudSubs = nh.subscribe("/vrep/kinectPoints", 1,
                             &StateEstimation::pointCloudCallback, this);

    ballPoseSubs = nh.subscribe("/vrep/ballPose", 1,
                                &StateEstimation::ballPoseCallback, this);
		estimatedPub_x =nh.advertise<std_msgs::Float32> ("/estimated_X",1);   
		measuredPub_x =nh.advertise<std_msgs::Float32> ("/measured_X",1);   
		realPub_x =nh.advertise<std_msgs::Float32> ("/real_X",1);  

	  estimatedPub_y =nh.advertise<std_msgs::Float32> ("/estimated_Y",1);   
		measuredPub_y =nh.advertise<std_msgs::Float32> ("/measure_Y",1);   
		realPub_y =nh.advertise<std_msgs::Float32> ("/real_Y",1);  

    isAdded = false;

    viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
    viewer->initCameraParameters();
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
  }
		vector< vector<float> >  matrix_product(vector< vector<float> > A, vector< vector<float> > b , int row , int column, int common){
				vector< vector<float> >  M ;
				float sum = 0 ;
				vector<float>  l ; 
				for(int i = 0 ;i < row ; i++ ) {
						l.clear();
						for(int j = 0 ;j < column ; j++ ) {
							sum = 0 ;
							for(int w = 0 ;w < common ; w++ ) {
							sum = sum + A[i][w] * b[w][j] ;
							}
							l.push_back(sum);
					  }
						M.push_back(l);
				}		
			return M ;
			}

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // wait from transform
    tfListener.waitForTransform("base_link", "kinect_depth", ros::Time::now(),
                                ros::Duration(1.0));

    // convert point cloud frame to robot frame
    sensor_msgs::PointCloud2 transformedPoints;
    pcl_ros::transformPointCloud("base_link", *msg, transformedPoints,
                                 tfListener);
    //pcl::PointCloud<pcl::PointXYZ> *pointCloud = new pcl::PointCloud<pcl::PointXYZ>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(transformedPoints, *pointCloud);
	
    std::cout << "size of pcl= " << pointCloud->points.size () << std::endl;
    // DO: detect ball here

	//plane detection

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.1);

	seg.setInputCloud (pointCloud);
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
	PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	}

	std::cout << "Model coefficients: " << coefficients->values[0] << " " 
		                                << coefficients->values[1] << " "
		                              	<< coefficients->values[2] << " " 
		                             	<< coefficients->values[3] << std::endl;

  	std::cout << "Model inliers: " << inliers->indices.size () << std::endl;

	//for (size_t i = 0; i < inliers->indices.size (); ++i)
    //    inliers->indices[i] << "    " << pointCloud->points[inliers->indices[i]].x << " "
    //                                  << pointCloud->points[inliers->indices[i]].y << " "
    //                                  << pointCloud->points[inliers->indices[i]].z << std::endl;
   
 	//remove indices of plane
 	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(pointCloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*pointCloud);

	std::cout << "size after plane removal = " << pointCloud->points.size () << std::endl;
	
    //plane removed view	
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr constPointCloud(pointCloud);
 
    viewer->removePointCloud("cloud");
    viewer->addPointCloud(constPointCloud, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
      
    viewer->spinOnce(100);

	//limit x axis to detect only ball
	bool balloccluded = true ; 
	int cloudsize= 0 ; 
	for (int i = 0; i < pointCloud->points.size (); i++) {
		if( pointCloud->points[i].x < 3.8  and pointCloud->points[i].x > 1.6){
			  cloudsize++ ;   
			  balloccluded = false ;
			}
   	}

	//create new cloud data for ball
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data
  	cloud->width  = cloudsize;
  	cloud->height = 1;
  	cloud->points.resize (cloud->width * cloud->height);

	cloudsize= 0 ; 
	float measured_x = 0;
	float measured_y = 0;

	for (int i = 0; i < pointCloud->points.size (); i++) {
		if( pointCloud->points[i].x < 3.8 and pointCloud->points[i].x > 1.6){
			  cloud->points[cloudsize].x = pointCloud->points[i].x ;
			  measured_x = measured_x + pointCloud->points[i].x;  
    		  cloud->points[cloudsize].y = pointCloud->points[i].y ;
			  measured_y = measured_y + pointCloud->points[i].y ;
   			  cloud->points[cloudsize].z = pointCloud->points[i].z ;
			  cloudsize++ ; 

    		  // print point values for debugging purposes
      		  std::cout << "p[" << i << "].x=" << pointCloud->points[i].x << " .y="
        									   << pointCloud->points[i].y << " .z=" 
											   << pointCloud->points[i].z << std::endl;
			}
    }

	
   if(cloudsize > 0){//turn measured data to world frame
   measured_x = (measured_x /  cloudsize)-1.4 ; 
   measured_y = (measured_y /  cloudsize)+0.25 ; }

   data_msg.data = measured_x;
   measuredPub_x.publish(data_msg); 

   data_msg.data = measured_y;
   measuredPub_y.publish(data_msg); 

   std::cout << "size after limit=" << cloud->points.size () << std::endl;
   std::cout << "Measured x:" <<  measured_x << "y:" << measured_y << std::endl;
 
   // DO: apply kalman filter here 

	//time update (prediction)
	new_state[0] = new_state[0] + new_state[2] ; 
	new_state[1] = new_state[1] + new_state[3] ; 
	new_state[2] = new_state[2] ; 
	new_state[3] = new_state[3] ; 

	std::cout << "Predicted x:" <<  new_state[0] << "y:" << new_state[1] << std::endl;

	data_msg.data =  new_state[0]; 
	estimatedPub_x.publish(data_msg);

	data_msg.data =  new_state[1]; 
	estimatedPub_y.publish(data_msg);


	if(!balloccluded){//we can see the ball
		
			Ball_cov_diag  = matrix_product(Ball_cov_diag, F_dyn_t , 4, 4, 4 ) ; 
			Ball_cov_diag  = matrix_product(F_dyn, Ball_cov_diag , 4, 4, 4 ) ; 
	    
			//compute kalman gain
			Kalman_gain = matrix_product(Ball_cov_diag, H_t , 4 , 2, 4 );//
			S = matrix_product(H, Kalman_gain , 2 , 2, 4 );
			S[0][0] = S[0][0] + R_diag[0] ; 
			S[1][1] = S[1][1] + R_diag[1] ;

			Kalman_gain = matrix_product(Ball_cov_diag ,H_t , 4 , 2 , 4) ; 
			//inverse S
			float reverse_cons = (S[0][0] * S[1][1] - S[1][0] * S[0][1]) ; 
			float temp_S = S[0][0] ;
			S[0][0] = S[1][1] /  reverse_cons ; 
			S[1][1] = temp_S /  reverse_cons ; 
			S[0][1] = (-1 * S[0][1]) /  reverse_cons ; 
			S[1][0] = (-1 * S[1][0]) /  reverse_cons ; 

			Kalman_gain = matrix_product(Kalman_gain , S , 4 , 2 , 2);

      //measurement update(corrected)
			measurement_error[0] =  measured_x - new_state[0] ;
			measurement_error[1] =  measured_y - new_state[1] ; 

			new_state[0] += measurement_error[0] * Kalman_gain[0][0] + measurement_error[1] * Kalman_gain[0][1] ;
		    new_state[1] += measurement_error[0] * Kalman_gain[1][0] + measurement_error[1] * Kalman_gain[1][1] ;
		    new_state[2] += measurement_error[0] * Kalman_gain[2][0] + measurement_error[1] * Kalman_gain[2][1] ;
		    new_state[3] += measurement_error[0] * Kalman_gain[3][0] + measurement_error[1] * Kalman_gain[3][1] ;

			temp = matrix_product(Kalman_gain , H , 4 , 4 , 2);//
			temp[0][0] = 1 - temp[0][0] ;temp[0][1] =-temp[0][1];temp[0][2] =-temp[0][2];temp[0][3] =-temp[0][3]; 
			temp[1][1] = 1 - temp[1][1] ;temp[1][0] =-temp[1][0];temp[1][2] =-temp[1][2];temp[1][3] =-temp[1][3];
			temp[2][2] = 1 - temp[2][2] ;temp[2][0] =-temp[2][0];temp[2][1] =-temp[2][1];temp[2][3] =-temp[2][3];
			temp[3][3] = 1 - temp[3][3] ;temp[3][0] =-temp[3][0];temp[3][1] =-temp[3][1];temp[3][2] =-temp[3][2];

			Ball_cov_diag = matrix_product(temp,Ball_cov_diag,4,4,4); 

 			std::cout << "Predict corrected x:" <<  new_state[0] << "y:" << new_state[1] << std::endl; 

	}else{//we cant see the ball
} 

  }

  // get the ground truth position of the ball for debugging and plotting
  void ballPoseCallback(const geometry_msgs::PoseStamped& ballPose) {
    std::cout << "real ball.x=" << ballPose.pose.position.x << " ball.y="
              << ballPose.pose.position.y << std::endl;
		data_msg.data = ballPose.pose.position.x ;
		realPub_x.publish(data_msg)  ; 

		data_msg.data = ballPose.pose.position.y ;
		realPub_y.publish(data_msg)  ; 


  }

	
};

int main(int argc, char **argv) {
  std::cout << "StateEstimation Start..." << std::endl;
  ros::init(argc, argv, "StateEstimation");
  StateEstimation stateEstimation;
  ros::spin();
  printf("StateEstimation Finish...");
}
