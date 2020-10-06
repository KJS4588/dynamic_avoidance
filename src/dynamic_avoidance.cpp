#include "dynamic_avoidance/dynamic_avoidance.h"

void DynamicAvoidance::initSetup(){
    point_sub_ = nh_.subscribe("/velodyne_points", 10, &DynamicAvoidance::pointCallback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 10);
    point_pub_ = nh_.advertise<visualization_msgs::Marker>("/mean_point", 10);
	stop_pub_ = nh_.advertise<std_msgs::Bool>("/obs_flag", 10);
	finish_pub_ = nh_.advertise<std_msgs::Bool>("/dynamic_finish", 10);
	
	stop_flag_.data = false;
}

void DynamicAvoidance::pointCallback(const sensor_msgs::PointCloud2ConstPtr &input){
	obs_ = clustering(input, 1, 7.5, -0.5, 0.5);
	if (obs_.size() == 0) {
		cout << "There is no obstacle in area" << endl;
		stop_flag_.data = false;
	}else {
		double dist = calcDist(obs_.at(0));

		cout << "DIST : " << dist << endl;
		if (dist <= DIST){
			stop_flag_.data = true;
			dynamic_finish_ = true;

		}else {
			stop_flag_.data = false;

			if (dynamic_finish_) {
				finish_flag_.data = true;
				finish_pub_.publish(finish_flag_);

				ros::shutdown();
			}
		}
	}

	stop_pub_.publish(stop_flag_);
}

vector<geometry_msgs::Point> DynamicAvoidance::clustering(const sensor_msgs::PointCloud2ConstPtr &input, double min_x, double max_x, double min_y, double max_y){
    pcl::PointCloud<PointType>::Ptr msg (new pcl::PointCloud<PointType>), cloud (new pcl::PointCloud<PointType>), cloud_filterd (new pcl::PointCloud<PointType>);

    pcl::fromROSMsg(*input, *msg);
	for (size_t i=0; i<msg->points.size(); i++){
		PointType point;
        point.x = msg->points[i].x*cos(10*M_PI/180) + msg->points[i].z*sin(10*M_PI/180);
		point.y = msg->points[i].y;
		point.z = -msg->points[i].x*sin(10*M_PI/180) + msg->points[i].z*cos(10*M_PI/180);
        point.intensity = msg->points[i].intensity;
        cloud->points.push_back(point);
	}
   	 
    cloud->header.frame_id = "velodyne";

    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_x, max_x);
    pass.filter(*cloud);

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(*cloud);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // Create a pcl object to hold the ransac filtered object
    pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType>()); 
    
    pcl::SACSegmentation<PointType> seg;
    
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.03);
    seg.setMaxIterations(100);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    extract.setNegative(true);
    extract.filter(*cloud_filterd);

    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree -> setInputCloud(cloud_filterd);

    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(1); //30cm
    ec.setMinClusterSize(6);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filterd);
    ec.extract(cluster_indices);


    pcl::PointCloud<PointType> cluster_cloud1, cluster_cloud2, cluster_cloud3, Result_cloud;
    cout << "Number of clusters is equal to " << cluster_indices.size() << endl;
    int j = 0;

    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
        if(it == cluster_indices.begin()){
        	for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            	PointType pt2;
            	pt2.x = cloud_filterd->points[*pit].x, pt2.y = cloud_filterd->points[*pit].y, pt2.z = cloud_filterd->points[*pit].z;
            	pt2.intensity = (float)(j+1);

            	cluster_cloud1.push_back(pt2);}
		}
        
		if(it == cluster_indices.begin()+1){
        	for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            	PointType pt2;
            	pt2.x = cloud_filterd->points[*pit].x, pt2.y = cloud_filterd->points[*pit].y, pt2.z = cloud_filterd->points[*pit].z;
            	pt2.intensity = (float)(j+1);
				cluster_cloud2.push_back(pt2);}
		}
		
        if(it == cluster_indices.begin()+2){
        	for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            	PointType pt2;
            	pt2.x = cloud_filterd->points[*pit].x, pt2.y = cloud_filterd->points[*pit].y, pt2.z = cloud_filterd->points[*pit].z;
            	pt2.intensity = (float)(j+1);
				cluster_cloud3.push_back(pt2);}
		}
        j++; 
    }
	
    double sum_x=0; 
    double sum_y=0;
    double sum_z=0;
	visualization_msgs::Marker mean_point;
	mean_point.header.frame_id = "velodyne";
	mean_point.header.stamp = ros::Time::now();
	mean_point.ns = "points";
	mean_point.action = visualization_msgs::Marker::ADD;
	mean_point.type = visualization_msgs::Marker::POINTS;
	mean_point.pose.orientation.w = 1;
	mean_point.id = 0;
	mean_point.color.g = 1.0f; 
	mean_point.color.a = 1.0;
    mean_point.scale.x = 0.1;
    mean_point.scale.y = 0.1;
	
	vector<geometry_msgs::Point> mean_p;
	
	if (cluster_cloud1.size() != 0){
        for (size_t i=0; i<cluster_cloud1.size(); i++){
            sum_x += cluster_cloud1[i].x;
            sum_y += cluster_cloud1[i].y;
            sum_z += cluster_cloud1[i].z;
        
        }
    
        geometry_msgs::Point p_;

        p_.x = sum_x / cluster_cloud1.size();
        p_.y = sum_y / cluster_cloud1.size();
        p_.z = sum_z / cluster_cloud1.size();

        mean_point.points.push_back(p_);
		mean_p.push_back(p_);
    }

    if (cluster_cloud2.size() != 0){
        for (size_t i=0; i<cluster_cloud2.size(); i++){
            sum_x += cluster_cloud2[i].x;
            sum_y += cluster_cloud2[i].y;
            sum_z += cluster_cloud2[i].z;
        
        }
    
        geometry_msgs::Point p_;

        p_.x = sum_x / cluster_cloud2.size();
        p_.y = sum_y / cluster_cloud2.size();
        p_.z = sum_z / cluster_cloud2.size();

        mean_point.points.push_back(p_);
		mean_p.push_back(p_);
		sum_x = 0; sum_y = 0; sum_z = 0;
    }

	if (cluster_cloud3.size() != 0){
        for (size_t i=0; i<cluster_cloud3.size(); i++){
            sum_x += cluster_cloud3[i].x;
            sum_y += cluster_cloud3[i].y;
            sum_z += cluster_cloud3[i].z;
        
        }
    
        geometry_msgs::Point p_;

        p_.x = sum_x / cluster_cloud3.size();
        p_.y = sum_y / cluster_cloud3.size();
        p_.z = sum_z / cluster_cloud3.size();

        mean_point.points.push_back(p_);
		mean_p.push_back(p_);
    }

    point_pub_.publish(mean_point); 

	Result_cloud += cluster_cloud1;
	Result_cloud += cluster_cloud2;
	Result_cloud += cluster_cloud3; 
	
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(Result_cloud, cloud_p);

    sensor_msgs::PointCloud2 result;
    pcl_conversions::fromPCL(cloud_p, result);
    result.header.frame_id = "velodyne";
    pub_.publish(result);

	return mean_p;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "dynamic_avoidance_node");
    DynamicAvoidance da;
    da.initSetup();
    ros::spin();
}

