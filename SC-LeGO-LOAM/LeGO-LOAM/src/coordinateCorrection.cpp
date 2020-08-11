#include "utility.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>




class coordinateCorrection
{
private:
    ros::NodeHandle nh;
    ros::Subscriber subCurrentFrame;
    tf::StampedTransform transformMapToInit;
    tf::TransformBroadcaster tfBroadcaster;
    Eigen::Quaterniond qtnion;
    
    pcl::PointCloud<PointType>::Ptr surfaceMapCloud;
   
    bool init = false; // whether initialization finished 
    int frameCount = 0;
    int frameNum = 10;// how many frames to estimate the /map coordinate. 

public:
    coordinateCorrection():nh("~")
    {
        surfaceMapCloud.reset(new pcl::PointCloud<PointType>());
        subCurrentFrame = nh.subscribe<sensor_msgs::PointCloud2>("/current_frame", 2, &coordinateCorrection::currentFrameHandler, this);
        transformMapToInit.frame_id_ = "/map";
        transformMapToInit.child_frame_id_ = "/camera_init"; 

    }
    void currentFrameHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        if(frameCount < frameNum){
            frameCount ++;
            pcl::PointCloud<PointType>::Ptr tempCloud(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*msg, *tempCloud);
            *surfaceMapCloud += *tempCloud;
        }
    }
    // 从地面的法线获取/map坐标系
    void getMapFrame(){

        if(!init && surfaceMapCloud->points.size()!=0){
            pcl::PointCloud<pcl::PointXYZ>::Ptr surfaceMapCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*surfaceMapCloud, *surfaceMapCloudXYZ);
            pcl::PointIndices::Ptr inliers;	//存储内点，使用的点
            inliers.reset(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients; //存储输出的模型的系数
            coefficients.reset(new pcl::ModelCoefficients);

            std::vector<int> groundIndex;
            for(int i = 0; i < surfaceMapCloudXYZ->points.size(); i++){
                if(surfaceMapCloudXYZ->points[i].y < -0.2 && surfaceMapCloudXYZ->points[i].y > -0.4) {
                    groundIndex.push_back(i);
                }
            }
            pcl::copyPointCloud(*surfaceMapCloudXYZ, groundIndex, *surfaceMapCloudXYZ);
            
            pcl::SACSegmentation<pcl::PointXYZ> segmen;
            // 可选设置
            segmen.setOptimizeCoefficients (true);
            //必须设置
            segmen.setModelType (pcl::SACMODEL_PLANE); //设置模型类型，检测平面
            segmen.setMethodType (pcl::SAC_RANSAC);		//设置方法【聚类或随机样本一致性】
            segmen.setDistanceThreshold (0.01);
            segmen.setInputCloud (surfaceMapCloudXYZ);
            segmen.segment (*inliers, *coefficients);	//分割操作
            if(coefficients->values[1]<0){
                coefficients->values[0] = coefficients->values[0]*(-1);
                coefficients->values[1] = coefficients->values[1]*(-1);
                coefficients->values[2] = coefficients->values[2]*(-1);
            }
            std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                    <<coefficients->values[1] << " "
                    <<coefficients->values[2] << " " 
                    <<coefficients->values[3] <<std::endl;

            Eigen::Vector3d vectorGroundUp; // 地面法线
            vectorGroundUp = {coefficients->values[0],coefficients->values[1],coefficients->values[2]};
            Eigen::Vector3d vectorCameraInit; // camera_init上方向
            vectorCameraInit = {0,1,0};
            qtnion = Eigen::Quaterniond::FromTwoVectors(vectorGroundUp,vectorCameraInit);
            Eigen::AngleAxisd rotation_vector(qtnion);
            std::cout<<"the anxis is : "<<rotation_vector.axis()<<std::endl;
            std::cout<<"the angle is : "<<rotation_vector.angle()*180/3.14159265358979323846<<std::endl;
            std::cout<<"the inlier points number is : "<<inliers->indices.size()<<std::endl;
            Eigen::Matrix3d rotMatrix1; // roomreal->/camera_init
            rotMatrix1 = qtnion.toRotationMatrix();
            Eigen::Matrix3d rotMatrix2;// /map->roomreal
            rotMatrix2 <<0,0,1,
                        1,0,0,
                        0,1,0;
            Eigen::Matrix3d rotMatrix3;
            rotMatrix3 = rotMatrix2 * rotMatrix1;
            qtnion = rotMatrix3;
            
            init = true;

            if(coefficients->values[1]<0.6){
                ROS_ERROR("the ground is not a plane, please try again elsewhere!");
                init = false;
            }

            if(inliers->indices.size()<20){
                ROS_ERROR("too few inlier ground points, please try again elsewhere!");
                init = false;
            }
        }
    }

    // publish /map->/camera_init TF
    void publish(){
        if(init){
            transformMapToInit.stamp_ = ros::Time().now();
            transformMapToInit.setRotation(tf::Quaternion(qtnion.x(), qtnion.y(), qtnion.z(), qtnion.w()));
            transformMapToInit.setOrigin(tf::Vector3(0, 0, 0));
            tfBroadcaster.sendTransform(transformMapToInit);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init (argc,argv,"lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Coordinate Correction Started.");

    coordinateCorrection CC;

    ros::Rate rate(10);

    while(ros::ok())
    {
        ros::spinOnce();

        CC.getMapFrame();

        CC.publish();

        rate.sleep();
    }
    return 0;
}