#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/visualization/cloud_viewer.h>

#include <visualization_msgs/MarkerArray.h>

#include <iostream>

#define COUNT_BALL 21

ros::Publisher pub;
ros::Publisher marker_pub;
ros::Publisher marker_string_pub;

void set_both_point_node(pcl::PointXYZRGB *searchPoint, visualization_msgs::Marker *node)
{
    for (int i = 0; i < COUNT_BALL; i++)
    {
        node[i].header.frame_id = "/l515_link"; // map frame 기준
        node[i].header.stamp = ros::Time::now();
        node[i].type = visualization_msgs::Marker::SPHERE;
        node[i].id = i;
        node[i].action = visualization_msgs::Marker::ADD;
        node[i].pose.orientation.w = 1.0;
        node[i].pose.position.x = 0.0f; //노드의 x 좌표
        node[i].pose.position.y = 0.0f; //노드의 y 좌표
        node[i].pose.position.z = 1.0f; //노드의 y 좌표
        // Points are green
        node[i].color.g = 0.5;
        node[i].color.a = 1.0;
        node[i].scale.x = 0.2;
        node[i].scale.y = 0.2;
        node[i].scale.z = 0.2;
    }

    searchPoint[0].x = 0.0f;
    searchPoint[0].y = -0.5f;
    searchPoint[0].z = 1.7f;
    node[0].pose.position.x = 1.7f;
    node[0].pose.position.y = 0.0f;
    node[0].pose.position.z = 0.5f;

    searchPoint[1].y = -0.25f;
    searchPoint[1].x = 0.5f;
    searchPoint[1].z = 1.0f;
    node[1].pose.position.x = 1.0f;
    node[1].pose.position.y = -0.5f;
    node[1].pose.position.z = 0.25f;

    searchPoint[2].y = -0.25f;
    searchPoint[2].x = 0.3f;
    searchPoint[2].z = 1.0f;
    node[2].pose.position.x = 1.0f;
    node[2].pose.position.y = -0.3f;
    node[2].pose.position.z = 0.25f;

    searchPoint[3].y = -0.25f;
    searchPoint[3].x = 0.1f;
    searchPoint[3].z = 1.0f;
    node[3].pose.position.x = 1.0f;
    node[3].pose.position.y = -0.1f;
    node[3].pose.position.z = 0.25f;

    searchPoint[4].y = -0.25f;
    searchPoint[4].x = -0.1f;
    searchPoint[4].z = 1.0f;
    node[4].pose.position.x = 1.0f;
    node[4].pose.position.y = 0.1f;
    node[4].pose.position.z = 0.25f;

    searchPoint[5].y = -0.25f;
    searchPoint[5].x = -0.3f;
    searchPoint[5].z = 1.0f;
    node[5].pose.position.x = 1.0f;
    node[5].pose.position.y = 0.3f;
    node[5].pose.position.z = 0.25f;

    searchPoint[6].y = -0.25f;
    searchPoint[6].x = -0.5f;
    searchPoint[6].z = 1.0f;
    node[6].pose.position.x = 1.0f;
    node[6].pose.position.y = 0.5f;
    node[6].pose.position.z = 0.25f;

    searchPoint[7].y = -0.5f;
    searchPoint[7].x = 0.5f;
    searchPoint[7].z = 1.4f;
    node[7].pose.position.x = 1.4f;
    node[7].pose.position.y = -0.5f;
    node[7].pose.position.z = 0.5f;

    searchPoint[8].y = -0.5f;
    searchPoint[8].x = 0.3f;
    searchPoint[8].z = 1.4f;
    node[8].pose.position.x = 1.4f;
    node[8].pose.position.y = -0.3f;
    node[8].pose.position.z = 0.5f;

    searchPoint[9].y = -0.5f;
    searchPoint[9].x = 0.1f;
    searchPoint[9].z = 1.4f;
    node[9].pose.position.x = 1.4f;
    node[9].pose.position.y = -0.1f;
    node[9].pose.position.z = 0.5f;

    searchPoint[10].y = -0.5f;
    searchPoint[10].x = -0.1f;
    searchPoint[10].z = 1.4f;
    node[10].pose.position.x = 1.4f;
    node[10].pose.position.y = 0.1f;
    node[10].pose.position.z = 0.5f;

    searchPoint[11].y = -0.5f;
    searchPoint[11].x = -0.3f;
    searchPoint[11].z = 1.4f;
    node[11].pose.position.x = 1.4f;
    node[11].pose.position.y = 0.3f;
    node[11].pose.position.z = 0.5f;

    searchPoint[12].y = -0.5f;
    searchPoint[12].x = -0.5f;
    searchPoint[12].z = 1.4f;
    node[12].pose.position.x = 1.4f;
    node[12].pose.position.y = 0.5f;
    node[12].pose.position.z = 0.5f;

    searchPoint[13].y = 0.0f;
    searchPoint[13].x = 0.3f;
    searchPoint[13].z = 0.5f;
    node[13].pose.position.x = 0.5f;
    node[13].pose.position.y = -0.3f;
    node[13].pose.position.z = 0.0f;

    searchPoint[14].y = 0.0f;
    searchPoint[14].x = 0.1f;
    searchPoint[14].z = 0.5f;
    node[14].pose.position.x = 0.5f;
    node[14].pose.position.y = -0.1f;
    node[14].pose.position.z = 0.0f;

    searchPoint[15].y = 0.0f;
    searchPoint[15].x = -0.1f;
    searchPoint[15].z = 0.5f;
    node[15].pose.position.x = 0.5f;
    node[15].pose.position.y = 0.1f;
    node[15].pose.position.z = 0.0f;

    searchPoint[16].y = 0.0f;
    searchPoint[16].x = -0.3f;
    searchPoint[16].z = 0.5f;
    node[16].pose.position.x = 0.5f;
    node[16].pose.position.y = 0.3f;
    node[16].pose.position.z = 0.0f;

    searchPoint[17].y = -0.25f;
    searchPoint[17].x = 0.9f;
    searchPoint[17].z = 1.0f;
    node[17].pose.position.x = 1.0f;
    node[17].pose.position.y = -0.9f;
    node[17].pose.position.z = 0.25f;

    searchPoint[18].y = -0.25f;
    searchPoint[18].x = 0.7f;
    searchPoint[18].z = 1.0f;
    node[18].pose.position.x = 1.0f;
    node[18].pose.position.y = -0.7f;
    node[18].pose.position.z = 0.25f;

    searchPoint[19].y = -0.25f;
    searchPoint[19].x = -0.7f;
    searchPoint[19].z = 1.0f;
    node[19].pose.position.x = 1.0f;
    node[19].pose.position.y = 0.7f;
    node[19].pose.position.z = 0.25f;

    searchPoint[20].y = -0.25f;
    searchPoint[20].x = -0.9f;
    searchPoint[20].z = 1.0f;
    node[20].pose.position.x = 1.0f;
    node[20].pose.position.y = 0.9f;
    node[20].pose.position.z = 0.25f;
}

// This is to save on typing
typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t;

point_cloud_t mPtrPointCloud1;
void cloud_cb2(const sensor_msgs::PointCloud2 &ros_pc)
{
    pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary
    pcl_conversions::toPCL(ros_pc, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, mPtrPointCloud1);
}

void cloud_cb(const sensor_msgs::PointCloud2 &ros_pc)
{
    // See http://wiki.ros.org/hydro/Migration for the source of this magic.
    pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary
    pcl_conversions::toPCL(ros_pc, pcl_pc);

    // Convert point cloud to PCL native point cloud
    //point_cloud_t::Ptr input_ptr(new point_cloud_t());
    //pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);
    /*
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud2);
    
    sor.setLeafSize(0.05, 0.05, 0.05);

    // Create output point cloud
    

    // Run filter
    sor.filter(*cloud);
    */

    // 시각적 확인을 위해 색상 통일 (255,255,255)
    /*
    for (size_t i = 0; i < cloud->points.size(); ++i){
        cloud->points[i].r = 255;
        cloud->points[i].g = 255;
        cloud->points[i].b = 255;
    }*/
    //Octree 오브젝트 생성
    float resolution = 0.05f; //복셀 크기 설정(Set octree voxel resolution)
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution);
    octree.setInputCloud(cloud);      // 입력
    octree.addPointsFromInputCloud(); //Octree 생성 (Build Octree)
    //기준점(searchPoint) 설정 방법 #1(x,y,z 좌표 지정)
    std_msgs::String msg_string;

    visualization_msgs::MarkerArray node_link_arr;
    pcl::PointXYZRGB searchPoint[COUNT_BALL];
    visualization_msgs::Marker node[COUNT_BALL];

    set_both_point_node(searchPoint, node);

    //기준점(searchPoint) 설정 방법 #2(3000번째 포인트)
    //pcl::PointXYZRGB searchPoint = cloud->points[3000];
    //기준점 좌표 출력
    //std::cout << "searchPoint :" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z  << std::endl;
    //기준점과 동일한 복셀내 존재 하는 하는 포인트 탐색(Voxel Neighbor Search)

    /*
    
    std::vector<int> pointIdxVec;  //결과물 포인트의 Index 저장(Save the result vector of the voxel neighbor search) 
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    //std::cout << min_x <<' '<<min_y<<' '<<min_z<<' '<<max_x<<' '<<max_y<<' '<<max_z<<endl;
    bool isInBox = (searchPoint.x >= min_x && searchPoint.x <= max_x) && (searchPoint.y >= min_y && searchPoint.y <= max_y) && (searchPoint.z >= min_z && searchPoint.z <= max_z);
    //std::cout << isInBox <<endl;
    if (isInBox && !octree.isVoxelOccupiedAtPoint(searchPoint))
    {
        if (octree.voxelSearch(searchPoint, pointIdxVec))
        {
            std::cout << "voxelSearch" <<endl;
            //시각적 확인을 위하여 색상 변경 (255,0,0)
            for (size_t i = 0; i < pointIdxVec.size(); ++i){
                cloud->points[pointIdxVec[i]].r = 255;
                cloud->points[pointIdxVec[i]].g = 0;
                cloud->points[pointIdxVec[i]].b = 0;
            }        
        }
    }
    else
    {
        std::cout << "not octree :" <<endl;
    }

    


    
    // 기준점에서 가까운 순서중 K번째까지의 포인트 탐색 (K nearest neighbor search)
    int K = 50;   // 탐색할 포인트 수 설정 
    std::vector<int> pointIdxNKNSearch; //Save the index result of the K nearest neighbor
    std::vector<float> pointNKNSquaredDistance;  //Save the index result of the K nearest neighbor
    if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {   
          //시각적 확인을 위하여 색상 변경 (0,255,0)
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){
            cloud->points[pointIdxNKNSearch[i]].r = 0;
            cloud->points[pointIdxNKNSearch[i]].g = 255;
            cloud->points[pointIdxNKNSearch[i]].b = 0;
        }    
    }

   
     // 탐색된 점의 수 출력 
    std::cout << "K = 50 nearest neighbors:" << pointIdxNKNSearch.size() << endl;
     */

    //기준점에서 지정된 반경내 포인트 탐색 (Neighbor search within radius)
    float radius = 0.11;                                       //탐색할 반경 설정(Set the search radius)
    std::vector<int> pointIdxRadiusSearch[COUNT_BALL];         //Save the index of each neighbor
    std::vector<float> pointRadiusSquaredDistance[COUNT_BALL]; //Save the square of the Euclidean distance between each neighbor and the search point

    char buff[COUNT_BALL];

    for (int i = 0; i < COUNT_BALL; i++)
    {
        buff[i] = '0';
        if (octree.radiusSearch(searchPoint[i], radius, pointIdxRadiusSearch[i], pointRadiusSquaredDistance[i]) > 0)
        {
            for (size_t j = 0; j < pointIdxRadiusSearch[i].size(); ++j)
            {
                cloud->points[pointIdxRadiusSearch[i][j]].r = 0;
                cloud->points[pointIdxRadiusSearch[i][j]].g = 0;
                cloud->points[pointIdxRadiusSearch[i][j]].b = 255;
            }
        }
        std::cout << ' ' << pointIdxRadiusSearch[i].size();

        if (i == 0 || i == 6)
        {
            std::cout << endl;
        }

        if (pointIdxRadiusSearch[i].size() > 500)
        {
            node[i].color.r = 0.5;
            node[i].color.g = 0;
            buff[i] = '1';
        }
    }
    buff[COUNT_BALL] = '\0';

    for (int i = 0; i < COUNT_BALL; i++)
    {
        node_link_arr.markers.push_back(node[i]);
    }

    std::cout << endl;
    /*
    if (octree.radiusSearch(searchPoint[0], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {    
        
          //시각적 확인을 위하여 색상 변경 (0,0,255)
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
            cloud->points[pointIdxRadiusSearch[i]].r = 0;
            cloud->points[pointIdxRadiusSearch[i]].g = 0;
            cloud->points[pointIdxRadiusSearch[i]].b = 255;
        }        
       
    } */
    // 탐색된 점의 수 출력

    //*input_ptr+=mPtrPointCloud1;
    // Set up VoxelGrid filter to bin into 10cm grid
    //pcl::VoxelGrid<pcl::PointXYZ> sor;
    //sor.setInputCloud(input_ptr);

    //sor.setLeafSize(0.1, 0.1, 0.1);

    // Create output point cloud
    point_cloud_t::Ptr output_ptr(new point_cloud_t());

    // Run filter
    //sor.filter(*output_ptr);

    // Now covert output back from PCL native type to ROS
    sensor_msgs::PointCloud2 ros_output;
    //pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
    pcl::toPCLPointCloud2(*cloud, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, ros_output);

    // Publish the data
    pub.publish(ros_output);
    marker_pub.publish(node_link_arr);

    std::string char2str(buff);
    msg_string.data = char2str;
    marker_string_pub.publish(msg_string);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_voxel");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("l515/depth/pointcloud", 1, cloud_cb);
    ros::Subscriber sub2 = nh.subscribe("l515_back/depth/pointcloud", 1, cloud_cb2);
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("l515/depth/pointcloud_new", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker/node", 1);
    marker_string_pub = nh.advertise<std_msgs::String>("/hero_marker_string", 1);
    // Spin
    ros::spin();
}