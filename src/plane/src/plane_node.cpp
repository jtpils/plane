#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/common.h>

const int N_SCANS = 16;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plane");
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZ> laser, yang, plane2;
    pcl::io::loadPCDFile("outPlane.pcd", laser);

    std::cout << ros::Time::now() << std::endl;
    while(1)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr laser_ptr(new pcl::PointCloud<pcl::PointXYZ>(laser));
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(laser_ptr));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
        ransac.setDistanceThreshold(0.1);
        ransac.computeModel();
        pcl::PointCloud<pcl::PointXYZ> plane;
        std::vector<int> indices;
        ransac.getInliers(indices);
        if(indices.size() < 160)
        {
            break;
        }
        pcl::copyPointCloud(laser, indices, plane);
        plane2 += plane;

        pcl::PointCloud<pcl::PointXYZ> tmp;
        for(int j = 0; j < laser.points.size(); j ++)
        {
            std::vector<int>::iterator iter = find(indices.begin(), indices.end(), j);
            if(iter == indices.end())
            {
                tmp.push_back(laser.points[j]);
            }
        }
        laser = tmp;


        std::vector<pcl::PointCloud<pcl::PointXYZ> > scan;
        scan.resize(N_SCANS);
        for(int k = 0; k < plane.points.size(); k ++)
        {
            pcl::PointXYZ point = plane.points[k];
            int scanID = floor(atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * 180.0 / M_PI);
            if(scanID < 0)
            {
                scanID += (N_SCANS - 1);
            }
            if(scanID < 0)
            {
                continue;
            }
            scan[scanID].push_back(point);
        }

        for(int r = 0; r < N_SCANS; r ++)
        {
            if(scan[r].size() < 2)
            {
                continue;
            }
            
            float minAngle = DBL_MAX, maxAngle = DBL_MIN;
            int minIndex = -1, maxIndex = -1;
            for(int q = 0; q < scan[r].size(); q ++)
            {
                float angle = atan2(scan[r].points[q].x, scan[r].points[q].y) * 180.0 / M_PI;
                if(angle < 0)
                {
                    angle += 360;
                }
                
                if(minAngle > angle)
                {
                    minAngle = angle;
                    minIndex = q;
                }
                
                if(maxAngle < angle)
                {
                    maxAngle = angle;
                    maxIndex = q;
                }
            }

            yang.push_back(scan[r].points[minIndex]);
            yang.push_back(scan[r].points[maxIndex]);
        }
    }
    std::cout << ros::Time::now() << std::endl;

    pcl::io::savePCDFile("yang.pcd", plane2);
    pcl::io::savePCDFile("line.pcd", yang);
    return 0;
}
