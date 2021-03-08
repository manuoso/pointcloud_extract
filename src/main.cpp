/////////////////////////////////////////////////////////////////
//															   //
//                    Main Extract PTS                         //
//															   //
//				Author: Manuel P. J. (aka. manuoso)			   //
//															   //
/////////////////////////////////////////////////////////////////

#include <iostream>
#include <thread>
#include <chrono>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudExtracted;

//---------------------------------------------------------------------------------------------------------------------
size_t split(const std::string &txt, std::vector<std::string> &strs, char ch) {
    size_t pos = txt.find( ch );
    size_t initialPos = 0;
    strs.clear();

    // Decompose statement
    while( pos != std::string::npos ) {
        strs.push_back( txt.substr( initialPos, pos - initialPos ) );
        initialPos = pos + 1;

        pos = txt.find( ch, initialPos );
    }

    // Add the last one
    strs.push_back( txt.substr( initialPos, std::min( pos, txt.size() ) - initialPos + 1 ) );

    return strs.size();
}

//---------------------------------------------------------------------------------------------------------------------
bool convertToPointCloud(std::string _dir)  
{   
    std::ifstream file_(_dir);
    if(file_.is_open()){
        for (std::string line_; std::getline(file_, line_); ) {
            std::vector<std::string> tokens;		
            split(line_, tokens, ' ');
            
            if(tokens.size() == 7){
                pcl::PointXYZRGB pointExtracted;

                float pX, pY, pZ, pR, pG, pB;
                std::stringstream ssX, ssY, ssZ, ssR, ssG, ssB;
                ssX << tokens[0]; ssX >> pX;
                ssY << tokens[1]; ssY >> pY;
                ssZ << tokens[2]; ssZ >> pZ;
                ssR << tokens[4]; ssR >> pR; 
                ssG << tokens[5]; ssG >> pG;
                ssB << tokens[6]; ssB >> pB;

                pointExtracted.x = pX;
                pointExtracted.y = pY;
                pointExtracted.z = pZ;
                pointExtracted.r = pR;
                pointExtracted.g = pG;
                pointExtracted.b = pB;
                    
                pointCloudExtracted->push_back(pointExtracted);
            }	
        }
        file_.close();
    }else{
        std::cout << "File NOT open" << std::endl;
        return false;
    }

    pcl::io::savePCDFileASCII("poindCloudRGB.pcd", *pointCloudExtracted);
    pcl::io::savePLYFileASCII("poindCloudRGB.ply", *pointCloudExtracted);
    
    return true; 
}

int main(int _argc, char **_argv){

    pointCloudExtracted.reset (new pcl::PointCloud<pcl::PointXYZRGB>);

    if(convertToPointCloud(_argv[1])){

        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Pointcloud Leica"));
        viewer->setBackgroundColor(0, 0, 0);

        viewer->addPointCloud(pointCloudExtracted, "cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
        viewer->addCoordinateSystem (1.0);

        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    return 0;

}

