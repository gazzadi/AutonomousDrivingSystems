#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/cloud_iterator.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include "../include/Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include <unordered_set>
#include "../include/tree_utilities.hpp"
#include <boost/filesystem.hpp>
snamespace fs = boost::filesystem;
//#define USE_PCL_LIBRARY
using namespace lidar_obstacle_detection;

typedef std::unordered_set<int> my_visited_set_t;

//This function sets up the custom kdtree using the point cloud
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, int dimension)
{
    //insert point cloud points into tree
    for (int i = 0; i < cloud->size(); ++i)
    {
        tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
    }
}

/*
OPTIONAL
This function computes the nearest neighbors and builds the clusters
    - Input:
        + cloud: Point cloud to be explored
        + target_ndx: i-th point to visit
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
        + cluster: Here we add points that will represent the cluster
        + max: Max cluster size
    - Output:
        + visited: already visited points
        + cluster: at the end of this function we will have one cluster
*/
void proximity(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree* tree, float distanceTol, my_visited_set_t& visited, std::vector<int>& cluster, int max)
{
	if (cluster.size() < max)
    {
        cluster.push_back(target_ndx);
        visited.insert(target_ndx);

        std::vector<float> point {cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};
    
        // get all neighboring indices of point
        std::vector<int> neighborNdxs = tree->search(point, distanceTol);

        for (int neighborNdx : neighborNdxs)
        {
            // if point was not visited
            if (visited.find(neighborNdx) == visited.end())
            {
                proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
            }

            if (cluster.size() >= max)
            {
                return;
            }
        }
    }
}

/*
OPTIONAL
This function builds the clusters following a euclidean clustering approach
    - Input:
        + cloud: Point cloud to be explored
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters 
        + setMinClusterSize: Minimum cluster size
        + setMaxClusterSize: Max cluster size
    - Output:
        + cluster: at the end of this function we will have a set of clusters
TODO: Complete the function
*/
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree* tree, float distanceTol, int setMinClusterSize, int setMaxClusterSize)
{
	my_visited_set_t visited{};                                                          //already visited points
	std::vector<pcl::PointIndices> clusters;                                             //vector of PointIndices that will contain all the clusters
    std::vector<int> cluster;                                                            //vector of int that is used to store the points that the function proximity will give me back

    
    //for (int i=0; i<tree)
    for(int idx = 0; idx < cloud->points.size(); idx++)
    {
        if(visited.find(idx) == visited.end())
        {   
            //cluster.clear(); // reset what is inside cluster
            proximity(cloud, idx, tree, distanceTol, visited, cluster, setMaxClusterSize);
            if(cluster.size() >= setMinClusterSize)
            {
                pcl::PointIndices current_cluster;
                current_cluster.indices = std::move(cluster);
                clusters.push_back(std::move(current_cluster));
            }
        }
    }
	return clusters;

    //for every point of the cloud
    //  if the point has not been visited (use the function called "find")
    //    find clusters using the proximity function
    //
    //    if we have more clusters than the minimum
    //      Create the cluster and insert it in the vector of clusters. You can extract the indices from the cluster returned by the proximity funciton (use pcl::PointIndices)   
    //    end if
    //  end if
    //end for
	//return clusters;	
}

int user_data;

void 
ProcessAndRenderPointCloud (Renderer& renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // --- Mostra la point cloud originale ---
    
    // TODO: 1) Downsample the dataset h
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::VoxelGrid<pcl::PointXYZ> filterer;
    filterer.setInputCloud (cloud);
    filterer.setLeafSize (0.2f, 0.2f, 0.2f); //this value defines how much the PC is filtered
    filterer.filter (*cloud_filtered);


    // 2) here we crop the points that are far away from us, in which we are not interested
    pcl::CropBox<pcl::PointXYZ> cb(true);
    cb.setInputCloud(cloud_filtered);
    
    cb.setMin(Eigen::Vector4f (-20, -6, -2, 1));
    cb.setMax(Eigen::Vector4f ( 30, 7, 5, 1));
    cb.filter(*cloud_filtered); 

    // TODO: 3) Segmentation and apply RANSAC
    // TODO: 4) iterate over the filtered cloud, segment and remove the planar inliers 

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented (new pcl::PointCloud<pcl::PointXYZ>),  //point cloud with planes
        cloud_aux (new pcl::PointCloud<pcl::PointXYZ>); //aux point cloud
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.2); // determines how close a point must be to the model in order to be considered an inlier

    int i = 0, nr_points = (int) cloud_filtered->size ();
    
    // Now we will remove the planes from the filtered point cloud 
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); //the resultant model coefficients
    //inliers represent the points of the point cloud representing the plane, coefficients of the model that represents the plane (4 points of the plane)
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ()); 
    // While 30% of the original cloud is still there
    while (cloud_filtered->size () > 0.25 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud <-
        seg.setInputCloud (cloud_filtered);
        /*
        Base method for segmentation of a model in a PointCloud given by <setInputCloud (), setIndices ()>
        [out]	inliers	the resultant point indices that support the model found (inliers)
        [out]	model_coefficients	the resultant model coefficients that describe the plane 
        */
        seg.segment (*inliers, *coefficients); //we get one of the planes and we put it into the inliers variable
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // Extract the inliers (here we extract the points of the plane moving the indices representing the plane to cloud_segmented)
        extract.setInputCloud (cloud_filtered); 
        
        //PCL defines a way to define a region of interest / list of point indices that the algorithm should operate on, rather than the entire cloud, via setIndices.
        extract.setIndices (inliers);
        extract.setNegative (false); // Retrieve indices to all points in cloud_filtered but only those referenced by inliers:
        extract.filter (*cloud_segmented);   // We effectively retrieve JUST the plane
        


        // Here we will extract the plane from the original filtered point cloud
        extract.setNegative (true); // original cloud - plane 
        extract.filter (*cloud_aux);  // We write into cloud_f the cloud without the extracted plane
        
        cloud_filtered.swap (cloud_aux); // Here we swap the cloud (the removed plane one) with the original
        i++;
    }

    


    // TODO: 5) Create the KDTree and the vector of PointIndices
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered); 


    // TODO: 6) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!)
    
    #ifdef USE_PCL_LIBRARY

        cout<<"pcl library"<<endl;
        //PCL functions
        //HERE 6)

        //If you take a very small value, it can happen that an actual object can be seen as multiple clusters. 
        //On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster
        
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.7); // 2cm

        //We impose that the clusters found must have at least setMinClusterSize() points and maximum setMaxClusterSize() points
        ec.setMinClusterSize (20);
        ec.setMaxClusterSize (10000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        
        std::vector<pcl::PointIndices> cluster_indices;
        ec.extract (cluster_indices);
    #else
        cout<<"non sto usando la pcl library"<<endl;
        // Optional assignment
        my_pcl::KdTree treeM;
        treeM.set_dimension(3);
        setupKdtree(cloud_filtered, &treeM, 3); //dentro quindi il tree ha elementi formati da x y z
        
        std::vector<pcl::PointIndices> cluster_indices;
        cluster_indices = euclideanCluster(
            cloud_filtered, 
            &treeM, 
            0.7,//clusterTolerance 
            20,//setMinClusterSize 
            10000);//setMaxClusterSize
    #endif

    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};

    /*
    renderer.ClearViewer();
    renderer.RenderPointCloud(cloud_filtered, "cloud_filtrata_background", Color(0.1, 0.2, 0.9));
    renderer.RenderPointCloud(cloud, "cloud_originale", Color(0.1,0.1,0.1));
    /*
    std::cout << "Mostrando la point cloud prima del filtering. Premi Invio per continuare..." << std::endl;
    std::cin.get(); // Mette in pausa l'esecuzione finché non premi Invio
    */

    /**Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. 

    To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
    Compute euclidean distance
    **/
    int j = 0;
    int clusterId = 0;
    std::vector<Box> boxes;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->push_back ((*cloud_filtered)[*pit]); 
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        //renderer.RenderPointCloud(cloud,"originalCloud"+std::to_string(clusterId),colors[2]);
        // TODO: 7) render the cluster and plane without rendering the original cloud 
        //<-- here
        //----------

        //Here we create the bounding box on the detected clusters
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

        //TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle
        Box box{clusterId, minPt.x, minPt.y, minPt.z,
        maxPt.x, maxPt.y, maxPt.z};
        //TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle
        //please take a look at the function RenderBox to see how to color the box
        

        boxes.insert(boxes.end(), box);
        //renderer.RenderBox(box, j);

        ++clusterId;
        //j++;
    }  
    
    for (int it=0; it != boxes.size(); ++it)
    {
        Box box = boxes.at(it);
        float x_dim = box.x_max - box.x_min;
        float y_dim = box.y_max - box.y_min;
        float z_dim = box.z_max - box.z_min;

        //cout<<"Oggetto "<<it<<endl;
        /*cout<<x_dim<<endl;
        cout<<y_dim<<endl;
        cout<<z_dim<<endl;
        */
        std::string cube = "box"+std::to_string(it);
        const std::string label = "_"+cube;
        
        
        //x lunghezza
        //y larghezza
        //z altezza
        float red = x_dim / 7;
        float green = y_dim / 7;
        float blue = z_dim / 7;
        /*cout<<"red: "<<red;
        cout<<" green: "<<green;
        cout<<" blue: "<<blue<<endl;
        */
        renderer.addText((box.x_max + box.x_min)/2.0f, 
                        (box.y_max + box.y_min)/2.0f, 
                        (box.z_max + box.z_min)/2.0f, 
                        label);
        renderer.RenderBox(box, j, Color(red,green,blue));
        //renderer.RenderText(box, j, Color(0,1,0));
        /*if (box.x_dim < 3.5f 
        && box.y_dim < 1.5f 
        && box.z_dim < 1.2f)
        {
            renderer.RenderBox(box, j, Color(0,0,1));
            cout<<" piccolo"<<endl;
        }
        else
        {
            renderer.RenderBox(box, j, Color(1,0,0));
            cout<<" grande"<<endl;
        }*/

        j++;
    }
    //std::cin.get();

}


int main(int argc, char* argv[])
{
    Renderer renderer;
    renderer.InitCamera(CameraAngle::XY);
    // Clear viewer
    renderer.ClearViewer();

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    
    std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"../dataset_1"},
                                                boost::filesystem::directory_iterator{});

    // sort files in ascending (chronological) order
    std::sort(stream.begin(), stream.end());

    auto streamIterator = stream.begin();

    while (not renderer.WasViewerStopped())
    {
        renderer.ClearViewer();

        pcl::PCDReader reader;
        reader.read (streamIterator->string(), *input_cloud);
        auto startTime = std::chrono::steady_clock::now();

        ProcessAndRenderPointCloud(renderer,input_cloud);
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "[PointCloudProcessor<PointT>::ReadPcdFile] Loaded "
        << input_cloud->points.size() << " data points from " << streamIterator->string() <<  "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        renderer.SpinViewerOnce();
    }
}
