In this report i want to expose the choiches made for the completion of this assignemt, with a focus on the parameter used for each part of the filtering and clustering.

#### Naive Object Detection
To expand my project i've added a simple and naive form of object detection, that colored the boxes differntly using the proportion of the object.
I've based this implementation on the average size of 3 type of object that i thought to be relevant to this clustering, and to simplify it i've used only 2 proportion for each object to :
- car: 4,5m x 2m x 1,5m
    - x/y proportion: 2,25
    - x/z proportion: 3
- bike: 2,2m x 0,7m x 1,25m(1,75m if a person currently on it) 
    - x/y proportion: 3,14
    - x/z proportion: 1,5 mean
        - min 1,25
        - max 1,76
- person: 0,22m x 0,4m x 1,5m-1,9m
    - x/y proportion: 0,55
    - x/z proportion: 0,13 mean
        - min 0,14
        - max 0,11 

For the detection i've used range based on this values but applying a ±33%



#### Dataset_1
The following parameters are the one that i've used to clustering on the first dataset:
- **voxelFiltering-leafSize** = 0,2m in each dimension
- **segmentation** = 
    - RANSAC algorithm until only 25% of the original cloud remain
    - distanceThreshold = 0.2
- **clustering** = euclidean clustering, with these parameters:
    - _clusterTolerance_ = 0.6cm
    - _minClusterSize_ = 20 points
    - _maxClusterSize_ = 500 points    

I found out that this pointCloud is 

This scene was very simple, and represent a car going in a straight line in a urban road, with car parked on both side.
Surely the straight edges of the street and buildings helps the cluster and i needed only few tries to find the correct parameters.

The majority of cluster in our scene are of cars and the implementation of the naive object detection can correctly identify them, losing only some for few frames.


Don't having so much diversity we will concentrate to analyze the results of the object detection on the second dataset. 


#### Dataset_2
This dataset describe a scene surely more complex that the first one, the car this time don't ride a straight road, and the trajectory have a lot more of different objects in the street.
From the point cloud also seems that the car it's passing throught a road cross.

The complexity of the scene makes the finetuning done on dataset_1 not very useful, it works to create some vague cluster, but it fail to define correct delimitation.

I've tried changing some of the parameters to see better results, but it did't work really well, i will discuss here which parameters i changed and the effect obtained:
- Increasing DistanceThreshold in segmentation from 0,2m to 0,3m: this take care of the sidewalk on the curves and slighlty improve the result
- Decreasing the percentage of cloud i want after the segmentation phase from 25% to 20%: this helped a little, i've tried to go even lower but the information lost was too much.
- Increasing the clusterTolerance from 0.6cm to 0.8cm

I think 
