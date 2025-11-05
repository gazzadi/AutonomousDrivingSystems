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
- **segmentation** 
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
- Increasing the clusterTolerance from 0.6m to 0.8m to simplify the clustering process in areas where the points of object are more sparse

Even with this modification the clustering on this scenario is not very efficient, probably for these reasons:
- The finetuning is not a good way to find a solution, because the movement of the car made it work for some areas (like in the beginning and in the end) but do a poor job on very noisy areas like the turn in the middle.
- This dataset represent a scene with a lot of different actors, so clustering with the same principle all the different object maybe it's not he best solution, probabily we need to implement a different strategy of clustering. The euclidean clustering maybe it's not the best solution for this scenario.
- Some of the surface in the point cloud are not plain, so the segmentation phase don't work very well and that is creating some problems detecting the clusters.
- The cropping method implemented it's very basic, maybe cropping the area with a well sized sphere can give better result to take out all the not useful data.

Now want to talk brifly about the naive object detection implemented. In this second dataset we have more interesting result, but still not very precise.
In the part where the clustering work better, the car are detected correctely, and even a bike is detected for a good amount of frames.
But we see all the limit of the implementation in the person detection, that due to the not so precise clustering, and the variable size ratio have a difficult time identifing the person. Sometimes it even identify a pole as a person.
I can conlude that this naive approach to object detection, even if it can be useful for a vague car detection, it's not very reliable with other objects. Even trying to increasing the error on the ratio doesn't give more useful results.




