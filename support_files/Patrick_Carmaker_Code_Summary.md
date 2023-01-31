# Patrick CarMaker Code Summary
## User.cpp:
```C++
    LidarTracker lidar_tracker;
    int current_iteration = 0;
    int LidarL = 0;
    int LidarR = 1;
```
```C++
    void
    User_Out (const unsigned CycleNo)
    {
        RBS_OutMap(CycleNo);

        if (SimCore.State != SCState_Simulate)
        return;

        if (current_iteration == 0 || current_iteration >= 60)
        {
            lidar_tracker.calculateActualBusPosition();
            lidar_tracker.processLidar(LidarL, LidarR);
            lidar_tracker.calculateError();
            current_iteration = 0;
        }
        current_iteration++;
    }
```
## StateAndMatrix.h
    1. struct StateVector
    2. struct MeasurementVector
    3. template <size_t rows, size_t columns> struct Matrix

## LidarDetectionAndTracking.h:
    1. struct PointCloudPoint
    2. struct PCDpostion
    3. struct GridStruct
    4. struct GridCell
    5. struct GeometricPoint2D
    6. struct params
    7. class LidarTracker
    8. struct Color

## LidarDetectionAndTracking.cpp:
1. LidarTracker::LidarTracker():
   - resize and reserve data
   - initialize Matrices for Kalman Filter
2. void LidarTracker::processLidar(int lidarL, int lidarR):
   - check whether to plot this iteration ?
   - extract relevant point cloud data
     ``` C++
        std::array<std::vector<PointCloudPoint>, 2> pcd_;
     ``` 
     ```C++
        extractPCD(lidarL, pcd_[0]);
	    extractPCD(lidarR, pcd_[1]);
     ```
     ```C++
        void LidarTracker::extractPCD(int lidar_in, std::vector<PointCloudPoint>& pcd)
        {
            // reset old data
            for (auto& p : pcd) {
                p.setLOFandID(0., -1);
            }

            // read in new data
            tScanPoint* p = LidarRSI[lidar_in].ScanPoint;
            for (int i = 0; i < LidarRSI[lidar_in].nScanPoints; i++)
            {
                //int column = p->BeamID % param.number_of_columns;
                //int row = p->BeamID / param.number_of_columns;
                //int index = column * param.number_of_rows + row;
                int index = (p->BeamID % param.number_of_columns) * param.number_of_rows + p->BeamID / param.number_of_columns;
                pcd[index].setLOFandID(p->LengthOF, p->BeamID);
                p++;
            }
        }
     ```
   - calculate angles and cartesian coordinates for point cloud data
        ```C++
        void LidarTracker::preprocessPointCloud(std::vector<PointCloudPoint>& pcd)
        ```
        - skip outdated data
        - ```C++
            calculateAzimuth(p);
		    calculateElevation(p);
		    calculateCartesian(p);
          ```
            azimuth: from left to right
            elevation: from bottom to top
            ```C++
                ToDo: LengthOF, BeamID from Lidar  =>  Azimuth, Elevation, Cartesian (x, y, z)

                int column = p.beamID % param.number_of_columns;
                p.azimuth = param.min_azimuth - sensor_rotation_offset + azimuth_offset + param.azimuth_range / param.number_of_columns * column;

                int row = p.beamID / param.number_of_columns;
                p.elevation = param.min_elevation + elevation_offset + param.elevation_range / param.number_of_rows * row;

                LengthOF;	// Path length in m

                p.x = p.lengthOF / 2. * std::cos(p.azimuth) * std::cos(p.elevation);
                p.y = p.lengthOF / 2. * std::sin(p.azimuth) * std::cos(p.elevation) + dy_sensor_mount;
                p.z = p.lengthOF / 2. * std::sin(p.elevation) + param.dz_sensor_mounting;
            ``` 
        - check if the point is valid, w.r.t. {min_allowed_z, max_allowed_z, max_x_ego, max_y_ego, max_x, min_x, max_y}
            ```C++
                p.valid = true;
            ```
  
3. filter ground points
     ```C++
        void LidarTracker::filterGroundPoints(std::vector<PointCloudPoint>& pcd, std::vector<PointCloudPoint>& ground_pcd)
     ```

4. find clusters
     ```C++
        std::vector<std::vector<PointCloudPoint>> clusters;
	    std::vector<std::vector<GridCell>> clusters_area;
	    clusterPointCloud(pcd_, clusters, clusters_area);
     ```

5. find bus cluster 
     ```C++
        int bus_cluster_index = findBusCluster(clusters, clusters_area);

        MeasurementVector buspose;
        if (bus_cluster_index == -1) {
            //Log("no buscluster found, timer: %i\n", timer);
        }
        else {
            fitBusPosition(clusters[bus_cluster_index], buspose);
        }
     ```

6. Tracking: trackBus(buspose)
     ```C++
        void LidarTracker::trackBus(MeasurementVector const& buspose){
            last_measurement = buspose;
            // if the prediction is not valid yet, rely on the measurement
            // get velocity
	        double v = Car.ConBdy1.v;   // ???
            // update Kalman Gain
	        // innovate
            // update covariance
            // predict
            // update Jacobi Matrix
            // update covariance
            // debug
            if (param.dont_use_KF) {
                last_output.x = buspose.x;
                last_output.y = buspose.y;
                last_output.phi = buspose.phi;
            }
            // save when plotting
            if (debug_plotting) {
                bus_position_history.emplace_back(last_output);
            }
        }
     ```
7. plot
     ```C++
        if (plot_this_iteration)
        {
            // plot point cloud
            plotClusteredCloud(clusters, ground_pcd_);

            // plot bus position history
            if (plot_number == plot_time.size() - 1) {
                plotBusPositionHistory();
            }

            plotConvexHull();

            Log("Created plot with number: %i \n", plot_number);
        }
        timer++;
     ``` 



---

Correct the errors of the project "masterarbeit_patrick_carmaker", which apper after compiling on linux system:
```C++
"LidarDetectionAndTracking.h" in code line 20:
#include "Car.h"
        => 
#include "Car/Car.h"
```

```C++
"LidarDetectionAndTracking.h" in code line 210:
__int64 plot_number = 0;     
// __int64 is only supported by MSVC on windows, but not by g++ on linux
        => 
size_t plot_number = 0;
```

```C++
"LidarDetectionAndTracking.cpp" in code line 495:
for (auto& point = cluster.begin() + 1; point != cluster.end(); point++)
        => 
for (auto point = cluster.begin() + 1; point != cluster.end(); point++)
```


