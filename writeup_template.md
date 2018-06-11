## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

To plan the motion of the drone in this dense environnement we had the possibility to build a graph or a grid. The grid is pretty straightforward and work well for self-driving car as long as we have to plan in 2 dimensions. But even in 2 dimensions this method is expensive because it is both complete and optimal. The graph allows us to build probabilistic roadmap which are only asymptotically complete and optimal. It works well in 3 dimensions because it doesn't try to find the perfect path but one possible path among many randomized graph. As a matter of fact a path to go from A to B will never be computed identically two consecutive time cause the graph will be randomly generated from a generation to the other. It's fast but most of the time the path as to be refined and/or pruned to make it efficient.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes a basic A* algorithm, only authorizing 4 actions (Up Down Left Right) which is not realistic.

![simple_case](./misc/simple_case.gif)

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.7

We read the first line of the csv file to extract lat0 and lon0 as floating point values and used the self.set_home_position() method to set global home. We used the csv module to handle the csv file and we use a comprehensive list to extract the data of interest in a clean way.

``
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=';,', dtype='str')[0].split(", ")
lat0, lon0 = [float(d.split(" ")[1]) for d in data]
``

![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

We retrieved the current local position from the global one using the global_to_local() function which hide the details of using the utm module. With the utm library, it's possible to quickly find the zone number and letter as well as easting and northing for a particular latitude and longitude (given in decimal degrees) like this:

``
import utm
(easting, northing, zone_number, zone_letter) = utm.from_latlon(latitude, longitude)
``

Similarly, you can convert from a UTM position back to latitude and longitude:

``
(latitude, longitude) = utm.to_latlon(easting, northing, zone_number, zone_letter)
``

Here is our final code:

``
current_lcl_pos = global_to_local(current_glbl_pos, self.global_home)
``

![high_up](./misc/high_up.png)

#### 3. Set grid start position from local position

``
grid_start = (-north_offset, -east_offset)
grid_start = (int(current_lcl_pos[0]), int(current_lcl_pos[1]))
``

![simple_case](./misc/simple_case.gif)

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

As we use a graph to build a probabilistic roadmap, the nodes are choosen randomly on the map in a 3-dimensionnal range and then are deleted if thez collapse with a a building/obstacle. 

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

We modified A* to make it work with a graph. We selected the 10th closest neighborhood of each nodes as point of the graph. We could take more but it could result in heavy computationality.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

We decide to prune the the path using a personalized method. We are looping over each point from the first to the last but we are testing the connection between the current point returned by the loop and the last point. In other word we are usin a reversed loop into the loop in order to test direct connection over the path we got from a_star.

![path_planning](./misc/path_planning.gif)

### Execute the flight
#### 1. Does it work?
It works fine but a problem is the drone is transitionning too fast along the start and the end altitude, ignoring the path and sometimes colliding with a wall. We could avoid this problem by using the receiding technics which consists in cubic grid centered on the drone while its following the path. Sometimes the drone take wide angle when the path present a shift in direction and if a wall is to close it crashes. We could avoid that problem by creating a repulsive vector field around obstacles. This field would eventually slower the drone when it's close from an obstacle.

![receiding](./misc/receiding.png)
