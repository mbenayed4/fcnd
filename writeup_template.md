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

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

planning_utils.py:
   Provides utilities to:
   - create grid
   - deterine valid actions from a grid position: I have 8 possible action from any position: N S E S NE NW SE SW
   - Astar: find best path from start to goal based on heuristic cost function
   - Heuristic: Sqrt
   - prune path: prunes path base on collinearity
   - collinearity: determines if 3 points are near each other

 
motion_planning.py:
- Given Start, Goal, determines best cost path, and executes flight path (arming, guiding to goal, disarming)
- Determines start and goal
- Loads obstacle map
- Discretize the environment into a grid 
- Performs A* search 
- Uses a collinearity test to remove unnecessary waypoints.
- Returns waypoints in local ECEF coordinates 

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

read 1st line from csv file, parse lat0/lon0, set home_position()
 
#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.
this is accomplished using global_to_local(global_pos, global_home)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

done

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

done

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

Added states for NE NW SE SW
Updated heuristic to calcular Sqrt(x2 + y2)
#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

using prune_path function
uses collinearity between 3 points to prune paths

### Execute the flight
#### 1. Does it work?
It works!
