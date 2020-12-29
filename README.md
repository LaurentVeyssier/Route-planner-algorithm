# Route-planner-algorithm
Find the shortest route using A* algorithm and graphs (Route Planner application)

This is the final project of UDACITY's [Data Structures & Algorithms Nanodegree Program](https://www.udacity.com/course/data-structures-and-algorithms-nanodegree--nd256).

The objective of the project is to use A* search to implement a "Google-maps" style route planning algorithm.

# Project content

The project uses a graph with 40 location points. Existing edges (roads) between locations are provided as well as the (x,y) coordinates of all location points.
The objective is to find the shortest route from a starting location to a goal location.
The algorithm outputs the shortest travelling distance and the shortest route (ordered sequence of traversed locations).


I implement 3 different algorithms in the py file.
- Calculation of the shortest route using Dijkstra Dynamic programming algorithm
- Calculation of the shortest route using Uniform Cost algorithm
- Calculation of the shortest route using A* optimal algorithm used for route planning

# A* algorithm

A-Star always select the location with shortest total distance from start to goal ("Best estimated total path cost first"). This is superior to Uniform Cost algorithm which always selects the location with shortest distance from the start location ("Best path cost first"), therefore leading to excessive exploration of the map. On the other hand, Uniform cost is guaranteed to return the shortest path.

A* uses a heuristic function to estimate the distance between two locations. A* is guaranteed to find the solution provided this estimating function returns a distance which is less than or equal to the true path length between the points. Since the 2D coordinates of each locations are provided, I use the euclidian distance, or straight-line distance, between a point and the target. This is guaranteed to be shorter than reality.

As a consequence, A* combines the advantages of a greedy search and the Uniform Cost search which is guaranteed and provides the best search strategy.

A* estimation function f can be broken into: `f = g + h` where `g` is the path cost from the start location up to current location and `h` is the estimated distance from the current location to the goal point.
- By minimizing `g`, the algorithm allows to keep the path as short as possible.
- By minimizing `h`, it allows to stay focused on finding the goal (and reduce unnecessary exploration).

# Results

illustrative graph with start location (black), itinary (red) and goal location (yellow).

![](asset/newplot.png)



