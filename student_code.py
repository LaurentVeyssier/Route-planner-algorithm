import numpy as np
from copy import deepcopy

#################################################################
####################### Dijkstra algorithm ######################
#################################################################

# Find minimum path cost using Dijkstra's algorithm
def extract_min_vertex_from_queue(distances, queue):
    dist_for_queue =  {k:distances[k] for k in queue}
    return [k for k,v in sorted(dist_for_queue.items(), key= lambda x: x[1])][0]

def dijkstra(M, start_node, end_node):
    # initialize parameters
    coordinates = M.intersections       # dictionary  {node index : [x,y] location coordinates}
    distances = {node: np.inf for node in M._graph.nodes()}
    distances[start_node] = 0
    queue = M._graph.nodes()
    
    while queue != []:
        #print(extract_min_vertex_from_queue(distances, queue).value)
        current_node = extract_min_vertex_from_queue(distances, queue)
        queue.remove(current_node)
        for neighbour in M.roads[current_node]:
            dist = distances[current_node] + calculate_distance(current_node, neighbour, coordinates)
            if dist < distances[neighbour]:
                distances[neighbour] = dist
    
    return distances[end_node]


#################################################################
#################   Uniform Cost algorithm ######################
#################################################################

def calculate_distance(nodeA, nodeB, coordinates):
    '''calculate the distance between to locations using their coordinates and pythagore theorem
    inputs:
    - nodeA (int)
    - nodeB (int)
    - coordinates: dictionary with (x,y) coordinates for all nodes in graph
    Output:
    - return the euclidian distance between the 2 points A and B
    '''
    xa,ya = coordinates[nodeA]
    xb,yb = coordinates[nodeB]
    return np.sqrt( (xa-xb)**2 + (ya-yb)**2 )


def find_nearest_frontier_node(routes):
    '''select and return the location on the frontier having the shortest path_cost
    input:
    - routes: dictionary associating nodes on the frontier to a composed of a tuple (route, path length). The route is the sequence of traversed locations from start -> to frontier node
    output:
    - return the location on the frontier having the shortest path_cost
    '''
    path_costs =  {node:routes[node][1] for node in routes}
    return [node for node,path_cost in sorted(path_costs.items(), key= lambda x: x[1])][0]

# Find shortest route using Uniform Cost greedy algorithm
def shortest_path_Uniform_Cost_Search(M,start,goal):
    '''this function returns the shortest path between a start location and a goal location calculated with Uniform Cost algorithm
    inputs:
        - M: a Map object (Class Map)
        - start: the starting node index (integer)
        - goal: the target node index (integer)
    output:
        - a list of integers representing the shortest path calculated using Uniform Cost algorithm
        '''
    
    print("shortest path called")
    # informative function indicating the minimum path cost using dijkstra algorithm
    print('Minimum traversing distance to reach goal using Dijkstra greedy algorithm: {:.2f}'.format(dijkstra(M, start, goal)))
    
    # Initialize parameters
    #nodes = M._graph.nodes()           # list of all intersections of the graph M
    coordinates = M.intersections       # dictionary  {node index : [x,y] location coordinates}
    neighbours = M.roads                # adjacency list of list [node_index => [list of neighbours],...]
    explored = set()                    # set for locations marked as explored
    frontier = {start:([start], 0)}     # dictionary associating nodes on the frontier to a composed of a tuple (route, path length). 
                                        # The route is the sequence of traversed locations from start -> to frontier node
    candidates = []                     # list for all possible routes found (from start to goal). Tuple: (route, path_length)
    
    # Identify all possible routes to the target goal with Uniform Cost algorithm 
    while len(frontier) > 0:
        
        # while there is node on the frontier, select the one with shortest distance to start (greedy approach)
        current_node = find_nearest_frontier_node(frontier)
        # collect identified route up to this node on the frontier
        current_route, path_cost = frontier[current_node]
        # remove node from frontier
        frontier.pop(current_node)
        # mark node as explored
        explored.add(current_node)
        
        # visit all neighbours of the selected node on the frontier
        for neighbour in neighbours[current_node]:
            # add neighbour to the route. use deepcopy to preserve current_route unchanged for later re-use
            new_route = deepcopy(current_route)
            new_route.append(neighbour)
            
            # calculate step_cost to transition to the neighbour from frontier node
            step_cost = calculate_distance(current_node, neighbour, coordinates)
            # update path cost with new step cost
            new_path_cost = path_cost + step_cost
            
            # if the neighbour is the target then store the tuple (route, path_cost) as a candidate
            if neighbour == goal:
                candidates.append((new_route, new_path_cost))
            
            # if the neighbour is already explored do nothing and move to next neighbour
            elif neighbour not in explored:
                # if the neighbour is not already on the frontier, add it to the frontier with its (route, path_cost)
                if neighbour not in frontier:
                    frontier[neighbour] = (new_route, new_path_cost) 
                
                # if the neighbour is already on the frontier, then update frontier if the new route is shorter
                else:
                    # collect the path cost of the route already on the frontier
                    existing_cost = frontier[neighbour][1]
                    # compare path cost with new route
                    if new_path_cost < existing_cost:
                        # update frontier with new route if the route is shorter, dropping the older one
                        frontier[neighbour] = (new_route, new_path_cost)
    
    # Amongst the possible routes, return the shortest one
    # Cann allow to review and select alternative routes instead of shortest one
    shortest_route = None
    shortest_distance = None
    for candidate, path_cost in candidates:
        if not shortest_route:
            shortest_route = candidate
            shortest_distance = path_cost
        elif path_cost < shortest_distance:
            shortest_route = candidate
            shortest_distance = path_cost
    for candidate in candidates:
        print(candidate)
    print('Minimum traversing distance to reach goal using Uniform Cost greedy algorithm: {:.2f}'.format(shortest_distance))
    return shortest_route

#################################################################
######################### A* algorithm ##########################
#################################################################

def goaltest(location, goal):
    return location == goal

def find_nearest_frontier_node_AStar(routes, goal, coordinates):
    '''select and return the location on the frontier having the shortest total estimated path_cost to goal.
    The total cost g+f is the sum of :
        - (g) current path cost up to frontier node
        - (f) estimated straight-line distance from frontier node to goal location
    input:
    - routes: dictionary associating nodes on the frontier to a composed of a tuple (route, path length). The tuple contains the sequence of traversed locations from start -> to frontier node and associated path_length
    - goal(int): the goal location
    - coordinates: dictionary with (x,y) coordinates for all nodes in graph
    output:
    - return the location on the frontier having the shortest total estimated path_cost to goal
    '''
    path_costs =  {node:routes[node][1] + calculate_distance(node, goal, coordinates) for node in routes}
    return [node for node, path_cost in sorted(path_costs.items(), key= lambda x: x[1])][0]

# Find shortest route using A* algorithm
def shortest_path(M,start,goal):
    '''this function returns the shortest path between a start location and a goal location using A* algorithm
    inputs:
        - M: a Map object (Class Map)
        - start: the starting node index (integer)
        - goal: the target node index (integer)
    output:
        - a list of integers representing the shortest path from start to goal
        '''
    
    print("shortest path called")
    # informative function indicating the minimum path cost using dijkstra greedy algorithm
    print('Minimum traversing distance to reach goal using Dijkstra greedy algorithm: {:.2f}'.format(dijkstra(M, start, goal)))
    
    # Initialize parameters
    #nodes = M._graph.nodes()           # list of all intersections of the graph M
    coordinates = M.intersections       # dictionary  {node index : [x,y] location coordinates}
    neighbours = M.roads                # adjacency list of list [node_index => [list of neighbours],...]
    explored = set()                    # set for locations marked as explored
    frontier = {start:([start], 0)}     # dictionary associating nodes on the frontier to a composed of a tuple (route, path length). 
                                        # The route is the sequence of traversed locations from start -> to frontier node
    
    # Identify shortest route to the target goal with A* algorithm 
    while len(frontier) > 0:
        
        # while there is node on the frontier, select the one with shortest distance to start (greedy approach)
        current_node = find_nearest_frontier_node_AStar(frontier, goal, coordinates)
        # collect identified route up to this node on the frontier
        current_route, path_cost = frontier[current_node]
        
        # if frontier node with shortest path = goal then we found the shortest path
        if goaltest(current_node, goal):
            break
        
        # remove node from frontier
        frontier.pop(current_node)
        # mark node as explored
        explored.add(current_node)
        
        # visit all neighbours of the selected node on the frontier
        for neighbour in neighbours[current_node]:
            # add neighbour to the route. use deepcopy to preserve current_route unchanged for re-use in for loop
            new_route = deepcopy(current_route)
            new_route.append(neighbour)
            
            # calculate step_cost to transition to the neighbour from frontier node
            step_cost = calculate_distance(current_node, neighbour, coordinates)
            # update path cost with new step cost
            new_path_cost = path_cost + step_cost
            
            # if the neighbour is already explored do nothing and move to next neighbour
            if neighbour not in explored:
                # if the neighbour is not already on the frontier, add it to the frontier with its (route, path_cost)
                if neighbour not in frontier:
                    frontier[neighbour] = (new_route, new_path_cost) 
                
                # if the neighbour is already on the frontier, then update frontier if the new route is shorter
                else:
                    # collect the path cost of the route already on the frontier
                    existing_cost = frontier[neighbour][1]
                    # compare path cost with new route
                    if new_path_cost < existing_cost:
                        # update frontier with new route if the route is shorter, dropping the older one
                        frontier[neighbour] = (new_route, new_path_cost)
    
    # Return the shortest route
    print('Minimum traversing distance to reach goal using A* algorithm: {:.2f}'.format(frontier[goal][1]))
    return current_route


#################################################################
##################### Dynamic Programming #######################
#################################################################

def print_best_path(j, Q, start, goal):
    print('shortest path and distance to target:')
    sum_costs = 0
    current_node = start
    while current_node != goal:
        print(current_node,'->',end=' ')
        # Move to the next node and increment costs
        next_node = np.argmin(Q[current_node, :] + j)
        sum_costs += Q[current_node, next_node]
        current_node = next_node
    print(goal)
    print('Cost: {:.04f}'.format(sum_costs))
        
def shortest_path_DP(M,start,goal):
    '''this function returns the shortest path between a start location and a goal location using Dynamic Programming
    inputs:
        - M: a Map object (Class Map)
        - start: the starting node index (integer)
        - goal: the target node index (integer)
    output:
        - a list of integers representing the shortest path from start to goal
        '''
    
    print("shortest path called")
    # informative function indicating the minimum path cost using dijkstra greedy algorithm
    print('Minimum traversing distance to reach goal using Dijkstra greedy algorithm: {:.2f}'.format(dijkstra(M, start, goal)))
    
    # Initialize parameters
    nodes = M._graph.nodes()               # list of all intersections of the graph M
    coordinates = M.intersections          # dictionary  {node index : [x,y] location coordinates}
    neighbours = M.roads                   # adjacency list of list [node_index => [list of neighbours],...]
    j = np.zeros_like(nodes, dtype='float')
    next_j = np.empty_like(nodes, dtype='float')
    
    # Prepare distance matrix Q. Suqare matrix of size (num_nodes x num_nodes)
    Q = np.ones((len(nodes),len(nodes)))
    # All invalid pairs have infinite distance
    Q = Q * np.inf                         
    # Fill Q with valid graph edge distances
    for node in nodes:
        for neighbour in neighbours[node]:
            if Q[node, neighbour] == np.inf:
                   Q[node, neighbour] = calculate_distance(node, neighbour, coordinates)
    # Initialize goal with 0 distance
    Q[goal, goal] = 0
   
    # Run iterative algorithm to converge towards distance values using Bellman equation
    max_iter = 500
    i=0
    while i < max_iter:
        for nodeA in nodes:
            next_j[nodeA] = np.min(Q[nodeA,:] + j)   # Bellman equation
            # this is equivalent to...
            '''lowest_cost = np.inf
                for nodeB in nodes:
                cost = Q[nodeA, nodeB] + j[nodeB]
                if cost < lowest_cost:
                    lowest_cost = cost
            next_j[nodeA] = lowest_cost'''
        
        # check convergence, else iterate
        #if np.equal(j, next_j).all():   # for integers
        if np.allclose(next_j, j):       # to use with floats
            print('iterations converged after',i,'steps with dynamic programming')
            break
        else:
            j[:] = next_j    # copy contents of next_j to j
            i+=1
    
    # print out results
    print_best_path(j, Q, start, goal)
    
    return Q,j, j[start]
