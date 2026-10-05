# Required imports
import numpy as np
import networkx as nx
from Boundaries import Boundaries
from Map import EPSILON

# Number of nodes expanded in the heuristic search (stored in a global variable to be updated from the heuristic functions)
NODES_EXPANDED = 0

def h1(current_node, objective_node) -> np.float32:
    """
    Primera heurística admisible: Distancia Manhattan entre nodos
    Esta heurística es admisible porque la distancia Manhattan nunca sobreestima
    el coste real, ya que el coste mínimo posible de cada paso es EPSILON
    """
    global NODES_EXPANDED
    
    # Parse node strings to tuples of integers
    current = eval(current_node)
    objective = eval(objective_node)
    
    # Calculate Manhattan distance (|x1-x2| + |y1-y2|)
    manhattan_distance = abs(current[0] - objective[0]) + abs(current[1] - objective[1])
    
    # Multiply by minimum cell cost to ensure admissibility
    h = manhattan_distance * EPSILON
    
    NODES_EXPANDED += 1
    return h

def h2(current_node, objective_node) -> np.float32:
    """
    Segunda heurística admisible: Distancia Euclidiana entre nodos
    Esta heurística es admisible porque la distancia euclidiana es siempre menor o igual
    que el camino real, y multiplicamos por EPSILON (mínimo coste posible)
    """
    global NODES_EXPANDED
    
    # Parse node strings to tuples of integers
    current = eval(current_node)
    objective = eval(objective_node)
    
    # Calculate Euclidean distance (sqrt((x1-x2)^2 + (y1-y2)^2))
    euclidean_distance = np.sqrt((current[0] - objective[0])**2 + (current[1] - objective[1])**2)
    
    # Multiply by minimum cell cost to ensure admissibility
    h = euclidean_distance * EPSILON
    
    NODES_EXPANDED += 1
    return h

def build_graph(detection_map: np.array, tolerance: np.float32) -> nx.DiGraph:
    """
    Builds an adjacency graph (not an adjacency matrix) from the detection map
    The only possible connections from a point in space (now a node in the graph) are:
      -> Go up
      -> Go down
      -> Go left
      -> Go right
    Not every point has always 4 possible neighbors
    """
    # Create empty directed graph
    G = nx.DiGraph()
    
    # Get dimensions of the detection map
    height, width = detection_map.shape
    
    # Iterate through each cell in the detection map
    for i in range(height):
        for j in range(width):
            # Current node coordinates as string representation
            current_node = str((i, j))
            
            # Skip nodes with detection values above tolerance threshold
            if detection_map[i, j] > tolerance:
                continue
            
            # Add the current node to the graph
            G.add_node(current_node)
            
            # Check and add edges to neighboring cells (if within bounds and below tolerance)
            
            # Up neighbor (i-1, j)
            if i > 0 and detection_map[i-1, j] <= tolerance:
                neighbor = str((i-1, j))
                G.add_edge(current_node, neighbor, weight=detection_map[i-1, j])
            
            # Down neighbor (i+1, j)
            if i < height-1 and detection_map[i+1, j] <= tolerance:
                neighbor = str((i+1, j))
                G.add_edge(current_node, neighbor, weight=detection_map[i+1, j])
            
            # Left neighbor (i, j-1)
            if j > 0 and detection_map[i, j-1] <= tolerance:
                neighbor = str((i, j-1))
                G.add_edge(current_node, neighbor, weight=detection_map[i, j-1])
            
            # Right neighbor (i, j+1)
            if j < width-1 and detection_map[i, j+1] <= tolerance:
                neighbor = str((i, j+1))
                G.add_edge(current_node, neighbor, weight=detection_map[i, j+1])
    
    return G

def discretize_coords(high_level_plan: np.array, boundaries: Boundaries, map_width: np.int32, map_height: np.int32) -> np.array:
    """
    Converts coordinates from (lat, lon) into (row, col) indices in the detection map
    """
    # Create array to store discretized coordinates
    discretized_coords = np.zeros_like(high_level_plan, dtype=np.int32)
    
    for i in range(len(high_level_plan)):
        # Calculate row index (latitude)
        lat_range = boundaries.max_lat - boundaries.min_lat
        lat_normalized = (high_level_plan[i, 0] - boundaries.min_lat) / lat_range
        row = int(lat_normalized * (map_height - 1))
        
        # Calculate column index (longitude)
        lon_range = boundaries.max_lon - boundaries.min_lon
        lon_normalized = (high_level_plan[i, 1] - boundaries.min_lon) / lon_range
        col = int(lon_normalized * (map_width - 1))
        
        # Store discretized coordinates
        discretized_coords[i, 0] = row
        discretized_coords[i, 1] = col
    
    return discretized_coords

def path_finding(G: nx.DiGraph,
                 heuristic_function,
                 locations: np.array, 
                 initial_location_index: np.int32, 
                 boundaries: Boundaries,
                 map_width: np.int32,
                 map_height: np.int32) -> tuple:
    """
    Implementation of the main searching / path finding algorithm
    """
    global NODES_EXPANDED
    NODES_EXPANDED = 0  # Reset expanded nodes counter
    
    # Convert high-level plan coordinates (lat, lon) to grid indices (row, col)
    discretized_locations = discretize_coords(high_level_plan=locations, 
                                             boundaries=boundaries, 
                                             map_width=map_width, 
                                             map_height=map_height)
    
    # Create an empty solution plan
    solution_plan = []
    
    # Define the visiting order (starting from initial_location_index)
    num_locations = len(locations)
    visit_order = [(initial_location_index + i) % num_locations for i in range(num_locations)]
    
    # For each consecutive pair of locations in the visit order
    for i in range(len(visit_order) - 1):
        start_idx = visit_order[i]
        end_idx = visit_order[i + 1]
        
        # Get grid coordinates for start and end points
        start_coords = discretized_locations[start_idx]
        end_coords = discretized_locations[end_idx]
        
        # Convert numpy int32 values to Python int and create string representation to match graph nodes
        # This is the fix for the NodeNotFound error
        start_node = str((int(start_coords[0]), int(start_coords[1])))
        end_node = str((int(end_coords[0]), int(end_coords[1])))
        
        try:
            # Check if nodes exist in the graph
            if start_node not in G:
                print(f"Start node {start_node} not found in graph!")
                return [], NODES_EXPANDED
            
            if end_node not in G:
                print(f"End node {end_node} not found in graph!")
                return [], NODES_EXPANDED
            
            # Use A* to find shortest path between current pair of locations
            path = nx.astar_path(G=G, 
                                source=start_node, 
                                target=end_node, 
                                heuristic=heuristic_function, 
                                weight='weight')
            
            # Add path to solution plan
            solution_plan.append(path)
            
        except nx.NetworkXNoPath:
            print(f"No path found between {start_node} and {end_node}!")
            # Return empty solution if no path is found
            return [], NODES_EXPANDED
        except Exception as e:
            print(f"Error finding path: {e}")
            return [], NODES_EXPANDED
    
    return solution_plan, NODES_EXPANDED

def compute_path_cost(G: nx.DiGraph, solution_plan: list) -> np.float32:
    """
    Computes the total cost of the whole planning solution
    """
    total_cost = 0.0
    
    # Iterate through each segment in the solution plan
    for path_segment in solution_plan:
        # Add up the costs of transitions between consecutive nodes
        for i in range(len(path_segment) - 1):
            current_node = path_segment[i]
            next_node = path_segment[i + 1]
            
            # Get the edge weight (detection level) between these nodes
            edge_weight = G[current_node][next_node]['weight']
            total_cost += edge_weight
    
    return total_cost