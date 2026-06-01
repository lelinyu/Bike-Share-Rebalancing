import json
import os
import csv
import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# --- CONFIGURATION ---
DAYS = ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday']
TIME_BLOCK = "06:00-09:00"
CAPACITIES = [20, 40]
TRUCK_COUNTS = [1, 2, 3, 4, 5]
TIME_LIMITS_MINS = [60, 90, 120, 150, 180]
SERVICE_TIME_SEC = 5 * 60

# --- DATA LOADING ---
base_dir = os.path.dirname(os.path.abspath(__file__))
api_dir = os.path.join(base_dir, 'api') if os.path.exists(os.path.join(base_dir, 'api')) else base_dir

try:
    with open(os.path.join(api_dir, 'matrix.json'), 'r') as f:
        time_matrix = json.load(f)
    with open(os.path.join(api_dir, 'stations.json'), 'r') as f:
        stations = json.load(f)
except FileNotFoundError as e:
    print(f"❌ Error loading data: {e}")
    print("Make sure this script is in the same folder as your 'api' directory!")
    exit()

# Pre-clean the matrix so algorithms don't crash on NaNs
safe_matrix = np.nan_to_num(time_matrix, nan=999999).astype(int).tolist()

def get_station_fluxes(target_day, target_time):
    fluxes = {}
    for node in range(len(stations)):
        if node == 0:
            fluxes[node] = 0
            continue
        try:
            fluxes[node] = stations[node]['flux_profiles'][target_day][target_time]
        except KeyError:
            fluxes[node] = 0
    return fluxes

def evaluate_route_fulfillment(route, fluxes, capacity):
    max_satisfied = 0
    for start_inv in range(capacity + 1):
        satisfied = 0
        current_inv = start_inv
        
        for node in route:
            f = fluxes.get(node, 0)
            if f > 0:  
                take = min(f, capacity - current_inv)
                current_inv += take
                satisfied += take
            elif f < 0:  
                give = min(abs(f), current_inv)
                current_inv -= give
                satisfied += give
                
        if satisfied > max_satisfied:
            max_satisfied = satisfied
    return max_satisfied

def calculate_route_time(routes, matrix):
    total_sec = 0
    for route in routes:
        for i in range(len(route) - 1):
            total_sec += matrix[route[i]][route[i+1]]
            if i > 0 and route[i] != 0: 
                total_sec += SERVICE_TIME_SEC
    return round(total_sec / 60.0, 1)

# --- ALGORITHM 1: NEAREST NEIGHBOR ---
def algo_nearest_neighbor(matrix, num_trucks, time_limit_sec):
    visited = set([0])
    routes = []
    
    for _ in range(num_trucks):
        curr_node = 0
        curr_time = 0
        route = [0]
        
        while True:
            best_node = -1
            best_dist = float('inf')
            
            for nxt in range(1, len(matrix)):
                if nxt not in visited:
                    dist = matrix[curr_node][nxt]
                    return_dist = matrix[nxt][0]
                    
                    if dist < best_dist and (curr_time + dist + SERVICE_TIME_SEC + return_dist) <= time_limit_sec:
                        best_dist = dist
                        best_node = nxt
                        
            if best_node == -1:
                break 
                
            route.append(best_node)
            visited.add(best_node)
            curr_time += best_dist + SERVICE_TIME_SEC
            curr_node = best_node
            
        route.append(0)
        if len(route) > 2:
            routes.append(route)
    return routes

# --- ALGORITHM 2: GREEDY DEMAND ---
def algo_highest_demand(matrix, fluxes, num_trucks, time_limit_sec):
    visited = set([0])
    routes = []
    
    for _ in range(num_trucks):
        curr_node = 0
        curr_time = 0
        route = [0]
        
        while True:
            best_node = -1
            best_flux_magnitude = -1
            best_dist = float('inf')
            
            for nxt in range(1, len(matrix)):
                if nxt not in visited:
                    dist = matrix[curr_node][nxt]
                    return_dist = matrix[nxt][0]
                    flux_mag = abs(fluxes.get(nxt, 0))
                    
                    if (curr_time + dist + SERVICE_TIME_SEC + return_dist) <= time_limit_sec:
                        if flux_mag > best_flux_magnitude:
                            best_flux_magnitude = flux_mag
                            best_node = nxt
                            best_dist = dist
                        elif flux_mag == best_flux_magnitude and dist < best_dist:
                            best_node = nxt
                            best_dist = dist
                            
            if best_node == -1:
                break
                
            route.append(best_node)
            visited.add(best_node)
            curr_time += best_dist + SERVICE_TIME_SEC
            curr_node = best_node
            
        route.append(0)
        if len(route) > 2:
            routes.append(route)
    return routes

# --- ALGORITHM 3: OR-TOOLS ---
def algo_ortools(matrix, fluxes, num_trucks, capacity, time_limit_sec):
    demands = []
    penalties = []
    
    for node in range(len(matrix)):
        if node == 0:
            demands.append(0)
            penalties.append(0)
            continue
            
        raw_flux = fluxes.get(node, 0)
        capped_flux = max(min(raw_flux, capacity), -capacity)
        demands.append(capped_flux)
        penalties.append(int(abs(raw_flux) * 10000))

    data = {
        'time_matrix': matrix,
        'demands': demands,
        'num_vehicles': num_trucks,
        'vehicle_capacity': capacity,
        'depot': 0, 
        'max_time_seconds': time_limit_sec,
        'service_time_seconds': SERVICE_TIME_SEC
    }

    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        driving_time = data['time_matrix'][from_node][to_node]
        if from_node == to_node: return 0
        return driving_time + data['service_time_seconds']

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    routing.AddDimension(transit_callback_index, 0, data['max_time_seconds'], True, 'Time')

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimension(demand_callback_index, 0, data['vehicle_capacity'], False, 'Capacity')

    for node in range(len(data['time_matrix'])):
        if node == data['depot']: continue
        routing.AddDisjunction([manager.NodeToIndex(node)], penalties[node])

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.SAVINGS
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    
    # Strictly limit OR-Tools to 1 second so the batch processor runs quickly
    search_parameters.time_limit.seconds = 1 

    solution = routing.SolveWithParameters(search_parameters)
    routes = []
    
    if solution:
        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            route = []
            
            while not routing.IsEnd(index):
                route.append(manager.IndexToNode(index))
                index = solution.Value(routing.NextVar(index))
            
            route.append(manager.IndexToNode(index))
            if len(route) > 2:
                routes.append(route)
                
    return routes

# --- MAIN EXECUTION ---
if __name__ == "__main__":
    output = []
    
    print("🚀 Running Tri-Algorithm Comparison...")
    total_permutations = len(DAYS) * len(CAPACITIES) * len(TIME_LIMITS_MINS) * len(TRUCK_COUNTS)
    print(f"Executing {total_permutations} configurations. Please wait...")

    for day in DAYS:
        fluxes = get_station_fluxes(day, TIME_BLOCK)
        
        for capacity in CAPACITIES:
            for limit_mins in TIME_LIMITS_MINS:
                limit_sec = limit_mins * 60
                
                for trucks in TRUCK_COUNTS:
                    # 1. Nearest Neighbor
                    nn_routes = algo_nearest_neighbor(safe_matrix, trucks, limit_sec)
                    nn_sat = sum(evaluate_route_fulfillment(r, fluxes, capacity) for r in nn_routes)
                    nn_vis = sum(len(r)-2 for r in nn_routes) 
                    nn_time = calculate_route_time(nn_routes, safe_matrix)
                    
                    # 2. Greedy Demand
                    gd_routes = algo_highest_demand(safe_matrix, fluxes, trucks, limit_sec)
                    gd_sat = sum(evaluate_route_fulfillment(r, fluxes, capacity) for r in gd_routes)
                    gd_vis = sum(len(r)-2 for r in gd_routes)
                    gd_time = calculate_route_time(gd_routes, safe_matrix)

                    # 3. OR-Tools
                    or_routes = algo_ortools(safe_matrix, fluxes, trucks, capacity, limit_sec)
                    or_sat = sum(evaluate_route_fulfillment(r, fluxes, capacity) for r in or_routes)
                    or_vis = sum(len(r)-2 for r in or_routes)
                    or_time = calculate_route_time(or_routes, safe_matrix)

                    # Calculate AI Improvement Percentage
                    best_baseline = max(nn_sat, gd_sat)
                    if best_baseline > 0:
                        improvement = round(((or_sat - best_baseline) / best_baseline) * 100, 1)
                    elif or_sat > 0:
                        improvement = 100.0
                    else:
                        improvement = 0.0

                    output.append({
                        "Day": day,
                        "Time_Block": TIME_BLOCK,
                        "Capacity": capacity,
                        "Trucks": trucks,
                        "Time_Limit_Mins": limit_mins,
                        "NN_Satisfied": nn_sat,
                        "NN_Visited": nn_vis,
                        "NN_Time": nn_time,
                        "GD_Satisfied": gd_sat,
                        "GD_Visited": gd_vis,
                        "GD_Time": gd_time,
                        "OR_Satisfied": or_sat,
                        "OR_Visited": or_vis,
                        "OR_Time": or_time,
                        "OR_Improvement_%": improvement
                    })

    # --- SAVE TO CSV ---
    output_filename = "algorithm_comparison_results.csv"
    with open(output_filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=output[0].keys())
        writer.writeheader()
        writer.writerows(output)

    print(f"\n✅ All testing complete!")
    print(f"📁 Detailed metrics for all {total_permutations} combinations saved to '{output_filename}'.")