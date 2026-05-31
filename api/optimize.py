from http.server import BaseHTTPRequestHandler
import json
import os
from pathlib import Path
import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# Define the service time buffer (in minutes)
SERVICE_TIME_MINUTES = 5 

class handler(BaseHTTPRequestHandler):

    def do_GET(self):
        """Serve the UI on root path and JSON status elsewhere."""
        if self.path in ('/', '/index.html'):
            index_path = Path(__file__).resolve().parent.parent / 'index.html'
            try:
                html = index_path.read_text(encoding='utf-8')
                self.send_response(200)
                self.send_header('Content-type', 'text/html; charset=utf-8')
                self.end_headers()
                self.wfile.write(html.encode('utf-8'))
                return
            except OSError:
                self.send_response(500)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'error': 'Could not load index.html'}).encode('utf-8'))
                return

        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps({
            'status': 'API is running successfully!',
            'message': 'Please send POST requests with truck parameters to calculate routes.'
        }).encode('utf-8'))
    
    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        user_inputs = json.loads(post_data)
        
        minutes_allowed = int(user_inputs.get('minutes', 120))
        num_trucks = int(user_inputs.get('trucks', 3))
        truck_capacity = int(user_inputs.get('capacity', 20))
        target_day = user_inputs.get('day', 'Monday')
        target_time = user_inputs.get('time_block', '06:00-09:00')
        
        base_dir = os.path.dirname(os.path.abspath(__file__))
        try:
            with open(os.path.join(base_dir, 'matrix.json'), 'r') as f:
                time_matrix = json.load(f)
            with open(os.path.join(base_dir, 'stations.json'), 'r') as f:
                stations = json.load(f)
        except Exception as e:
            self.send_error(500, f"Error loading data files: {str(e)}")
            return

        safe_matrix = np.nan_to_num(time_matrix, nan=999999) 
        
        demands = []
        penalties = []
        for node in range(len(stations)):
            if node == 0:  # Depot
                demands.append(0)
                penalties.append(0)
                continue
                
            try:
                raw_flux = stations[node]['flux_profiles'][target_day][target_time]
            except KeyError:
                raw_flux = 0
                
            # Capacity Capping: Force demand to fit in the truck
            capped_flux = max(min(raw_flux, truck_capacity), -truck_capacity)
            demands.append(capped_flux)
            
            # Penalty based on raw flux so the AI prioritizes massive shortages
            penalties.append(int(abs(raw_flux) * 10000))

        data = {
            'time_matrix': safe_matrix.astype(int).tolist(),
            'demands': demands,
            'num_vehicles': num_trucks,
            'vehicle_capacity': truck_capacity,
            'depot': 0, 
            'max_time_seconds': int(minutes_allowed * 60),
            'service_time_seconds': int(SERVICE_TIME_MINUTES * 60)
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

        routing.AddDimension(
            transit_callback_index,
            0, data['max_time_seconds'], True, 'Time'
        )

        def demand_callback(from_index):
            from_node = manager.IndexToNode(from_index)
            return data['demands'][from_node]

        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
        routing.AddDimension(
            demand_callback_index,
            0, data['vehicle_capacity'], False, 'Capacity'
        )

        # --- THE BUG FIX IS HERE ---
        for node in range(len(data['time_matrix'])):
            if node == data['depot']: continue
            
            # By removing the "if penalties > 0" check, we guarantee EVERY station is optional.
            # Stations with 0 bikes get a 0 penalty and are instantly, safely ignored.
            routing.AddDisjunction([manager.NodeToIndex(node)], penalties[node])
        # ---------------------------

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        # Upgraded to SAVINGS algorithm, which is heavily optimized for capacity constraints
        search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.SAVINGS
        search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        search_parameters.time_limit.seconds = 3 

        solution = routing.SolveWithParameters(search_parameters)

        all_routes = []
        
        if solution:
            for vehicle_id in range(data['num_vehicles']):
                index = routing.Start(vehicle_id)
                truck_route = []
                
                while not routing.IsEnd(index):
                    node_index = manager.IndexToNode(index)
                    lon = stations[node_index]['lon']
                    lat = stations[node_index]['lat']
                    truck_route.append([lon, lat])
                    index = solution.Value(routing.NextVar(index))
                
                depot_node = manager.IndexToNode(index)
                truck_route.append([stations[depot_node]['lon'], stations[depot_node]['lat']])
                
                if len(truck_route) > 2:
                    all_routes.append(truck_route)

        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps({'routes': all_routes}).encode('utf-8'))