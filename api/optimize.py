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
        # 1. Read the inputs sent from the frontend
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        user_inputs = json.loads(post_data)
        
        minutes_allowed = int(user_inputs.get('minutes', 60))
        num_trucks = int(user_inputs.get('trucks', 1))
        
        # 2. Load your pre-saved JSON data
        # We use os.path to safely locate the JSON files in the same directory as this script
        base_dir = os.path.dirname(os.path.abspath(__file__))
        
        try:
            with open(os.path.join(base_dir, 'matrix.json'), 'r') as f:
                time_matrix = json.load(f)
            with open(os.path.join(base_dir, 'stations.json'), 'r') as f:
                stations = json.load(f)
        except Exception as e:
            self.send_error(500, f"Error loading data files: {str(e)}")
            return

        # 3. Run the OR-Tools Routing code
        safe_matrix = np.nan_to_num(time_matrix, nan=999999) 
        data = {
            'time_matrix': safe_matrix.astype(int).tolist(),
            'num_vehicles': num_trucks,
            'depot': 0, # Assuming first station is the depot
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
            0,
            data['max_time_seconds'],
            True,
            'Time'
        )

        # Add Penalties for skipping (based on flux)
        for node in range(len(data['time_matrix'])):
            if node == data['depot']: continue
            # Handle potential dict structures if pandas exported to JSON records
            flux = stations[node]['net_flux'] if isinstance(stations[node], dict) else stations[node][1] 
            penalty = int(abs(flux) * 10000)
            routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        search_parameters.time_limit.seconds = 3 # Keep it fast for the web!

        solution = routing.SolveWithParameters(search_parameters)

        # 4. Format the output as a list of GPS routes
        all_routes = []
        
        if solution:
            for vehicle_id in range(data['num_vehicles']):
                index = routing.Start(vehicle_id)
                truck_route = []
                
                while not routing.IsEnd(index):
                    node_index = manager.IndexToNode(index)
                    # Extract Longitude and Latitude for this station
                    lon = stations[node_index]['lon']
                    lat = stations[node_index]['lat']
                    truck_route.append([lon, lat])
                    index = solution.Value(routing.NextVar(index))
                
                # Add the final return trip to the depot
                depot_node = manager.IndexToNode(index)
                truck_route.append([stations[depot_node]['lon'], stations[depot_node]['lat']])
                
                # Only send the route if the truck actually left the depot (more than 2 stops)
                if len(truck_route) > 2:
                    all_routes.append(truck_route)

        # 5. Send the data back to the frontend
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        
        # We send back an array of routes (since we have multiple trucks)
        self.wfile.write(json.dumps({'routes': all_routes}).encode('utf-8'))