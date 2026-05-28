from http.server import BaseHTTPRequestHandler
import json
# Import OR-Tools and your routing logic here...

class handler(BaseHTTPRequestHandler):

    def do_GET(self):
        """Handle standard browser requests gracefully."""
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
        
        minutes_allowed = user_inputs.get('minutes', 60)
        num_trucks = user_inputs.get('trucks', 1)
        
        # 2. Load your pre-saved JSON data (matrix.json, stations.json)
        
        # 3. Run the OR-Tools Routing code...
        
        # 4. Format the output as a list of GPS coordinates for the map
        route_coordinates = [
            [-118.258, 34.048], # Example Long/Lat
            [-118.254, 34.050]
        ]
        
        # 5. Send the data back to the frontend
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps({'route': route_coordinates}).encode('utf-8'))