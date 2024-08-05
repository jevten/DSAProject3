import osmnx as ox
import folium
import webbrowser
import time
from collections import deque
import heapq

#dijkstra's implementation inspired from 8b slide pseudocode
def dijkstra(G, source, target):
    #priority queue for tuples dij implementation
    queue = [(0, source)]
    #dict to store prev node and distances
    distances = {node: float('infinity') for node in G.nodes}
    distances[source] = 0
    previous_nodes = {node: None for node in G.nodes}

    # Update the distance for each neighbor whil queue not empty
    while queue:
        current_distance, current_node = heapq.heappop(queue)
        if current_distance > distances[current_node]:
            continue
        for neighbor, edge_data in G[current_node].items():
            distance = current_distance + edge_data[0]['length']
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(queue, (distance, neighbor))
    # Reconstruct the shortest path
    path, current_node = deque(), target
    while previous_nodes[current_node] is not None:
        path.appendleft(current_node)
        current_node = previous_nodes[current_node]
    path.appendleft(source)

    return list(path)

#bfs implementation inspired from 8a slide module
def bfs_path(G, source, target):
    #queue to manage bfs
    queue = deque([source])
    #dict to track visited nodes
    visited = {source: None}

    #standard bfs implementation get first node and remove from queue then visit neighbors and add to queue
    while queue:
        node = queue.popleft()
        if node == target:
            break
        for neighbor in G.neighbors(node):
            if neighbor not in visited:
                visited[neighbor] = node
                queue.append(neighbor)

    #construct path from source to destnation node
    path = []
    #if found constuct
    if target in visited:
        current_node = target
        while current_node is not None:
            path.append(current_node)
            current_node = visited[current_node]
            #reverse to start from source node
        path.reverse()
    return path

# User input for place name
place_name = input("Enter the place name (e.g., 'New Fairfield, Connecticut, USA'): ")

# Retrieve OSM data for the specified place from osmnx library
G = ox.graph_from_place(place_name, network_type='drive')

# Graph's nodes and edges lists from osmnx
nodes, edges = ox.graph_to_gdfs(G)

# Create map using folium library
center_lat, center_lon = (nodes['y'].mean(), nodes['x'].mean())
m = folium.Map(location=[center_lat, center_lon], zoom_start=14)

# Add edges to the map using folium
for _, row in edges.iterrows():
    folium.PolyLine(locations=[(lat, lon) for lat, lon in zip(row['geometry'].xy[1], row['geometry'].xy[0])],
                    color='blue', weight=2.5, opacity=1).add_to(m)

# Add graph nodes with click events to show node id
node_id_map = {}
for i, (node_id, row) in enumerate(nodes.iterrows(), 1):
    folium.CircleMarker(location=(row['y'], row['x']), radius=3, color='red', fill=True,
                        popup=folium.Popup(f"Node {i}", parse_html=True)).add_to(m)
    node_id_map[i] = node_id

# Save initial map to an HTML file before graph overlay with edges to open in maps
initial_map_file = 'map.html'
m.save(initial_map_file)
webbrowser.open(initial_map_file)

print("The map has been opened in your web browser.")
print("Enter the node numbers on the map to select the source and destination nodes.")

# User input for source and destination node numbers
source_node_num = int(input("Enter the number of the source node: "))
destination_node_num = int(input("Enter the number of the destination node: "))

# Get node IDs from user input
source_node = node_id_map[source_node_num]
destination_node = node_id_map[destination_node_num]

# Dijkstra's algorithm graph add timer to compare
start_time_dijkstra = time.time()
shortest_path_dijkstra = dijkstra(G, source=source_node, target=destination_node)
end_time_dijkstra = time.time()
dijkstra_time = end_time_dijkstra - start_time_dijkstra

# BFS function on graph add timer to compare
start_time_bfs = time.time()
shortest_path_bfs = bfs_path(G, source_node, destination_node)
end_time_bfs = time.time()
bfs_time = end_time_bfs - start_time_bfs

#display algorithim times to compare
print()
print(f"Dijkstra's algorithm execution time: {dijkstra_time:.6f} seconds")
print(f"BFS execution time: {bfs_time:.6f} seconds")

#clear the map to prepare for the final map with highlighted paths
m = folium.Map(location=[center_lat, center_lon], zoom_start=14)
#remove the blue edges in the map so final map with routes isnt cluttered
for _, row in edges.iterrows():
    folium.PolyLine(locations=[(lat, lon) for lat, lon in zip(row['geometry'].xy[1], row['geometry'].xy[0])],
                    color='white', weight=2.5, opacity=1).add_to(m)

#highlight source and destination nodes
folium.CircleMarker(location=(G.nodes[source_node]['y'], G.nodes[source_node]['x']), radius=7, color='blue', fill=True, fill_color='blue', popup='Source').add_to(m)
folium.CircleMarker(location=(G.nodes[destination_node]['y'], G.nodes[destination_node]['x']), radius=7, color='orange', fill=True, fill_color='orange', popup='Destination').add_to(m)

# Highlight the shortest path for Dijkstra's algorithm in folium
path_edges_dijkstra = [(shortest_path_dijkstra[i], shortest_path_dijkstra[i+1]) for i in range(len(shortest_path_dijkstra)-1)]
for u, v in path_edges_dijkstra:
    edge_data = G.get_edge_data(u, v)[0]
    if 'geometry' in edge_data:
        points = [(lat, lon) for lon, lat in edge_data['geometry'].coords]
    else:
        points = [(G.nodes[u]['y'], G.nodes[u]['x']), (G.nodes[v]['y'], G.nodes[v]['x'])]
    folium.PolyLine(points, color='green', weight=5, opacity=0.7, tooltip="Dijkstra's path").add_to(m)

# Highlight the shortest path for BFS in folium
path_edges_bfs = [(shortest_path_bfs[i], shortest_path_bfs[i+1]) for i in range(len(shortest_path_bfs)-1)]
for u, v in path_edges_bfs:
    edge_data = G.get_edge_data(u, v)[0]
    if 'geometry' in edge_data:
        points = [(lat, lon) for lon, lat in edge_data['geometry'].coords]
    else:
        points = [(G.nodes[u]['y'], G.nodes[u]['x']), (G.nodes[v]['y'], G.nodes[v]['x'])]
    folium.PolyLine(points, color='purple', weight=5, opacity=0.7, tooltip="BFS path").add_to(m)

# Display HTML file
print()
print("Displaying updated map in browser.")
print("Blue circle represents source node, Orange represents destination node")
print("***In event of one path, it means two paths overlap***")
html_file_with_paths = 'map_with_shortest_paths.html'
m.save(html_file_with_paths)
webbrowser.open(html_file_with_paths)
