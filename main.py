import osmnx as ox
import networkx as nx
import folium
import webbrowser
import time
from collections import deque

# place for which you want to create the graph--can change on user input later
place_name = "New Fairfield, Connecticut, USA"

# retrieve OSM data for the specified place from osmnx lib
G = ox.graph_from_place(place_name, network_type='drive')

#graph's nodes and edges lists
nodes, edges = ox.graph_to_gdfs(G)

# create map using folium
center_lat, center_lon = (nodes['y'].mean(), nodes['x'].mean())
m = folium.Map(location=[center_lat, center_lon], zoom_start=14)

# add edges to the map
for _, row in edges.iterrows():
    folium.PolyLine(locations=[(lat, lon) for lat, lon in zip(row['geometry'].xy[1], row['geometry'].xy[0])],
                    color='blue', weight=2.5, opacity=1).add_to(m)

# add graph nodes with click events to show number
node_id_map = {}
for i, (node_id, row) in enumerate(nodes.iterrows(), 1):
    folium.CircleMarker(location=(row['y'], row['x']), radius=3, color='red', fill=True,
                        popup=folium.Popup(f"Node {i}", parse_html=True)).add_to(m)
    node_id_map[i] = node_id

# save initial map to an HTML file before graph overlay w edges
initial_map_file = 'map.html'
m.save(initial_map_file)
webbrowser.open(initial_map_file)

print("The map has been opened in your web browser.")
print("Enter the node numbers on the map to select the source and destination nodes.")

# user input for source and destination node numbers
source_node_num = int(input("Enter the number of the source node: "))
destination_node_num = int(input("Enter the number of the destination node: "))

# get  node IDs
source_node = node_id_map[source_node_num]
destination_node = node_id_map[destination_node_num]

# Dijkstra's algorithm --->need to implement using netorkx dijkstra rn
start_time_dijkstra = time.time()
shortest_path_dijkstra = nx.shortest_path(G, source=source_node, target=destination_node, weight='length')
end_time_dijkstra = time.time()
dijkstra_time = end_time_dijkstra - start_time_dijkstra

# BFS function
def bfs_edges(G, source):
    visited = set()
    queue = deque([source])
    visited_edges = []
    while queue:
        node = queue.popleft()
        for neighbor in G.neighbors(node):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
                visited_edges.append((node, neighbor))
    return visited_edges

# Find the shortest path using BFS and track visited edges
start_time_bfs = time.time()
visited_edges_bfs = bfs_edges(G, source_node)
shortest_path_bfs = nx.shortest_path(G, source=source_node, target=destination_node)
end_time_bfs = time.time()
bfs_time = end_time_bfs - start_time_bfs

print(f"Dijkstra's algorithm execution time: {dijkstra_time:.6f} seconds")
print(f"BFS execution time: {bfs_time:.6f} seconds")

# highlight the shortest path for Dijkstra's algorithm
path_edges_dijkstra = [(shortest_path_dijkstra[i], shortest_path_dijkstra[i+1]) for i in range(len(shortest_path_dijkstra)-1)]
for u, v in path_edges_dijkstra:
    edge_data = G.get_edge_data(u, v)[0]
    if 'geometry' in edge_data:
        points = [(lat, lon) for lon, lat in edge_data['geometry'].coords]
    else:
        points = [(G.nodes[u]['y'], G.nodes[u]['x']), (G.nodes[v]['y'], G.nodes[v]['x'])]
    folium.PolyLine(points, color='green', weight=5, opacity=0.7, tooltip="Dijkstra's path").add_to(m)

# highlight all visited edges during BFS in pink
for u, v in visited_edges_bfs:
    edge_data = G.get_edge_data(u, v)[0]
    if 'geometry' in edge_data:
        points = [(lat, lon) for lon, lat in edge_data['geometry'].coords]
    else:
        points = [(G.nodes[u]['y'], G.nodes[u]['x']), (G.nodes[v]['y'], G.nodes[v]['x'])]
    folium.PolyLine(points, color='pink', weight=3, opacity=0.7, tooltip="BFS visited edges").add_to(m)

# highlight the shortest path for BFS
path_edges_bfs = [(shortest_path_bfs[i], shortest_path_bfs[i+1]) for i in range(len(shortest_path_bfs)-1)]
for u, v in path_edges_bfs:
    edge_data = G.get_edge_data(u, v)[0]
    if 'geometry' in edge_data:
        points = [(lat, lon) for lon, lat in edge_data['geometry'].coords]
    else:
        points = [(G.nodes[u]['y'], G.nodes[u]['x']), (G.nodes[v]['y'], G.nodes[v]['x'])]
    folium.PolyLine(points, color='purple', weight=5, opacity=0.7, tooltip="BFS path").add_to(m)

# display html file
html_file_with_paths = 'map_with_shortest_paths.html'
m.save(html_file_with_paths)
webbrowser.open(html_file_with_paths)
