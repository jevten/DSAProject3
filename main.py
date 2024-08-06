import osmnx as ox
import folium
import time
from collections import deque
import heapq
import tkinter as tk
from tkinter import messagebox
import webbrowser
import os


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

#html file loader for map to open in broswer
def load_map(place_name):
    #global to access across load map and calculate routes
    global G, nodes, edges, node_id_map, initial_map_file
    #add try/exception for cases where place is entered incorrectly or failed loading map data
    try:
        #use osmnx to get graph map data
        G = ox.graph_from_place(place_name, network_type='drive')
        nodes, edges = ox.graph_to_gdfs(G)

        #find cetner of map and make using folium lib
        center_lat, center_lon = (nodes['y'].mean(), nodes['x'].mean())
        m = folium.Map(location=[center_lat, center_lon], zoom_start=14)

        #add edges in map from graph
        for _, row in edges.iterrows():
            folium.PolyLine(locations=[(lat, lon) for lat, lon in zip(row['geometry'].xy[1], row['geometry'].xy[0])],
                            color='blue', weight=2.5, opacity=1).add_to(m)

        #add nodes in map from verticies in graph
        node_id_map = {}
        for i, (node_id, row) in enumerate(nodes.iterrows(), 1):
            folium.CircleMarker(location=(row['y'], row['x']), radius=3, color='red', fill=True,
                                popup=folium.Popup(f"Node {i}", parse_html=True)).add_to(m)
            node_id_map[i] = node_id

        #save map
        initial_map_file = 'map.html'
        m.save(initial_map_file)

        # ensure the file path is correct then display
        map_path = os.path.abspath(initial_map_file)
        webbrowser.open(f'file://{map_path}')
    except Exception as e:
        messagebox.showerror("Error", f"Failed to load map for '{place_name}': {e}")

#functions to handle user input on gui
def get_place_name():
    #handle gui input
    def on_submit():
        #get user input
        place_name = place_name_entry.get().strip()
        if not place_name:
            messagebox.showerror("Input Error", "Please enter a valid place name.")
        else:
            root.destroy()
            load_map(place_name)
            get_node_inputs()
    #create gui window
    root = tk.Tk()
    root.title("Dijkstra's vs BFS Map Route Finder")
    #add text
    tk.Label(root, text="Enter the place name (e.g., 'Gainesville, Florida, USA'):").grid(row=0, column=0, padx=10, pady=5)
    #add user input box
    place_name_entry = tk.Entry(root)
    place_name_entry.grid(row=0, column=1, padx=10, pady=5)

    #call on submit to gather input
    submit_button = tk.Button(root, text="Submit", command=on_submit)
    submit_button.grid(row=1, column=0, columnspan=2, pady=10)

    root.mainloop()


def get_node_inputs():
    #second gui input handler after first map is displayed to gather node inputs
    def on_calculate():
        try:
            #get node numbers for source and destination
            source_node_num = int(source_node_entry.get())
            destination_node_num = int(destination_node_entry.get())
            source_node = node_id_map[source_node_num]
            destination_node = node_id_map[destination_node_num]

            #perform dijsktras using select nodes and graph
            start_time_dijkstra = time.time()
            shortest_path_dijkstra = dijkstra(G, source=source_node, target=destination_node)
            end_time_dijkstra = time.time()
            dijkstra_time = end_time_dijkstra - start_time_dijkstra

            #perform bfs using select nodes and graph
            start_time_bfs = time.time()
            shortest_path_bfs = bfs_path(G, source=source_node, target=destination_node)
            end_time_bfs = time.time()
            bfs_time = end_time_bfs - start_time_bfs

            #create second map for routes
            m = folium.Map(location=[nodes['y'].mean(), nodes['x'].mean()], zoom_start=14)
            for _, row in edges.iterrows():
                folium.PolyLine(locations=[(lat, lon) for lat, lon in zip(row['geometry'].xy[1], row['geometry'].xy[0])],color='white', weight=2.5, opacity=1).add_to(m)
            path_edges_dijkstra = [(shortest_path_dijkstra[i], shortest_path_dijkstra[i + 1]) for i in range(len(shortest_path_dijkstra) - 1)]
            #trace dijkstra's path
            for u, v in path_edges_dijkstra:
                edge_data = G.get_edge_data(u, v)[0]
                if 'geometry' in edge_data:
                    points = [(lat, lon) for lon, lat in edge_data['geometry'].coords]
                else:
                    points = [(G.nodes[u]['y'], G.nodes[u]['x']), (G.nodes[v]['y'], G.nodes[v]['x'])]
                folium.PolyLine(points, color='green', weight=5, opacity=0.7, tooltip="Dijkstra's path").add_to(m)

            path_edges_bfs = [(shortest_path_bfs[i], shortest_path_bfs[i + 1]) for i in range(len(shortest_path_bfs) - 1)]
            #trace bfs route
            for u, v in path_edges_bfs:
                edge_data = G.get_edge_data(u, v)[0]
                if 'geometry' in edge_data:
                    points = [(lat, lon) for lon, lat in edge_data['geometry'].coords]
                else:
                    points = [(G.nodes[u]['y'], G.nodes[u]['x']), (G.nodes[v]['y'], G.nodes[v]['x'])]
                folium.PolyLine(points, color='purple', weight=5, opacity=0.7, tooltip="BFS path").add_to(m)

            #add indicators of source/destination node on map
            folium.CircleMarker(location=(G.nodes[source_node]['y'], G.nodes[source_node]['x']), radius=7, color='blue',fill=True, fill_color='blue', popup='Source').add_to(m)
            folium.CircleMarker(location=(G.nodes[destination_node]['y'], G.nodes[destination_node]['x']), radius=7,color='orange', fill=True, fill_color='orange', popup='Destination').add_to(m)

            #save file
            map_with_paths_file = 'map_with_shortest_paths.html'
            m.save(map_with_paths_file)

            # ensure the file path is correct and display
            map_path = os.path.abspath(map_with_paths_file)
            webbrowser.open(f'file://{map_path}')

            #add time results to gui
            result_text = f"Dijkstra's algorithm execution time: {dijkstra_time:.6f} seconds\n"
            result_text += f"BFS execution time: {bfs_time:.6f} seconds\n"
            result_label.config(text=result_text)

        #error handling
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid numerical values for node numbers.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to calculate paths: {e}")

    #second gui window creation
    root = tk.Tk()
    root.title("Dijkstra's vs BFS Map Route Finder")

    #source input handling
    tk.Label(root, text="Source Node Number:").grid(row=0, column=0, padx=10, pady=5)
    source_node_entry = tk.Entry(root)
    source_node_entry.grid(row=0, column=1, padx=10, pady=5)

    #destination input handling
    tk.Label(root, text="Destination Node Number:").grid(row=1, column=0, padx=10, pady=5)
    destination_node_entry = tk.Entry(root)
    destination_node_entry.grid(row=1, column=1, padx=10, pady=5)

    calculate_button = tk.Button(root, text="Calculate Paths", command=on_calculate)
    calculate_button.grid(row=2, column=0, columnspan=2, pady=10)

    result_label = tk.Label(root, text="", justify="left")
    result_label.grid(row=3, column=0, columnspan=2, padx=10, pady=5)

    root.mainloop()


# start by calling get_place_name
get_place_name()