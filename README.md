# Comparing Dijkstra's vs BFS Path Finding Algorithims with Map Visualizations using OSMnx and Folium

## Project Overview
TA Note: Commits are from one user... worked together in-person
This project visualizes the shortest paths between nodes on a map using Dijkstra's algorithm and Breadth-First Search (BFS). 
The map data is retrieved using the OSMnx library which creates a graph with all nodes, and the visualization is done using the Folium library. 
The project allows users to select a place, visualize the map, and interactively choose source and destination nodes to compare the performance 
and paths of Dijkstra's and BFS algorithms.

## Features
- Basic GUI for input handling and results
- Retrieve and visualize map data for any specified location.
- Highlight the shortest paths using Dijkstra's algorithm and BFS.
- Interactive selection of source and destination nodes.
- Compare execution times of Dijkstra's and BFS algorithms.
- Display the map with highlighted paths in a web browser.

## Requirements

### Software

- Python 3.6 or higher

### Python Packages

- osmnx: For retrieving and handling OpenStreetMap data.
- folium: For creating interactive maps.
- webbrowser: To open the map in the default web browser.
- collections: For data structures like deque.
- heapq: For implementing the priority queue in Dijkstra's algorithm.
- time: For measuring the execution time of the algorithms.
- tkinter: For creating GUI.

### Installation

You can install the required packages using `pip`. Run the following command in your terminal:

```bash
pip install osmnx folium
macos- python3
pip3 install ....
