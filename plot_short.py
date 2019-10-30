#!/usr/bin/env python3

from matplotlib import pyplot as plt
import plot_util

instance_file = "../data/pbn423.tsp"
instance_file = "../data/xrb14233.tsp"
optimal_tour_file = "../data/pbn423.tour"
optimal_tour_file = "../data/xrb14233.tour"
coordinates = plot_util.read_point_file_path(instance_file)
short_edges = plot_util.read_edge_list("output/short_edges.txt")

def plot_edges(coordinates, edges, markers):
    total_length = 0
    for e in edges:
        p1 = coordinates[e[0]]
        p2 = coordinates[e[1]]
        x = [p1[0], p2[0]]
        y = [p1[1], p2[1]]
        plt.plot(x, y, markers)
        dx = x[1] - x[0]
        dy = y[1] - y[0]
        total_length += round((dx ** 2 + dy ** 2) ** 0.5)
    print("total edge length: " + str(total_length))

plot_edges(coordinates, short_edges, "r:x")
plot_util.read_and_plot_tour(coordinates, optimal_tour_file, ":k")

plt.gca().set_aspect("equal")
plt.show()

