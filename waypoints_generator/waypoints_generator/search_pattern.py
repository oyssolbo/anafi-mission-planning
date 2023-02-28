#!/bin/env/python3

# Initially developed by S. Allum
# Modified by Ø. Solbø

# This file is somewhat of a mess. Be careful!

import math
import pyproj
import plotly.express as px
import plotly.graph_objs as go
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pymap3d
import os
from queue import PriorityQueue
from copy import copy


def expanding_square_search_ned(
      altitude    : float, 
      camera_fov  : float, 
      overlap     : float, 
      area_size   : tuple
    ) -> np.ndarray:
	"""
	Based on the methodology for generating waypoints, but the order
	of visit is determined based on the sequence of the waypoint.

	The methodology here is quite simple, and there is little usage of
	automatic generating the waypoints, such as in generate_waypoints_...
	The reason is to avoid the travelling-salesman problem 
	"""

	search_positions = np.array([[0, 0, -altitude]])

	# Testing required to determine the lower positive limit
	if overlap >= 0.75 or overlap < 0: 
		return search_positions 
	
	area_size_north = area_size[0]
	area_size_east = area_size[1]

	# Should also prevent the search distance to become too large
	if area_size_north <= 1 or area_size_east <= 1:
		return search_positions

	hfov = camera_fov[0]
	vfov = camera_fov[1]

	# Calculate coverage area for each photo
	h_coverage = (altitude * np.tan(np.deg2rad(hfov / 2))) ** 2
	v_coverage = (altitude * np.tan(np.deg2rad(vfov / 2))) ** 2

	h_coverage_overlap = (1 - overlap) * h_coverage
	v_coverage_overlap = (1 - overlap) * v_coverage

	search_side_length = np.min([h_coverage_overlap, v_coverage_overlap])

	# Continue until the area is covered 
	current_n_pos, current_e_pos = 0, 0
	search_distance_north, search_distance_east = area_size_north / 2.0, area_size_east / 2.0

	length_multiplier = 1
	side_sign = 1
	side_idx = 0

	while abs(current_n_pos) < search_distance_north or abs(current_e_pos) < search_distance_east:
		
		# Must stop at every side length to search
		for stop_idx in range(1, length_multiplier + 1):

			if side_idx % 2 == 1:
				e_pos = current_e_pos + stop_idx * search_side_length * side_sign
				n_pos = current_n_pos

			else:
				n_pos = current_n_pos + stop_idx * search_side_length * side_sign
				e_pos = current_e_pos

			next_pos = [copy(n_pos), copy(e_pos), -altitude] # Must copy because fuck python
			search_positions = np.vstack([search_positions, [next_pos]])

		if side_idx % 2 == 1:
			length_multiplier += 1
			side_sign *= (-1)

			current_e_pos = e_pos
		else:
			current_n_pos = n_pos

		side_idx += 1

	return search_positions


def line_search_lla(
      altitude    : float, 
      camera_fov  : float, 
      overlap     : float, 
      area_size   : tuple,
      lla_origin  : tuple
    ) -> np.ndarray:
	lla_points = generate_waypoints_lla(altitude, camera_fov, overlap, area_size, lla_origin, False)
	return traverse_lines_lla(lla_points, 1e-5, 1e-5)


def generate_waypoints_en(
      altitude    : float, 
      camera_fov  : float, 
      overlap     : float, 
      area_size   : tuple
    ) -> np.ndarray:
  hfov = camera_fov[0]
  vfov = camera_fov[1]

  # Calculate coverage area for each photo
  h_coverage = (altitude * np.tan(np.deg2rad(hfov / 2))) ** 2
  v_coverage = (altitude * np.tan(np.deg2rad(vfov / 2))) ** 2

  h_coverage_overlap = (1 - overlap) * h_coverage
  v_coverage_overlap = (1 - overlap) * v_coverage

  east_north_points = generate_east_north_points((h_coverage_overlap, v_coverage_overlap), area_size)
  return east_north_points


def generate_waypoints_ned(
      altitude    : float, 
      camera_fov  : float, 
      overlap     : float, 
      area_size   : tuple,
      add_origin  : bool
    ) -> np.ndarray:
	east_north_points = generate_waypoints_en(altitude, camera_fov, overlap, area_size)
	if add_origin:
		east_north_points = np.vstack([east_north_points, [[0, 0]]])

	north_east_points = east_north_points[:, [1, 0]]
	altitudes = np.full((north_east_points.shape[0], 1), -altitude)
	ned_points = np.hstack([north_east_points, altitudes])

	# plot_grid_and_points((h_coverage, v_coverage), area_size, east_north_points)
	return ned_points


def generate_waypoints_lla(
      altitude    : float, 
      camera_fov  : float, 
      overlap     : float, 
      area_size   : tuple, 
      lla_origin  : tuple,
      add_origin  : bool
    ) -> np.ndarray:
	east_north_points = generate_waypoints_en(altitude, camera_fov, overlap, area_size)

	if add_origin:
		east_north_points = np.vstack([east_north_points, [[0, 0]]])

	# plot_grid_and_points((h_coverage, v_coverage), area_size, east_north_points)

	LLA_points = east_north_to_lat_lon_alt(east_north_points, 5, lla_origin[0], lla_origin[1])

	return LLA_points


def east_north_to_lat_lon_alt(
		  en_coordinates    : np.ndarray, 
      altitude          : float, 
      local_origin_lat  : float , 
      local_origin_lon  : float 
    ) -> np.ndarray:
	ell_wgs84 = pymap3d.Ellipsoid('wgs84')

	# Convert the XY coordinates to longitude, latitude, and altitude
	lat_lon_alt = []
	for east, north in en_coordinates:
		lat1, lon1, h1 = pymap3d.ned2geodetic(north, east, altitude, \
						local_origin_lat, local_origin_lon, altitude, \
						ell=ell_wgs84, deg=True)  # wgs84 ellisoid
		lat_lon_alt.append((lat1, lon1, altitude))

	return lat_lon_alt


def generate_east_north_points(
		  grid_size : tuple, 
      area_size : tuple 
    ) -> np.ndarray:
	x_points = np.arange(-area_size[0]/2, area_size[0]/2, grid_size[0])
	y_points = np.arange(-area_size[1]/2, area_size[1]/2, grid_size[1])

	sample_points = np.transpose([np.tile(x_points, len(y_points)), np.repeat(y_points, len(x_points))])

	sample_points_sorted = sort_points_by_distance_to_origin(sample_points)

	return sample_points_sorted


def sort_points_by_distance_to_origin(points : np.ndarray) -> np.ndarray:
	distances = np.linalg.norm(points, axis=1)
	sorted_indices = np.argsort(distances)
	return points[sorted_indices]


def get_fov_from_hfov(
		  image_width   : int, 
      image_height  : int, 
      hfov          : float
    ) -> tuple:
	aspect_ratio = image_width / image_height
	vfov = math.degrees(2 * math.atan(math.tan(math.radians(hfov / 2)) / aspect_ratio))

	return (hfov, vfov)


def plot_grid_and_points(
		  grid_size     : tuple, 
      area_size     : tuple, 
      sample_points : np.ndarray
    ) -> None:
	fig, ax = plt.subplots()
	x_step = grid_size[0]
	y_step = grid_size[1]
	colors = np.random.rand(len(sample_points),3)
	for i, point in enumerate(sample_points):
		ax.add_patch(
			patches.Rectangle(
				(point[0] - (x_step / 2), point[1] - (y_step / 2)), x_step, y_step,
				facecolor=colors[i],
				edgecolor='black',
				alpha=0.1,
				linewidth=1
			)
		)
	ax.scatter(sample_points[:,0], sample_points[:,1], color='black')
	ax.set_title("Search-boxes")
	ax.set_xlabel("East")
	ax.set_ylabel("North")
	plt.show()


def plot_traversal_on_map(list, distressed_coordinate=None):

		df = pd.DataFrame(list, columns=["Lat", "Long", "Alt"])

		df.reset_index(inplace=True)

		start = df.iloc[0]
		stop = df.iloc[-1]

		fig = go.Figure()

		color_scale = ['orange', 'red']

		fig.add_scattermapbox(lat=[start['Lat']],
													lon=[start['Long']],
													mode='markers',
													marker=dict(size=15, color=color_scale[0]),
													text=['Start'],
							showlegend=True,
							hoverinfo='text')

		fig.add_scattermapbox(lat=[stop['Lat']],
													lon=[stop['Long']],
													mode='markers',
													marker=dict(size=15, color=color_scale[1]),
													text=['Stop'],
													hoverinfo='text')

		fig.add_scattermapbox(lat=df['Lat'],
													lon=df['Long'],
													mode='lines+markers',
													line=dict(width=2, color='blue'),
													marker=dict(size=6, color='blue'),
													text=['' for i in range(len(df))],
													hoverinfo='skip')

		if distressed_coordinate is not None:
				lat, long, _ = distressed_coordinate
				fig.add_scattermapbox(lat=[lat],
															lon=[long],
															mode='markers',
															marker=dict(size=15, color='green'),
															text=['Distressed'],
															hoverinfo='lon+lat+text')

		fig.update_layout(mapbox_style="open-street-map",
											mapbox_zoom=13,
											mapbox_center={"lat": df['Lat'].mean(),
																			"lon": df['Long'].mean()})
		fig.update_layout(margin={"r":0,"t":0,"l":0,"b":0})
		fig.update_layout(legend=dict(x=0, y=1, traceorder="normal", font=dict(family="sans-serif", size=12, color="black"), bgcolor="LightSteelBlue", bordercolor="Black", borderwidth=2))
		fig.show()


def closest_node(coord, coords):
	min_dist = float("inf")
	closest = None
	for c in coords:
		dist = ((c[0]-coord[0])**2 + (c[1]-coord[1])**2 + (c[2]-coord[2])**2)**0.5
		if dist < min_dist:
			closest = c
			min_dist = dist
	return closest


def traverse_closest(coords):
	path = [coords[0]]
	coords.pop(0)

	while coords:
		next_node = closest_node(path[-1], coords)
		path.append(next_node)
		coords.remove(next_node)

	return path


def traverse_coordinates(coords):
	return traverse_lines_lla(coords, 1e-5, 1e-5)


def spiral_explore(distances):
	n = len(distances)
	m = len(distances[0])
	visited = [[False for _ in range(m)] for __ in range(n)]
	start = (n//2, m//2)
	q = PriorityQueue()
	q.put((0, start))
	while not q.empty():
		curr_dist, curr = q.get()
		if visited[curr[0]][curr[1]]:
			continue
		visited[curr[0]][curr[1]] = True
		x, y = curr
		neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
		for neighbor in neighbors:
			if 0 <= neighbor[0] < n and 0 <= neighbor[1] < m:
				q.put((curr_dist + distances[neighbor[0]][neighbor[1]], neighbor))
	return visited


def haversine(lon1, lat1, lon2, lat2):
	R = 6371  # radius of the Earth in kilometers
	dLat = math.radians(lat2 - lat1)
	dLon = math.radians(lon2 - lon1)
	lat1 = math.radians(lat1)
	lat2 = math.radians(lat2)

	a = math.sin(dLat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dLon/2)**2
	c = 2*math.asin(math.sqrt(a))

	return R * c


def generate_distances(coords):
	n = len(coords)
	distances = [[0] * n for _ in range(n)]
	for i in range(n):
		for j in range(i, n):
			lat1, lon1, _ = coords[i]
			lat2, lon2, _ = coords[j]
			distances[i][j] = distances[j][i] = haversine(lon1, lat1, lon2, lat2)
	return distances


def traverse_lines_lla(gnss_points, lat_threshold, lng_threshold):
	"""
	Traverse the vertical lines in a zig-zag pattern by alternating the direction of traversal.
	Points are grouped by latitude and sorted by longitude.

	:param gnss_points: list of GNSS points as (latitude, longitude, altitude) tuples
	:param lat_threshold: maximum difference in latitude between points to be grouped together
	:param lng_threshold: maximum difference in longitude between points to be grouped together
	:return: list of GNSS points in zig-zag traversal order
	"""
	# Group points by latitude
	grouped_points = {}
	for lat, lng, alt in gnss_points:
		for group_lat, group_points in grouped_points.items():
			if abs(group_lat - lat) <= lat_threshold:
				group = group_points
				break
		else:
			grouped_points[lat] = []
			group = grouped_points[lat]
		group.append((lat, lng, alt))

	# Sort each group of points by longitude
	sorted_grouped_points = [(lat, sorted(group, key=lambda x: x[1])) for lat, group in sorted(grouped_points.items())]

	result = []
	direction = 1
	for lat, lat_points in sorted_grouped_points:
		if direction == -1:
			lat_points = lat_points[::-1]
		result.extend(lat_points)
		direction = -direction
	return result


def visualize_traversal_plt(
			traversal	: np.ndarray, 
			title			: str					= "",
			xlabel		: str					= "",
			ylabel		: str 				= ""
		) -> None:
	"""
	Plots the traversal trajectory, from the first to the last point 
	in the array  
	"""
	lats, lons, alts = zip(*traversal)
	plt.plot(lons, lats, '-o', markersize=3)
	start = traversal[0]
	end = traversal[-1]
	plt.scatter(start[1], start[0], marker='o', color='red', label='start')
	plt.scatter(end[1], end[0], marker='x', color='green', label='end')
	plt.legend()
	plt.title(title)
	plt.xlabel(xlabel)
	plt.ylabel(ylabel)
	plt.show()


def get_traversal_grid(altitude, camera_fov, overlap, area_size, lla_origin):
	waypoints = generate_waypoints_lla(altitude, camera_fov, overlap, area_size, lla_origin)

	return traverse_coordinates(waypoints)


def main():
	camera_fov = get_fov_from_hfov(1280, 720, 69)
	start_point = (63.451605, 10.431137, 10) #(63.447808, 10.407823, 25) # Lat, Lon, Alt
	area_size = (250, 250)
	overlap = 0.25
	altitude = 10

	# waypoints = generate_waypoints_lla(altitude, camera_fov, overlap, area_size, start_point, False)
	# ned_waypoints = generate_waypoints_ned(altitude, camera_fov, overlap, area_size)
	# print(ned_waypoints.shape)
	# ned_waypoints = np.vstack([ned_waypoints, [[0, 0, -altitude]]]) # Also search the search position

	# print("Num waypoints: ", len(ned_waypoints))
	# print(ned_waypoints)
	# traversed = traverse_coordinates(waypoints)

	# visualize_traversal_plt(traversed)

	# ned_points = expanding_square_search_ned(altitude, camera_fov, overlap, area_size)
	# east_north_points = ned_points[:,[1,0]]
	# # print(east_north_points)
	# lla_points = east_north_to_lat_lon_alt(east_north_points, altitude, start_point[0], start_point[1])
	# print(lla_points)
	# traversed = traverse_coordinates(lla_points)

	# visualize_traversal_plt(ned_points, "Expanding square search", "East", "North")

	lla_points = line_search_lla(altitude, camera_fov, overlap, area_size, start_point)
	visualize_traversal_plt(lla_points, "Line search", "Latitude", "Longitude")

	# plot_traversal_on_map(lla_points, start_point)

	return

if __name__ == '__main__':
	main()