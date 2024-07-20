import networkx as nx
from pyproj import Transformer
import math
import heapq
import json
import sys
import collections
import time

from RouteVar import *
from Stop import *
from Path import *

class Graph:
    INF = 9999999999999999999
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:3405")
    def __init__(self):
        self.G = nx.MultiDiGraph()
        self.times_all = {}
        self.shortest_path_time = {}
        self.shortest_adj = {}
        self.shortest_cnt = {}
        self.stop_info = {}
        self.node_order = {}
        self.order_of = {}

    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))

    def make_graph(self):
        cnt = 0
        route_var_objects = RouteVarQuery()
        route_var_objects.readRouteVar()
        stop_objects = StopQuery()
        stop_objects.read_stop()
        path_objects = PathQuery()
        path_objects.read_path()
        for route_var_object in route_var_objects.route_vars:
            speed = route_var_object.get_distance() / (route_var_object.get_running_time() * 60)
            route_id = route_var_object.get_route_id()
            route_var_id = route_var_object.get_route_var_id()

            stop_object = stop_objects.searchByABC(RouteId=str(route_id), RouteVarId=str(route_var_id))
            path_object = path_objects.searchByABC(RouteId=str(route_id), RouteVarId=str(route_var_id))

            stops = [[stop.StopId, stop.Lng, stop.Lat] for stop in stop_object]
            coordinates = [[point.lng, point.lat] for point in path_object]

            start_stop_id = stops[0][0]
            stops = stops[1:]
            for stop in stops:
                cnt += 1
                if cnt % 1000 == 0:
                    print('..........Loading ' + str(cnt) + '/9946 edges..........')
                end_stop_id = stop[0]

                x = stop[1]
                y = stop[2]
                
                min_dist = self.INF
                for i in range(len(coordinates)):
                    cur_dist = self.get_distance(x, y, coordinates[i][0], coordinates[i][1])
                    if cur_dist < min_dist:
                        min_dist = cur_dist
                        closest_index = i

                distance = 0
                path = coordinates[0 : closest_index + 1]
                pathXY = [[*self.transformer.transform(point[1], point[0])] for point in path]
                coordinates = coordinates[closest_index:]
                for p1, p2 in zip(pathXY, pathXY[1:]):
                    distance += self.get_distance(p1[0], p1[1], p2[0], p2[1])
                time = distance / speed

                if self.G.has_edge(start_stop_id, end_stop_id):
                    if time < self.G.get_edge_data(start_stop_id, end_stop_id)[0]['time']:
                        self.G.get_edge_data(start_stop_id, end_stop_id)[0]['distance'] = distance
                        self.G.get_edge_data(start_stop_id, end_stop_id)[0]['time'] = time
                        self.G.get_edge_data(start_stop_id, end_stop_id)[0]['route_id'] = route_id
                        self.G.get_edge_data(start_stop_id, end_stop_id)[0]['route_var_id'] = route_var_id
                        self.G.get_edge_data(start_stop_id, end_stop_id)[0]['coordinates'] = coordinates
                else:
                    self.G.add_edge(start_stop_id, end_stop_id, distance=distance, time=time, 
                                    route_id=route_id, route_var_id=route_var_id, coordinates=path, shortcut_node = 0)
                if start_stop_id not in self.times_all:
                    self.times_all[start_stop_id] = {}
                self.times_all[start_stop_id][end_stop_id] = time
                start_stop_id = end_stop_id
        print('SUCCESSFULLY CREATED GRAPH')
    
    def make_graph_demo(self):
        nodes = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K']
        self.G.add_nodes_from(nodes)

        # Define edges with weights
        edges = [
            ('A', 'C', 5), ('A', 'B', 3),
            ('B', 'C', 3), ('B', 'D', 5),
            ('C', 'D', 2), ('C', 'J', 2),
            ('D', 'E', 7), ('D', 'J', 4), 
            ('E', 'F', 6), ('E', 'J', 3),
            ('F', 'H', 2), 
            ('G', 'H', 3), ('G', 'F', 4),
            ('H', 'I', 3), ('H', 'J', 2), 
            ('I', 'J', 4), ('I', 'G', 5), 
            ('J', 'K', 3),
            ('K', 'A', 3), ('K', 'I', 6)
        ]

        # Add edges with weights and shortcut_node attribute
        for u, v, w in edges:
            self.G.add_edge(u, v, weight=w, shortcut_node=0)
            self.G.add_edge(v, u, weight=w, shortcut_node=0)
            if u not in self.times_all:
                self.times_all[u] = {}
            if v not in self.times_all:
                self.times_all[v] = {}
            self.times_all[u][v] = w
            self.times_all[v][u] = w

    def dijkstra(self, start_vertex):
        vertices = list(self.G.nodes())

        visited = set()
        parents = {}
        adj = {v: [] for v in vertices}
        exist = {v: 0 for v in vertices}
        D = {v: float('inf') for v in vertices}

        parents[start_vertex] = start_vertex
        D[start_vertex] = 0
        exist[start_vertex] = 1
        pq = [(0, start_vertex)]
        while pq:
            current_cost, current_vertex = heapq.heappop(pq)
            if current_vertex in visited:
                continue
            visited.add(current_vertex)

            current_neighbors = list(self.G.neighbors(current_vertex))
            for neighbor in current_neighbors:
                old_cost = D[neighbor]
                new_cost = D[current_vertex] + self.times_all[current_vertex][neighbor]
                if new_cost < old_cost:
                    parents[neighbor] = current_vertex
                    exist[neighbor] = exist[current_vertex]
                    if neighbor not in adj[current_vertex]:
                        adj[current_vertex].append(neighbor)
                    D[neighbor] = new_cost
                    heapq.heappush(pq, (new_cost, neighbor))

        self.shortest_adj[start_vertex] = adj
        self.shortest_cnt[start_vertex] = exist
        return D, parents
    
    def dijkstra_all_pairs(self):
        vertices = list(self.G.nodes())
        cnt =  1
        t1 = time.time()
        for u in vertices:
            if cnt == 16:
                break
            print(cnt)
            cnt += 1
            for v in vertices:
                dist, parents = self.dijkstra(u)
                p = self.get_path(v, parents)
        t2 = time.time()
        print(t2 - t1, (t2 - t1) / (15*4397))

    def get_shortest_path_dijkstra(self, start_stop, end_stop):
        shortest_time, parents = self.dijkstra(start_stop)
        path = self.get_path(end_stop, parents)
        if not path:
            print(f'No path found from {start_stop} to {end_stop}!')
            return 0, path
        # print(shortest_time[end_stop], path) 
        return shortest_time[end_stop], path  

    def get_path(self, current_vertex, parents):
        path = []
        if parents.get(current_vertex, 0) == 0:
            return path
        while parents[current_vertex] != current_vertex:
            path.append(current_vertex)
            current_vertex = parents[current_vertex]
        path.append(current_vertex)
        return list(reversed(path))

    def get_shortest_path_dijkstra_pathcaching(self, source_node, target_node):
        self.load_stop_info_json()
        with open("input_data/path_caching_demo.json", "r", encoding='utf-8') as f:
            fixed_point = json.load(f)

        t1 = time.time()
        for i in range (0, 100):
            vertices = list(self.G.nodes)
            source_zone = self.stop_info[str(source_node)][2]
            target_zone = self.stop_info[str(target_node)][2]
            if fixed_point[source_zone][target_zone]['have_fixed_point'] == False:
                cur_time, cur_path = self.get_shortest_path_dijkstra(source_node, target_node)
            else:
                tmp_time = fixed_point[source_zone][target_zone]['time']
                tmp_path = fixed_point[source_zone][target_zone]['path']
                time1, path1 = self.get_shortest_path_dijkstra(source_node, tmp_path[0])
                time2, path2 = self.get_shortest_path_dijkstra(tmp_path[-1], target_node)
                cur_time = time1 + tmp_time + time2
                cur_path = path1 + tmp_path + path2
        t2 = time.time()
        print(f'Query time: {(t2-t1)/100}')
        return cur_time, cur_path

    def get_stop_info(self):
        stop_query = StopQuery()
        stop_query.read_stop()
        vertices = list(self.G.nodes())
        coordinate = {}
        cnt = 1
        for u in vertices:
            print(f"..........Loading {cnt}/4397 stop coordinates..........")
            cnt+=1
            nodes = stop_query.searchByABC(StopId = u)
            coordinate[u] = [nodes[0].get_lng(), nodes[0].get_lat(), nodes[0].get_zone()]
        return coordinate
    
    def write_stop_info_json(self):
        stop_info = self.read_stop_info()
        with open("input_data/stop_coordinate.json", "w", encoding='utf-8') as f:
            json_object = json.dumps(stop_info, indent=4, ensure_ascii=False)
            f.write(json_object)
        print("ALL STOP COORDINATES HAVE BEEN WRITTEN INTO FILE!")

    def load_stop_info_json(self):
        with open("input_data/stop_coordinate.json", "r", encoding='utf-8') as f:
            self.stop_info = json.load(f)
  
    def a_star(self, source_node, target_node): # Load stop info before use
        close_list = set()
        open_list = []
        g = {}
        f = {}
        h = {}
        parents = {}

        def heuristic(cur_id, target_id):
            h[cur_id] = self.get_distance(self.stop_info[str(cur_id)][0], self.stop_info[str(cur_id)][1], 
                                    self.stop_info[str(target_id)][0], self.stop_info[str(target_id)][1])

        g[source_node] = 0
        heuristic(source_node, target_node)
        f[source_node] = g[source_node] + h[source_node]
        parents[source_node] = source_node
        heapq.heappush(open_list, (f[source_node], source_node))
        while open_list:
            _, n = heapq.heappop(open_list)
            if n in close_list:
                continue
            if n == target_node:
                path = self.get_path(n, parents)
                return f[target_node], path
            
            close_list.add(n)
            
            for m in self.G.neighbors(n):
                g_new = g[n] + self.times_all[n][m]
                if m not in h:
                    heuristic(m, target_node)
                f_new = g_new + h[m]

                if m not in f or f_new < f[m]:
                    g[m] = g_new
                    f[m] = f_new
                    parents[m] = n
                    heapq.heappush(open_list, (f_new, m))
        
        return 0, []
    
    def get_shortest_path_astar(self, source_node, target_node):
        self.load_stop_info_json()
        t, p = self.a_star(source_node, target_node)
        if not p:
            print(f'No p found from {source_node} to {target_node}!')
            return 0, p
        print(t, p)
        return t, p

    def update_fixed_point(self, path1, path2, time1, time2):
        new_path = []
        cnt = {v: 1 for v in path1}

        for u in path2:
            if cnt.get(u, 0) == 1:
                i = path1.index(u)
                j = path2.index(u)

                while i < len(path1) and j < len(path2) and path1[i] == path2[j]:
                    new_path.append(path1[i])
                    i += 1
                    j += 1
                break
        
        if not new_path:
            if time2 < time1:
                return time2, path2
            else:
                return time1, path1

        new_time = 0
        for u, v in zip(new_path, new_path[1:]):
            new_time += self.times_all[u][v]
        return new_time, new_path
    
    def update_intersection(self, path1, path2, time1, time2):
        result = collections.Counter(path1) & collections.Counter(path2)
        new_path = list(result.elements())

        new_time = 0
        for u, v in zip(new_path, new_path[1:]):
            new_time += self.times_all[u][v]

        return new_time, new_path

    def get_shortest_path_astar_pathcaching_updating(self, source_node, target_node):
        self.load_stop_info_json()
        with open("input_data/path_caching.json", "r", encoding='utf-8') as f:
            fixed_point = json.load(f)
        time = 0
        path = []
        source_zone = self.stop_info[str(source_node)][2]
        target_zone = self.stop_info[str(target_node)][2]
        if fixed_point[source_zone][target_zone]['have_fixed_point'] == False:
            self.a_star(source_node, target_node)
        elif fixed_point[source_zone][target_zone]['have_fixed_point'] == True:
            new_time, new_path = self.a_star(source_node, target_node)
            if new_time is not None:
                tmp_time = fixed_point[source_zone][target_zone]['time']
                tmp_path = fixed_point[source_zone][target_zone]['path']
                time1, path1 = self.a_star(source_node, tmp_path[0])
                time2, path2 = self.a_star(tmp_path[-1], target_node)
                cur_time = time1 + tmp_time + time2
                cur_path = path1 + tmp_path + path2
                
                if new_time >= cur_time:
                    time = cur_time
                    path = cur_path
                else:
                    fixed_point[source_zone][target_zone]['time'], fixed_point[source_zone][target_zone]['path'] = self.update_intersection(tmp_path, new_path, tmp_time, new_time)
                    if fixed_point[source_zone][target_zone]['time'] == 0:
                        fixed_point[source_zone][target_zone]['have_fixed_point'] = False
                    time = new_time
                    path = new_path
        else:
            new_time, new_path = self.a_star(source_node, target_node)
            fixed_point[source_zone][target_zone]['have_fixed_point'] = False
            for v in new_path:
                if self.stop_info[str(v)] != source_zone and self.stop_info[str(v)] != target_zone:
                    fixed_point[source_zone][target_zone]['have_fixed_point'] = True
                    fixed_point[source_zone][target_zone]['time'] = new_time
                    fixed_point[source_zone][target_zone]['path'] = new_path
                    break
            time = new_time
            path = new_path
        
        print(time, path)
        with open("input_data/path_caching.json", "w", encoding='utf-8') as f:
            json_object = json.dumps(fixed_point, indent=4, ensure_ascii=False)
            f.write(json_object)
        if not path:
            print(f'No p found from {source_node} to {target_node}!')
            return 0, path
        return time, path
    
    def get_shortest_path_astar_pathcaching(self, source_node, target_node):
        self.load_stop_info_json()
        with open("input_data/path_caching_demo.json", "r", encoding='utf-8') as f:
            fixed_point = json.load(f)

        t1 = time.time()
        vertices = list(self.G.nodes)
        source_zone = self.stop_info[str(source_node)][2]
        target_zone = self.stop_info[str(target_node)][2]
        if fixed_point[source_zone][target_zone]['have_fixed_point'] == False:
            cur_time, cur_path = self.a_star(source_node, target_node)
        else:
            tmp_time = fixed_point[source_zone][target_zone]['time']
            tmp_path = fixed_point[source_zone][target_zone]['path']
            time1, path1 = self.a_star(source_node, tmp_path[0])
            time2, path2 = self.a_star(tmp_path[-1], target_node)
            cur_time = time1 + tmp_time + time2
            cur_path = path1 + tmp_path + path2
        t2 = time.time()
        print(f'Query time: {t2-t1}')
        return cur_time, cur_path

    def a_star_all_pairs(self):
        self.load_stop_info_json()
        vertices = list(self.G.nodes)
        cnt = 1
        for u in vertices:
            if cnt == 16: 
                break
            print('...........Loading ' + str(cnt) + '/4397 nodes..........')
            cnt += 1
            for v in vertices:
                self.a_star(u, v)

    def a_star_all_pairs_caching(self):
        with open("input_data/stop_coordinate.json", "r", encoding='utf-8') as f:
            stop_zone = json.load(f)
        with open("input_data/path_caching_demo.json", "r", encoding='utf-8') as f:
            fixed_point = json.load(f)
        
        vertices = list(self.G.nodes)
        cnt = 1
        for u in vertices:
            if cnt == 16: 
                break
            print('...........Loading ' + str(cnt) + '/4397 nodes..........')
            cnt += 1
            cnt1 = 1
            u_zone = stop_zone[str(u)][2]
            for v in vertices:
                if cnt1 % 500 == 0:
                    print(cnt1)
                cnt1 += 1
                v_zone = stop_zone[str(v)][2]
                if fixed_point[u_zone][v_zone]['have_fixed_point'] == False:
                    self.a_star(u, v)
                else:
                    tmp_time = fixed_point[u_zone][v_zone]['time']
                    tmp_path = fixed_point[u_zone][v_zone]['path']
                    time1, path1 = self.a_star(u, tmp_path[0])
                    time2, path2 = self.a_star(tmp_path[-1], v)
                    cur_time = time1 + tmp_time + time2
                    cur_path = path1 + tmp_path + path2

    def a_star_all_pairs_caching_updating(self):
        vertices = list(self.G.nodes)
        cnt = 1
        for u in vertices:
            if cnt == 16: 
                break
            print('...........Loading ' + str(cnt) + '/4397 nodes..........')
            cnt += 1
            cnt1 = 1
            for v in vertices:
                if cnt1 % 500 == 0:
                    print(cnt1)
                cnt1 += 1
                self.get_shortest_path_pathcaching(u, v)
                

    def generate_caching_json(self):
        with open("input_data/stop_coordinate.json", "r", encoding='utf-8') as f:
            stop_zone = json.load(f)
        with open("input_data/path_caching_demo.json", "r", encoding='utf-8') as f:
            fixed_point = json.load(f)

        vertices = list(self.G.nodes)
        cnt = 1
        for source_node in vertices:
            if cnt % 100 == 0 or cnt == 4397:
                print('...........Loading ' + str(cnt) + '/4397 nodes..........')
            cnt+=1
            shortest_time, parents =  self.dijkstra(source_node)
            source_zone = stop_zone[str(source_node)][2]

            for target_node in vertices:
                path = self.get_path(target_node, parents)
                target_zone = stop_zone[str(target_node)][2]

                if fixed_point[source_zone][target_zone]['have_fixed_point'] == False:
                    continue
                elif fixed_point[source_zone][target_zone]['have_fixed_point'] == True: 
                    fixed_point[source_zone][target_zone]['time'], fixed_point[source_zone][target_zone]['path'] = self.update_intersection(fixed_point[source_zone][target_zone]['path'], path, fixed_point[source_zone][target_zone]['time'], shortest_time[target_node])

                    if fixed_point[source_zone][target_zone]['time'] == 0:
                        fixed_point[source_zone][target_zone]['have_fixed_point'] = False
                else:
                    fixed_point[source_zone][target_zone]['have_fixed_point'] = False
                    for v in path:
                        if stop_zone[str(v)][2] != source_zone and stop_zone[str(v)][2] != target_zone:
                            fixed_point[source_zone][target_zone]['have_fixed_point'] = True
                            fixed_point[source_zone][target_zone]['time'] = shortest_time[target_node]
                            fixed_point[source_zone][target_zone]['path'] = path
                            break
        
        with open("input_data/path_caching_demo.json", "w", encoding='utf-8') as f:
            json_object = json.dumps(fixed_point, indent=4, ensure_ascii=False)
            f.write(json_object)

    def init_caching_json(self):
        with open("input_data/stop_coordinate.json", "r", encoding='utf-8') as f:
            stop_zone = json.load(f)
        vertices = list(self.G.nodes)
        fixed_point = {}

        cnt = 1
        for u in vertices:
            print('..............Loading ' + str(cnt) + '/4397 nodes............')
            cnt += 1
            for v in vertices:
                u_zone = stop_zone[str(u)][2]
                v_zone = stop_zone[str(v)][2]
                if u_zone not in fixed_point:
                    fixed_point[u_zone] = {}
                if v_zone not in fixed_point[u_zone]:
                    fixed_point[u_zone][v_zone] = {}
                fixed_point[u_zone][v_zone]['have_fixed_point'] = ''
                fixed_point[u_zone][v_zone]['time'] = 0
                fixed_point[u_zone][v_zone]['path'] = []

        with open("input_data/path_caching.json", "w", encoding='utf-8') as f:
            json_object = json.dumps(fixed_point, indent=4, ensure_ascii=False)
            f.write(json_object)

    def output_shortest_path_json(self, paths):

        # geojson file to view the path on the map
        stop_query = StopQuery()
        stop_query.read_stop()
        geojson_data = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "LineString",
                        "coordinates": []
                    },
                } 
            ]
        }
        
        for (a, b) in zip(paths, paths[1:]): 
            coordinates = self.G.get_edge_data(a, b)[0]['coordinates']
            for coordinate in coordinates:
                geojson_data['features'][0]['geometry']['coordinates'].append(coordinate)

        stop_objects = [stop_query.searchByABC(StopId=stopid) for stopid in paths]
        for stop_coordinate, stop_id in zip(stop_objects, paths):
            feature = {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [stop_coordinate[0].Lng, stop_coordinate[0].Lat]
                },
                "properties": {
                    "StopId": stop_id,
                }
            }
            geojson_data['features'].append(feature)

        json_object = json.dumps(geojson_data, indent = 4)
        with open("shortest_path_coordinates.geojson", "w") as outfile:
            outfile.write(json_object)
        
        # json file for all the edges gone through and start, end stop of each
        stop_query = StopQuery()
        stop_query.read_stop()
        edges_data = []
        first_edge = False
        for (a, b) in zip(paths, paths[1:]): 
            if first_edge is False:
                edge = self.G.get_edge_data(a, b)[0]
                first_edge = True
                prev_route_id = edge['route_id']
                prev_route_var_id = edge['route_var_id']
            else:
                edges = self.G.get_edge_data(a, b)
                find = False
                for key, item in edges.items():
                    if item['route_id'] == prev_route_id and item['route_var_id'] == prev_route_var_id:
                        edge = item
                        find = True
                        break
                if find is False:
                    edge = edges[0]
                    prev_route_id = edge['route_id']
                    prev_route_var_id = edge['route_var_id']
            start_stop = stop_query.searchByABC(StopId=a, RouteId=str(edge['route_id']), RouteVarId=str(edge['route_var_id'])) 
            end_stop = stop_query.searchByABC(StopId=b, RouteId=str(edge['route_id']), RouteVarId=str(edge['route_var_id'])) 
            edge_data = {
                'From': a,
                'To': b,
                'RouteId': edge['route_id'],
                'RouteVarId': edge['route_var_id'],
                'Distance': edge['distance'],
                'Time': edge['time'],
                'StartStop': start_stop[0].get_stop(),
                'EndStop': end_stop[0].get_stop()
            }
            edges_data.append(edge_data)
        with open("shortest_path_stops.json", "w", newline='', encoding='utf-8') as f:
            for edge in edges_data:
                json_object = json.dumps(edge, ensure_ascii=False)
                f.write(json_object + '\n')       

    # Contraction Hieracrhies
    def get_node_order(self):
        node_pq = []
        for v in list(self.G.nodes()):
            val = len(list(self.G.neighbors(v))) + len(list(self.G.predecessors(v)))
            heapq.heappush(node_pq, (val, v))
        i = 1
        while node_pq:
            _, v = heapq.heappop(node_pq)
            self.node_order[i] = v
            self.order_of[v] = i
            i += 1

    def local_dijkstra_without_v(self, u, v, P_max):
        vertices = list(self.G.nodes)
        visited = set()
        pq = [(0, u)]
        D = {v: float('inf') for v in vertices}
        visited.add(v)
        D[u] = 0
        while pq:
            cost, n = heapq.heappop(pq)
            if n in visited:
                continue
            if cost > P_max:
                break
            visited.add(n)

            for neighbor in list(self.G.neighbors(n)):
                if neighbor in self.order_of:
                    continue
                old_cost = D[neighbor]
                new_cost = D[n] + self.times_all[n][neighbor]
                if new_cost < old_cost:
                    D[neighbor] = new_cost
                    heapq.heappush(pq, (new_cost, neighbor))
        return D

    def edge_difference(self, v):
        dif = - len(list(self.G.neighbors(v))) - len(list(self.G.predecessors(v)))
        for u in list(self.G.predecessors(v)):
            if u in self.order_of:
                continue
            P = {}
            for w in list(self.G.neighbors(v)):
                if w in self.order_of:
                    continue
                P[w] = self.times_all[u][v] + self.times_all[v][w]
            if not P:
                continue
            P_max = max(P.values())

            D = self.local_dijkstra_without_v(u, v, P_max)
            
            for w in list(self.G.neighbors(v)):
                if w in self.order_of:
                    continue
                if D[w] > P[w]:
                    dif += 1
        return dif
    
    def get_node_order_edge_difference(self):
        node_pq = []
        for v in list(self.G.nodes()):
            dif = self.edge_difference(v)
            heapq.heappush(node_pq, (dif, v))
        return node_pq
    
    def preprocess(self):
        node_pq = self.get_node_order_edge_difference()
        order = 0
        while node_pq:
            # Calculate edge difference again to update the pq and get the next node
            _, v = heapq.heappop(node_pq)

            new_dif = self.edge_difference(v)
            if node_pq and new_dif > node_pq[0][0]:
                heapq.heappush(node_pq, (new_dif, v))
                continue

            order += 1
            if order % 500 == 0:
                print(f"..........Contracting {order}/4397 nodes..........")
            self.order_of[v] = order
            self.node_order[order] = v

            for u in list(self.G.predecessors(v)):
                if u in self.order_of:
                    continue
                P = {}
                for w in list(self.G.neighbors(v)):
                    if w in self.order_of:
                        continue
                    P[w] = self.times_all[u][v] + self.times_all[v][w]
                if not P:
                    continue
                P_max = max(P.values())

                D = self.local_dijkstra_without_v(u, v, P_max)

                for w in list(self.G.neighbors(v)):
                    if w in self.order_of:
                        continue
                    
                    if D[w] > P[w]:
                        if self.G.has_edge(u, w):
                            self.G.get_edge_data(u, w)[0]['shortcut_node'] = v
                        else:
                            self.G.add_edge(u, w, shortcut_node=v)
                        self.times_all[u][w] = P[w]
        print('Preprocess Done!')

    def bidirectional_dijkstra(self, source_node, target_node):
        vertices = list(self.G.nodes())
        visited_start = set()
        visited_end = set()
        parents1 = {}
        parents2 = {}
        dist1 = {v: float('inf') for v in vertices}
        dist2 = {v: float('inf') for v in vertices}

        parents1[source_node] = source_node
        parents2[target_node] = target_node
        dist1[source_node] = 0
        dist2[target_node] = 0
        pq_start = [(0, source_node)]
        pq_end = [(0, target_node)]
        while pq_start or pq_end:
            if pq_start:
                _, current_vertex = heapq.heappop(pq_start)
                if current_vertex in visited_start:
                    continue
                visited_start.add(current_vertex)

                for neighbor in self.G.neighbors(current_vertex):
                    if self.order_of[neighbor] <= self.order_of[current_vertex]:
                        continue

                    new_cost = dist1[current_vertex] + self.times_all[current_vertex][neighbor]
                    if new_cost < dist1[neighbor]:
                        parents1[neighbor] = current_vertex
                        dist1[neighbor] = new_cost
                        heapq.heappush(pq_start, (new_cost, neighbor))
            if pq_end:
                _, current_vertex = heapq.heappop(pq_end)
                if current_vertex in visited_end:
                    continue
                visited_end.add(current_vertex)

                for neighbor in self.G.predecessors(current_vertex):
                    if self.order_of[neighbor] <= self.order_of[current_vertex]:
                        continue

                    new_cost = dist2[current_vertex] + self.times_all[neighbor][current_vertex]
                    if new_cost < dist2[neighbor]:
                        parents2[neighbor] = current_vertex
                        dist2[neighbor] = new_cost
                        heapq.heappush(pq_end, (new_cost, neighbor))

        L = [v for v in self.G.nodes if dist1[v] != float('inf') and dist2[v] != float('inf')]
        if not L:
            return 0, []

        shortest_time = math.inf
        common_node = 0
        for v in L:
            if shortest_time > dist1[v] + dist2[v]:
                shortest_time = dist1[v] + dist2[v]
                common_node = v

        def generate_shortcut(start_node, end_node):
            shortcut_node = self.G.get_edge_data(start_node, end_node)[0]['shortcut_node']
            if shortcut_node != 0:
                return generate_shortcut(start_node, shortcut_node) + [shortcut_node] + generate_shortcut(shortcut_node, end_node)
            else:
                return []

        shortest_path = []
        path1 = []
        cur_node = common_node

        while parents1[cur_node] != cur_node:
            tmp_node = parents1[cur_node]
            path = []
            if self.G.get_edge_data(tmp_node, cur_node)[0]['shortcut_node'] != 0:
                path = generate_shortcut(tmp_node, cur_node)
            path1 = path + path1
            path1 = [tmp_node] + path1
            cur_node = tmp_node

        cur_node = common_node
        path2 = []
        while parents2[cur_node] != cur_node:
            path2.append(cur_node)
            tmp_node = parents2[cur_node]
            path = []
            if self.G.get_edge_data(cur_node, tmp_node)[0]['shortcut_node'] != 0:
                path = generate_shortcut(cur_node, tmp_node)
            path2 += path
            cur_node = tmp_node
        path2.append(cur_node)
        
        shortest_path = path1 + path2
        return shortest_time, shortest_path

    def get_shortest_path_CH(self, source_node, target_node):
        t1 = time.time()
        self.preprocess()
        t2 = time.time()
        print("Preprocessing time:", t2 - t1)

        t, p = self.bidirectional_dijkstra(source_node, target_node)
        t3 = time.time()
        print('Query time: ', t3 - t2)
        if not p:
            print(f'No path found from {source_node} to {target_node}!')
            return 0, p
        print(t, p)
        return t, p
    
    def get_shortest_path_CH_pathcaching(self, source_node, target_node):
        t1 = time.time()
        self.preprocess()
        t2 = time.time()
        print("Preprocessing time:", t2 - t1)
        with open("input_data/path_caching_demo.json", "r", encoding='utf-8') as f:
            fixed_point = json.load(f)
        vertices = list(self.G.nodes)
        source_zone = self.stop_info[str(source_node)][2]
        target_zone = self.stop_info[str(target_node)][2]
        if fixed_point[source_zone][target_zone]['have_fixed_point'] == False:
            cur_time, cur_path = self.bidirectional_dijkstra(source_node, target_node)
        else:
            tmp_time = fixed_point[source_zone][target_zone]['time']
            tmp_path = fixed_point[source_zone][target_zone]['path']
            time1, path1 = self.bidirectional_dijkstra(source_node, tmp_path[0])
            time2, path2 = self.bidirectional_dijkstra(tmp_path[-1], target_node)
            cur_time = time1 + tmp_time + time2
            cur_path = path1 + tmp_path + path2
        t3 = time.time()
        print('Query time: ', (t3 - t2)/100)
        print(cur_time, cur_path)
        return cur_time, cur_path

    def CH_all_pairs(self):
        t1 = time.time()
        self.preprocess()
        t2 = time.time()
        print('Preprocess time: ', t2 - t1)
        vertices = list(self.G.nodes)
        cnt = 1
        for u in vertices:
            if cnt == 16:
                break
            print(cnt)
            cnt += 1
            for v in vertices:
                t, p = self.bidirectional_dijkstra(u, v)
        t3 =  time.time()
        print(t3 - t2, (t3 - t2) / (4397*15))

    def CH_all_pairs_caching(self):
        with open("input_data/stop_coordinate.json", "r", encoding='utf-8') as f:
            stop_zone = json.load(f)
        with open("input_data/path_caching_demo.json", "r", encoding='utf-8') as f:
            fixed_point = json.load(f)
        t1 = time.time()
        self.preprocess()
        t2 = time.time()
        vertices = list(self.G.nodes)
        cnt = 1
        for u in vertices:
            if cnt == 101: 
                break
            print('...........Loading ' + str(cnt) + '/4397 nodes..........')
            cnt += 1
            u_zone = stop_zone[str(u)][2]
            for v in vertices:
                v_zone = stop_zone[str(v)][2]
                if fixed_point[u_zone][v_zone]['have_fixed_point'] == False:
                    t, p = self.bidirectional_dijkstra(u, v)
                else:
                    tmp_time = fixed_point[u_zone][v_zone]['time']
                    tmp_path = fixed_point[u_zone][v_zone]['path']
                    t1, p1 = self.bidirectional_dijkstra(u, tmp_path[0])
                    t2, p2 = self.bidirectional_dijkstra(tmp_path[-1], v)
                    cur_time = t1 + tmp_time + t2
                    cur_path = p1 + tmp_path + p2
        t3 =  time.time()
        print(t3 - t2, (t3 - t2) / (4397*15))