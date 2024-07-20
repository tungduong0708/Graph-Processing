from Path import *
from Stop import *
from RouteVar import *
from Graph import *

import time
import openai
import json
import networkx as nx
from pyproj import Transformer
from rtree import *

if __name__ == '__main__':
    graph = Graph()
    time1 = time.time()
    graph.make_graph()
    time2 = time.time()
    graph.get_shortest_path_dijkstra(32, 3874)
    time3 = time.time()
    graph.get_shortest_path_astar(32, 3874)
    time4 = time.time()
    graph.get_shortest_path_CH(32, 3874)
    time5 = time.time()
    print("Time making graph: ", time2 - time1)
    print('Dijkstra time: ', time3 - time2)
    print('A* time: ', time4 - time3)
    print('CH time (preprocess + query): ', time5 - time4)
