from Graph import *
import time

if __name__ == "__main__":
    graph = Graph()
    t1 = time.time()
    graph.write_map_geojson()
    t2 = time.time()
    print(f"Execution time: {t2 - t1}")