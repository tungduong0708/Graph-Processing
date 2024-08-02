import osmium
import json

class OSMElementHandler(osmium.SimpleHandler):
    def __init__(self):
        super(OSMElementHandler, self).__init__()
        self.nodes = {}
        self.ways = {}
        self.relations = {}

    def node(self, n):
        self.nodes[n.id] = {
            'id': n.id,
            'lat': n.location.lat,
            'lon': n.location.lon,
            'tags': {tag.k: tag.v for tag in n.tags}
        }

    def way(self, w):
        self.ways[w.id] = {
            'id': w.id,
            'nodes': [n.ref for n in w.nodes],
            'tags': {tag.k: tag.v for tag in w.tags}
        }

    def relation(self, r):
        self.relations[r.id] = {
            'id': r.id,
            'members': [{'type': m.type, 'ref': m.ref, 'role': m.role} for m in r.members],
            'tags': {tag.k: tag.v for tag in r.tags}
        }

class Graph():
    def write_map_geojson(self):
        handler = OSMElementHandler()
        handler.apply_file("HoChiMinh.osm")

        geojson_data = {
            "type": "FeatureCollection",
            "features": []
        }
        cnt = 0
        for node in handler.nodes:
            cnt += 1
            if cnt % 1000 == 0:
                print(cnt)
            feature = {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [node['lng'], node['lat']]
                },
                "properties": {
                    "id": node['id']
                }
            }
            geojson_data['features'].append(feature)

        json_object = json.dumps(geojson_data, indent = 4)
        with open("shortest_path_coordinates.geojson", "w") as outfile:
            outfile.write(json_object)
        
        print(f"Write stop coordinates to file successfullly!")
        print(f"There are {cnt} nodes.")