from shapely.geometry import LineString
from sklearn.neighbors import KDTree
import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx
import numpy.linalg as LA
import numpy as np

def can_connect(n1, n2, polygons):
    l = LineString([n1, n2])
    for p in polygons:
        if p.crosses(l) and p.height >= min(n1[2], n2[2]):
            return False
    return True


def create_graph(polygons, nodes, k):
    g = nx.Graph()
    tree = KDTree(nodes)
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]
        
        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue
                
            if can_connect(n1, n2, polygons):
                g.add_edge(n1, n2, weight=LA.norm(np.array(n2) - np.array(n1)))
    return g
