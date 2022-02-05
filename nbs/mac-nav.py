import heapq
from math import inf

Map_Graph = {
    1: [(3, 1, 's')],
    2: [],
    3: [(11, 1, 'l'), (18, 1, 's')],
    4: [(2, 1, 'r'), (18, 1, 'l')],
    5: [(11, 1, 'r'), (2, 1, 's')],
    6: [(5, 1, 'r'), (8, 1, 'l')],
    7: [(9, 1, 'r'), (5, 1, 's')],
    8: [],
    9: [(13, 1, 'l'), (15, 1, 'r')],
    10: [(6, 1, 'r'), (15, 1, 's')],
    11: [(10, 1, 'r'), (17, 1, 'l')],
    12: [(4, 1, 'r'), (10, 1, 's')],
    13: [(17, 1, 's'), (4, 1, 'l')],
    14: [(6, 1, 'l'), (13, 1, 's')],
    15: [],
    16: [(12, 1, 's')],
    17: [],
    18: [(9, 1, 'l'), (8, 1, 's')]

}


################################################################################
# all heading angle changes measure keep north as 0 and towards right is +
# 2
# . .
# .  .
# .   .
# 0____1   0->2 is 0 degree change 0->1 is +90 degree change in heading angle
################################################################################


# change heading when turns
# update this with care
# heading_angle=0
# # +90 -90 +180 +360 etc

# start_node=0
# end_node=0

found_path = []

cur_node = 0
indexnode = 0

# visted_node=[]


def dijkstra(adj, start, target):
    d = {start: 0}
    parent = {start: None}
    pq = [(0, start)]
    visited = set()
    while pq:
        du, u = heapq.heappop(pq)
        if u in visited:
            continue
        if u == target:
            break
        visited.add(u)
        for v, weight, _ in adj[u]:
            if v not in d or d[v] > du + weight:
                d[v] = du + weight
                parent[v] = u
                heapq.heappush(pq, (d[v], v))

    fp = [target]
    tg = target

    while tg != start:
        fp.insert(0, parent[tg])
        tg = parent[tg]

    return fp


def find_path(node_order=[1, 17], graph=Map_Graph):
    # find path function using any shortest path algorithm
    found_path = dijkstra(Map_Graph, node_order[0], node_order[1])

    if len(node_order) > 2:
        for i in range(1, len(node_order)-1):
            found_path.pop()
            found_path.extend(
                dijkstra(Map_Graph, node_order[i], node_order[i+1]))
    return found_path


def check_node_change(total_distance):

    node_change = False
    # write code here
    # compute distance until last node based on visited nodes
    # check distance to next node
    # if total_dist-lastnode_dist> check_dist (+- x margin) send turn flag and direction

    return node_change

# def change_cur_node(node_val):
#     cur_node=node_val
#     visted_node.append(node_val)
#     return True


def get_turn_dir(cur_node, next_node):
    # refers the graph and found path (cur and next node) and returns the turn direction in degrees!!!!
    nodes = Map_Graph[cur_node]
    for x in nodes:
        if x[0] == next_node:
            return x[2]

    return "n"


if __name__ == "__main__":
    path = find_path([1, 6, 11, 15])
    print(path)
    for i in range(len(path)-1):
        print(get_turn_dir(path[i], path[i+1]))