import networkx as nx
from typing import *
import heapq

from src.templates.workerprocess import WorkerProcess
from threading import Thread


def dijkstra(G, start, target):
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
        for v in G.adj[u]:
            if v not in d or d[v] > du + 1:
                d[v] = du + 1
                parent[v] = u
                heapq.heappush(pq, (d[v], v))

    fp = [target]
    tg = target

    while tg != start:
        fp.insert(0, parent[tg])
        tg = parent[tg]

    return fp


class PathPlanning:
    def __init__(self, test: bool = False) -> None:
        if test:
            self.graph = nx.read_graphml("path_data/test_track.graphml")
        else:
            self.graph = nx.read_graphml("path_data/comp_track.graphml")

        self.node_dict = self.graph.nodes(data=True)

    def get_path(self, start_idx: str, end_idx: str) -> List[Tuple[int]]:
        path_list = dijkstra(self.graph, start_idx, end_idx)
        return self._convert_nx_path2list(path_list)

    def _convert_nx_path2list(self, path_list) -> List[Tuple[int]]:
        coord_list = []
        for i in path_list:
            data = self.node_dict[i]
            coord_list.append((data["x"], data["y"]))
        return coord_list


# class PathPlanningProcess(WorkerProcess):
#     def __init__(self, inPs, outPs, req_path: Tuple[str], test: bool = False):
#         super().__init__(inPs, outPs)
#         self.path_plan = PathPlanning(test=test)
#         assert len(req_path) == 2, "req_path must of the format (start_idx, end_idx)"
#         self.start_idx = req_path[0]
#         self.end_idx = req_path[1]

#     def run(self):
#         super(PathPlanningProcess, self).run()

#     def _init_threads(self):

#         """Initialize the thread."""
#         if self._blocker.is_set():
#             return

#         thr = Thread(
#             name="PlanningThread",
#             target=self._the_thread,
#             args=(
#                 self.inPs[0],
#                 self.outPs,
#             ),
#         )
#         thr.daemon = True
#         self.threads.append(thr)

#     def _the_thread(self, inP, outPs):
#         while True:
#             try:
#                 # start_idx, end_idx = inP.recv()
#                 coord_path = self.path_plan.get_path(self.start_idx, self.end_idx)
#                 print(coord_path)
#                 for outP in outPs:
#                     outP.send(coord_path)

#                 if not inP:
#                     break

#             except Exception as e:
#                 print("Path Planning Error:")
#                 print(e)
