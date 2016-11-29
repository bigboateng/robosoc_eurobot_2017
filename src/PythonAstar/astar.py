from math import sqrt


class AStar(object):
    def __init__(self, graph):
        self.graph = graph

    def heuristic(self, node, start, end):
        return sqrt((end.x - node.x)**2 + (end.y - node.y)**2)
        
    def search(self, start, end):
        discovered = set()
        visited = set()
        current = start
        discovered.add(current)

        while discovered:
            current = min(discovered, key=lambda o: o.h + o.g)

            if current == end:
                path = []
                while current.parent:
                    path.append(current)
                    current = current.parent
                path.append(current)
                return path[::-1]

            discovered.remove(current)
            visited.add(current)

            for node in self.graph[current]:
                if node.obstacle:
                    continue
                if node in visited:
                    continue
                if node in discovered:
                    new_g = current.g + current.move_cost(node)  # new move cost
                    if node.g > new_g:
                        node.g = new_g
                        node.parent = current
                else:
                    node.g = current.g + current.move_cost(node)
                    node.h = self.heuristic(node, start, end)
                    node.parent = current
                    discovered.add(node)
        return None
