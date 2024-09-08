class BruteForce:
    def __init__(self):
        self.points = []
    def insert(self,point):
        self.points.append(point)

    def nearestNeighbour(self,point,distancefnc):
        best_dist = float('inf')
        best_node = None
        for p in self.points:
            cur_dist = distancefnc(point,p)
            if cur_dist < best_dist:
                best_dist = cur_dist
                best_node = p
        return best_node
