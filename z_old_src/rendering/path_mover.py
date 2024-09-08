import pymunk
from typing import List
from ..RRTNode import RRTNodeTimed,RRTNode

#if only static path is given, this is the velocity
VELOCITY = 100


class PathMover:
    def __init__(self, path: List[RRTNode], body: pymunk.Body, FPS):
        self.path = convert_path(path)
        self.current = 0  # current waypoint trying to reach
        self.body = body
        self.threshtime = 1 / (FPS)  # time to waypoint to consider it reached
        self.last_change_time = 0
        self.already_called = False

    def move(self, time):
        """
        Called every frame to update the position of the body,
        it converts the path to velocity waypoints and moves the body accordingly
        correction via position setting are done or errors will accumulate
        :param time:
        :return:
        """
        if len(self.path) == 0:
            return
        if not self.already_called:
            self.update_velocity(time)
            self.already_called = True


        cur_waypoint = self.path[self.current]
        delta = time - (self.last_change_time + cur_waypoint[3])

        if abs(delta) < self.threshtime or delta > 0:
            self.current += 1
            if self.current < len(self.path):
                self.update_velocity(time)

    def update_velocity(self, time):
        cur_waypoint = self.path[self.current]
        self.body.position = cur_waypoint[4].x, cur_waypoint[4].y
        self.body.angle = cur_waypoint[4].angle
        self.body.velocity = cur_waypoint[0], cur_waypoint[1]
        self.body.angular_velocity = cur_waypoint[2]
        self.last_change_time = time
        self.body.space.reindex_shapes_for_body(self.body)


    @staticmethod
    def get_length(tp):
        return (tp[0] ** 2 + tp[1] ** 2) ** 0.5


def convert_path(path):
    """
    Converts position waypoints (x,y,angle) to velocity waypoints (xvel,yvel,anglularvel,time)
    :param path:
    :return:
    """
    # print(path)
    converted = []
    for i in range(len(path) - 1):
        node = path[i]
        next_node = path[i + 1]
        dir_x = next_node.x - node.x
        dir_y = next_node.y - node.y

        angle = next_node.angle - node.angle
        length = PathMover.get_length((dir_x, dir_y))

        t = length / VELOCITY
        if isinstance(node, RRTNodeTimed):
            # print(next_node.time, node.time)
            t = next_node.time - node.time
        vel = length / t

        elem_dir_x = dir_x / length * vel
        elem_dir_y = dir_y / length * vel
        converted.append((elem_dir_x, elem_dir_y, angle / t, t, node))
    if len(converted) == 0:
        return []
    converted.append((0, 0, 0, float("inf"),converted[-1][4]))
    return converted
