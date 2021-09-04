import json
import math
import random
from datetime import datetime
import numpy as np


def bounds_collision(pos, radius):
    # check bounds
    for i in range(len(pos)):
        if pos[i] < 0.0 + radius or pos[i] > 1.0 - radius:
            return True
    return False


class Generator:
    def __init__(self, dimensions):
        self.n = dimensions
        self.n_obstacles = 5
        self.obstacles = []
        self.collision_granularity = 10
        self.start = np.array([])
        self.goal = np.array([])
        self.max_tries = 1000
        self.radius_min = 0.01
        self.radius_max = 0.05
        self.speed_min_factor = 0.02
        self.speed_max_factor = 0.2
        self.agent_radius = 0.03

    def random_walk(self, start, start_time, v, radius):
        n_try = 0
        while n_try < self.max_tries:
            n_try += 1
            point = np.random.uniform(-1.0, 1.0, size=self.n)
            direction = v * (point / np.sqrt(np.sum(point ** 2)))

            # check collision for endpoint
            if self.has_collision(start + direction, radius, start_time + 1):
                continue

            # check collision for pathway
            if self.pathway_collision(start, direction, radius, start_time):
                continue
            return start + direction
        print("Couldn't sample Random Walk without collision!")
        return None

    def sample_collision_free(self, radius):
        n_try = 0
        while n_try < self.max_tries:
            n_try += 1
            pos = np.random.uniform(radius, 1.0 - radius, size=self.n)
            if self.has_collision(pos, radius, 0):
                continue
            return pos
        print("Couldn't sample new Point without collision!")
        return None

    def has_collision(self, pos, radius, time):
        if bounds_collision(pos, radius):
            return True
        if self.start.size != 0:
            dist = np.linalg.norm(pos - self.start)
            if dist < radius + self.agent_radius:
                return True
        if self.goal.size != 0:
            dist = np.linalg.norm(pos - self.goal)
            if dist < radius + self.agent_radius:
                return True
        if self.obstacle_collision(pos, radius, time):
            return True
        return False

    def obstacle_collision(self, pos, radius, time):
        # in between seconds case:
        if time - int(time) > 0.0001:
            first = int(time)
            for o in self.obstacles:
                o_direction = o["path"][first + 1] - o["path"][first]
                o_collision = o["path"][first] + (time - int(time)) * o_direction
                dist = np.linalg.norm(o_collision - pos)
                if dist < radius + o["r"]:
                    return True
        # full second case:
        else:
            for o in self.obstacles:
                dist = np.linalg.norm(o["path"][int(time)] - pos)
                if dist < radius + o["r"]:
                    return True
        return False

    def pathway_collision(self, start, direction, radius, start_time):
        for i in range(1, self.collision_granularity):
            t = start_time + i / self.collision_granularity
            pos = start + i / self.collision_granularity * direction
            if self.has_collision(pos, radius, t):
                return True
        return False

    def generate_dataset(self):
        random.seed()
        self.start = self.sample_collision_free(self.agent_radius)
        self.goal = self.sample_collision_free(self.agent_radius)

        # calculate radii bounds for the obstacle spheres
        # ranging from 0.5% to 5% of total volume
        # V_n(r) = (pi^(n/2)) / (gamma(n/2 + 1) * r^n
        # r_min = ((self.volume_min * math.gamma(self.n / 2.0 + 1.0)) / (math.pi ** (self.n / 2.0))) ** (1.0 / self.n)
        # r_max = ((self.volume_max * math.gamma(self.n / 2.0 + 1.0)) / (math.pi ** (self.n / 2.0))) ** (1.0 / self.n)

        # calculate bounds for speed
        v_min = math.sqrt(self.n) * self.speed_min_factor
        v_max = math.sqrt(self.n) * self.speed_max_factor

        for i in range(self.n_obstacles):
            print('obstacle ' + str(i))
            r = random.uniform(self.radius_min, self.radius_max)
            v = random.uniform(v_min, v_max)

            first = self.sample_collision_free(r)
            if first is None:
                return
            path = [first]
            for j in range(4):
                next_element = self.random_walk(path[j], j, v, r)
                if next_element is None:
                    return
                path.append(next_element)
            for j in range(3, 0, -1):
                path.append(path[j])
            self.obstacles.append(
                {"r": r,
                 "path": path})

        self.export_to_json()

    def export_to_json(self):
        d = {
            "start": self.start.tolist(),
            "goal": self.goal.tolist(),
            "radius": self.agent_radius,
            "obstacles": []
        }
        for o in self.obstacles:
            path = []
            for point in o["path"]:
                path.append(point.tolist())
            d["obstacles"].append({
                "r": o["r"],
                "path": path
            })
        json_string = json.dumps(d, indent=4)
        now = datetime.now()
        filename = 'data/testsets/' + str(self.n) + '/' + now.strftime("%m-%d %H:%M:%S") + '.json'
        # filename = "test.json"
        json_file = open(filename, "x")
        json_file.write(json_string)
        json_file.close()


if __name__ == '__main__':
    dim = 2
    generator = Generator(dim)
    generator.generate_dataset()

