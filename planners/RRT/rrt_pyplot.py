
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.animation import FuncAnimation

# Joint Limit Configuration Parameters
LIMITS = [[-10.0, 10.0], [-25.0, 25.0]]

# RRT Configuration Parameters
DELTA_Q = 1.0
REACHED_RAND_DISTANCE = 2.0
NUM_ITERATIONS = 100
NUM_DIMENSIONS = len(LIMITS)

class Config:
    def __init__(self, values):
        self.values = values
        self.identifier = str(values)

    def distance(self, other):
        return np.linalg.norm(np.array(self.values) - np.array(other.values))

    def length(self):
        return np.linalg.norm(np.array(self.values))

    def id(self):
        return self.identifier

class RRT:
    def __init__(self, init_config):
        self.limits = LIMITS
        self.graph = {init_config.id():[]}
        # self.fig, self.ax = plt.subplots()
        self.data = []
        self.num_dimensions = len(init_config.values)

    def get_config_from_id(self, str_identifier):
        p = [float(i) for i in str_identifier.strip('[]').split(', ')] 
        
        return Config(p)


    def add_config(self, near_config, new_config):
        self.graph[new_config.id()] = []

        connected = self.graph[near_config.id()] + [new_config.id()]
        self.graph[near_config.id()] = connected


    def get_nearest_vertex(self, config):
        nearest_vertex = None
        nearest_dist = np.inf

        for vertex_str in self.graph.keys():
            vertex = self.get_config_from_id(vertex_str)
            dist = config.distance(vertex)

            if dist < nearest_dist:
                nearest_vertex = vertex
                nearest_dist = dist

        return nearest_vertex

    def get_new_vertex(self, config_near, config_rand, delta_config):
        dist = config_near.distance(config_rand)

        if abs(dist) < REACHED_RAND_DISTANCE:
            return None

        new_values = []

        for indx in range(len(config_near.values)):
            delta = config_rand.values[indx] - config_near.values[indx]
            new_values.append(config_near.values[indx] + delta_config/dist*(delta))

        new_vertex = Config(new_values)

        return new_vertex


    def get_rand_config(self):
        rand_values = []

        for limit in self.limits:
            rand_values.append(round(np.random.uniform(*limit), 4))

        config = Config(rand_values)

        return config

    def build_graph(self, num_iterations, delta_q):
        for k in range(num_iterations):
            qrand = self.get_rand_config()
            qnear = self.get_nearest_vertex(qrand)

            qnew = self.get_new_vertex(qnear, qrand, delta_q)
            qnear_this_iteration= []

            while qnew:
                qnew = self.get_new_vertex(qnear, qrand, delta_q)
                if qnew:
                    self.add_config(qnear, qnew)
                    qnear_this_iteration.append(qnew.values)
                    qnear = qnew

            self.data.append(qnear_this_iteration)
        
        return


    def animate_tree(self):
        # Set up formatting for the movie files
        Writer = matplotlib.animation.writers['ffmpeg']
        writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
        
        fig, ax = plt.subplots()
        xdata, ydata = [], []
        ln, = plt.plot([], [], 'ro')
        def init():
            ax.set_xlim(*self.limits[0])
            ax.set_ylim(*self.limits[1])
            return ln,

        def update(frame):
            for point in frame:
                xdata.append(point[0])
                ydata.append(point[1])
            ln.set_data(xdata, ydata)
            return ln,

        ani = FuncAnimation(fig, update, frames=self.data,
                            init_func=init, blit=True)
        plt.show()

        # ani.save('im.mp4', writer=writer)


def main():  
    init_config = Config([0.0]*NUM_DIMENSIONS)
    rrt = RRT(init_config)

    rrt.build_graph(num_iterations=NUM_ITERATIONS, delta_q=DELTA_Q)
    rrt.animate_tree()

if __name__ == "__main__":
    main()

    input("Press <enter> to exit")