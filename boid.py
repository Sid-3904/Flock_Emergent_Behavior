import matplotlib.pyplot as plt
import numpy as np
import random
import matplotlib.animation as animation

# hyperparameters
vmax = 5
vmin = 3
r2 = 500
close2 = 2000
avoid = 0.01
aligning = 0.05
cohede = 0.05
turnfactor = 0.8
biasval = 0.005

# dimensions
n = 50
L = -100
R = 100
U = 100
D = -100
margin = 20
dt = 10

# boid class
class Boid:
    def __init__(self, x, y, vx, vy, b_id):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.b_id = b_id
        self.neighbours = []
        self.group = random.choice([0, 0, 1, 2])

    def move(self):
        # separation
        close_dx = 0
        close_dy = 0
        # alignment
        avg_vx = 0
        avg_vy = 0
        # cohesion
        avg_x = 0
        avg_y = 0
        for boid in self.neighbours:
            dx = self.x - boid.x
            dy = self.y - boid.y
            avg_vx += boid.vx
            avg_vy += boid.vy
            avg_x += boid.x
            avg_y += boid.y
            if dx**2 + dy**2 < close2:
                close_dx += dx
                close_dy += dy
        if self.neighbours :
            n = len(self.neighbours)
            avg_vx = sum(boid.vx for boid in self.neighbours)/n
            avg_vy = sum(boid.vy for boid in self.neighbours)/n
            avg_x = sum(boid.x for boid in self.neighbours)/n
            avg_y = sum(boid.y for boid in self.neighbours)/n
        else :
            avg_vx = self.vx
            avg_vy = self.vy
            avg_x = self.x
            avg_y = self.y
        
        self.vx += close_dx * avoid + (avg_x - self.x) * cohede + (avg_vx - self.vx) * aligning
        self.vy += close_dy * avoid + (avg_y - self.y) * cohede + (avg_vy - self.vy) * aligning

        # wall avoidance
        if self.x < L+margin:
            self.vx = self.vx + turnfactor
        if self.x > R-margin:
            self.vx = self.vx - turnfactor
        if self.y > U-margin:
            self.vy = self.vy - turnfactor
        if self.y < D+margin:
            self.vy = self.vy + turnfactor

        if self.group == 1 :
            self.vx = (1 - biasval)*self.vx + biasval
        # biased to left of screen
        elif self.group == 2 :
            self.vx = (1 - biasval)*self.vx -biasval

        # speed check
        speed = (self.vx ** 2 + self.vy ** 2) ** 0.5
        if speed > vmax:
            self.vx = (self.vx / speed) * vmax
            self.vy = (self.vy / speed) * vmax
        if speed < vmin:
            self.vx = (self.vx / speed) * vmin
            self.vy = (self.vy / speed) * vmin
        
        # update position
        self.x += self.vx
        self.y += self.vy

# utility functions
def distance(boid1, boid2):
    return (boid1.x - boid2.x) ** 2 + (boid1.y - boid2.y) ** 2

# maintain close boids id list
def update_neighbours(flock, boid):
    boid.neighbours = [other_boid for other_boid in flock if other_boid != boid and distance(boid, other_boid) < r2]
def motion(flock):
    def update(frame):
        for boid in flock:
            update_neighbours(flock, boid)
            boid.move()
        arrows = []
        for boid in flock:
            arrows.append(ax.arrow(boid.x, boid.y, boid.vx, boid.vy, head_width=1, head_length=1, length_includes_head=True, fc='red', ec='red'))
        return arrows

    fig, ax = plt.subplots()
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Flock Movement Simulation')
    ax.set_xlim(L, R)
    ax.set_ylim(D, U)
    ani = animation.FuncAnimation(fig, update, frames=range(100), interval=dt, blit=True)
    plt.show()

# initialize flock
flock = [Boid(random.uniform(L+margin, R-margin), random.uniform(D+margin, U-margin), random.uniform(0, 3*vmax), random.uniform(vmin, vmax), b_id) for b_id in range(n)]
motion(flock)