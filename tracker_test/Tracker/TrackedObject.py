from random import randint


class TrackedObject:
    tracks = []

    def __init__(self, i, xi, yi, max_age):
        self.i = i
        self.x = xi
        self.y = yi
        self.tracks = []
        self.age = 0
        self.max_age = max_age
        self.dead = False

        self.R = randint(0, 255)
        self.G = randint(0, 255)
        self.B = randint(0, 255)

    def get_rgb(self):
        return self.R, self.G, self.B

    def get_tracks(self):
        return self.tracks

    def get_id(self):
        return self.i

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def update_coords(self, xn, yn):
        self.age = 0
        self.tracks.append([self.x, self.y])
        self.x = xn
        self.y = yn

    def same_coords(self, xn, yn):
        return xn == self.x and yn == self.y

    def timed_out(self):
        return self.dead

    def age_one(self):
        self.age += 1

        if self.age > self.max_age:
            self.dead = True
        return True
