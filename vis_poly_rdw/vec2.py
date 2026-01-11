import math


class Vec2:
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = float(x)
        self.y = float(y)

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, s: float):
        return Vec2(self.x * s, self.y * s)

    __rmul__ = __mul__

    def __truediv__(self, s: float):
        return Vec2(self.x / s, self.y / s)

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def tuple(self):
        return (self.x, self.y)

    def length(self):
        return math.hypot(self.x, self.y)

    def normalized(self):
        l = self.length()
        if l == 0:
            return Vec2(0.0, 0.0)
        return self / l

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def cross(self, other):
        return self.x * other.y - self.y * other.x

    def rotate(self, theta_rad: float):
        c = math.cos(theta_rad)
        s = math.sin(theta_rad)
        return Vec2(self.x * c - self.y * s, self.x * s + self.y * c)

def rad_2_vec(rad: float):
    return Vec2(math.cos(rad), math.sin(rad))

def polar(origin: Vec2, point: Vec2):
    dx = point.x - origin.x
    dy = point.y - origin.y
    r = math.hypot(dx, dy)
    theta = math.atan2(dy, dx)
    return Vec2(r, theta)
