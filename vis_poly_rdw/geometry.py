import math
from .vec2 import Vec2

EPS = 1e-9

def clamp(v, a, b):
    return max(a, min(b, v))

def sign(v):
    if v > 0: return 1
    if v < 0: return -1
    return 0

def orient(a: Vec2, b: Vec2, c: Vec2):
    # cross of (b-a) x (c-b)
    return sign((b - a).cross(c - b))

def ray_line_intersect(ray_origin: Vec2, ray_dir: Vec2, p1: Vec2, p2: Vec2):
    # Solve ray_origin + t*ray_dir = p1 + u*(p2-p1)
    v = p2 - p1
    denom = ray_dir.cross(v)
    if abs(denom) < EPS:
        return -1.0
    w = p1 - ray_origin
    t = w.cross(v) / denom
    u = w.cross(ray_dir) / denom
    if t >= 0 and 0.0 <= u <= 1.0:
        return t
    return -1.0

def polygon_area(pts):
    # pts: list of Vec2 or list of tuple
    area = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i].x, pts[i].y
        x2, y2 = pts[(i + 1) % n].x, pts[(i + 1) % n].y
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5

def normalize(v: Vec2):
    return v.normalized()

def signed_angle(a: Vec2, b: Vec2):
    # angle from a to b (signed)
    return math.atan2(a.cross(b), a.dot(b))

def angle(a: Vec2, b: Vec2):
    # unsigned angle
    da = signed_angle(a, b)
    return abs(da)
