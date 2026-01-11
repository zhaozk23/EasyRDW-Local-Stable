import math
from .vec2 import Vec2, rad_2_vec, polar
from .geometry import orient, signed_angle, angle, polygon_area, normalize

SLICE_THETA_THRESHOLD = 0.0174533  # ~1 degree in radians? keep as in C++

class Slice:
    def __init__(self, pts, heading):
        # pts: list of Vec2, already relative to center
        self.pts = [Vec2(p.x, p.y) for p in pts]
        self.pts.append(Vec2(0.0, 0.0))
        self.p1 = self.pts[0]
        self.p2 = self.pts[-2]
        self.p1_theta = signed_angle(rad_2_vec(heading), normalize(self.p1))
        self.p2_theta = signed_angle(rad_2_vec(heading), normalize(self.p2))
        p1_to_p2 = self.p2 - self.p1
        self.bisector = normalize(self.p1 + (p1_to_p2 * 0.5))
        self.theta_offset = angle(rad_2_vec(heading), self.bisector)
        self.width = abs(angle(normalize(self.p1), normalize(self.p2)))
        self.area = polygon_area(self.pts)
        self.avg_height = 0.0
        for v in self.pts[:-1]:
            self.avg_height += v.length()
        if len(self.pts) > 1:
            self.avg_height = self.avg_height / (len(self.pts) - 1)
        else:
            self.avg_height = 0.0

class VisibilityPolygon:
    def __init__(self, boundary_pts=None, center=None, heading=0.0, env=None):
        # boundary_pts: list of Vec2 (absolute coords) OR None when using env
        self.verts = []
        self.slices = []
        self.center = None
        self.env = env
        self.heading = heading
        if boundary_pts is not None and center is not None:
            self.center = Vec2(center.x, center.y)
            for v in boundary_pts:
                self.verts.append(Vec2(v.x - center.x, v.y - center.y))
            self.compute_slices()

    def simplify(self):
        # Not implemented in detail; placeholder
        pass

    def compute_slices(self):
        self.slices = []
        n = len(self.verts)
        for i in range(n):
            p1 = self.verts[i]
            p2 = self.verts[(i + 1) % n]
            # skip colinear or too-narrow slices
            if orient(p1, p2, Vec2(0, 0)) == 0:
                continue
            if signed_angle(normalize(p1), normalize(p2)) <= SLICE_THETA_THRESHOLD:
                continue
            s = Slice([p1, p2], self.heading)
            self.slices.append(s)
        self.slices.sort(key=lambda s: s.theta_offset)
