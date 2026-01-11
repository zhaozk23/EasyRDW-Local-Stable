"""
Python translation of vis_poly_rdw and visibility-polygon based RDW.

This is a compact, self-contained translation of the core logic in the
provided C++ sources. It implements a simple visibility-polygon routine (ray
casting to all obstacle/vertex angles) and the RDW decision logic that
computes steer targets and sets gains.
"""
import math
from .vec2 import Vec2, rad_2_vec
from .visibility_polygon import VisibilityPolygon
from .geometry import ray_line_intersect, normalize


class RedirectionUnit:
    def __init__(self):
        self.apply_rota = False
        self.apply_trans = False
        self.apply_curve = False
        self.apply_bend = False
        self.rota_gain = 1.0
        self.trans_gain = 1.0
        self.curve_gain = 0.0
        self.curve_gain_dir = 1


class VisPolyRdw:
    def __init__(self, phys_env=None, virt_env=None, resetter=None):
        self.name = "Vis. Poly. RDW"
        self.cur_rota_gain = 1.0
        self.min_rota_gain = 0.67
        self.max_rota_gain = 1.24
        self.prev_rota_gain = 1.0

        self.cur_trans_gain = 1.0
        self.min_trans_gain = 0.86
        self.max_trans_gain = 1.26

        self.curve_radius = 7.5
        self.cur_curve_per_deg = 1.0 / self.curve_radius
        self.curve_dir = 1

        self.reset_policy = resetter

        self.prev_loss = -1.0
        self.cur_loss = -1.0

        self.phys_vis_poly = None
        self.virt_vis_poly = None
        self.steer_target = Vec2(0, 0)
        self.best_slice = None

    def get_vis_poly(self, pos: Vec2, env: dict, heading: float):
        # env expected to be a dict with 'vertices' (list of Vec2) and
        # 'obstacles' (list of list of Vec2). Implements a simple angular
        # sweep visibility algorithm.
        segments = []
        verts = env.get('vertices', [])
        for i in range(len(verts)):
            p1 = verts[i]
            p2 = verts[(i + 1) % len(verts)]
            segments.append((p1, p2))
        for obs in env.get('obstacles', []):
            for i in range(len(obs)):
                p1 = obs[i]
                p2 = obs[(i + 1) % len(obs)]
                segments.append((p1, p2))

        # collect unique angles towards every vertex
        angles = []
        eps = 1e-6
        for s in segments:
            for p in s:
                dx = p.x - pos.x
                dy = p.y - pos.y
                ang = math.atan2(dy, dx)
                angles.extend([ang - eps, ang, ang + eps])

        intersections = []
        for ang in sorted(set(angles)):
            dir = rad_2_vec(ang)
            best_t = float('inf')
            best_pt = None
            for seg in segments:
                t = ray_line_intersect(pos, dir, seg[0], seg[1])
                if t != -1.0 and t < best_t:
                    best_t = t
                    best_pt = Vec2(pos.x + dir.x * t, pos.y + dir.y * t)
            if best_pt is not None:
                intersections.append((ang, best_pt))

        # sort by angle and return visibility polygon in absolute coords
        intersections.sort(key=lambda x: x[0])
        pts = [p for a, p in intersections]
        return VisibilityPolygon(pts, pos, heading)

    def update_visibility_polygons(self, egocentric_user):
        # egocentric_user expected to provide state.get_virt_pos(), state.get_phys_pos(),
        # virtual_env(), physical_env() returning env dicts
        self.virt_vis_poly = self.get_vis_poly(egocentric_user.state.get_virt_pos(), egocentric_user.virtual_env(), egocentric_user.state.get_virt_heading())
        self.phys_vis_poly = self.get_vis_poly(egocentric_user.state.get_phys_pos(), egocentric_user.physical_env(), egocentric_user.state.get_phys_heading())

    def set_steer_target(self, egocentric_user):
        if not self.virt_vis_poly or not self.phys_vis_poly:
            return
        if not self.virt_vis_poly.slices or not self.phys_vis_poly.slices:
            return
        virt_slice = self.virt_vis_poly.slices[0]
        best_diff = float('inf')
        best = None
        for s in self.phys_vis_poly.slices:
            # limit to reasonable frontal slices
            if abs(s.theta_offset) > math.pi * 0.5:
                continue
            diff = abs(virt_slice.area - s.area)
            if diff < best_diff:
                best_diff = diff
                best = s
        if best is None:
            best = self.phys_vis_poly.slices[0]
        self.best_slice = best
        self.steer_target = egocentric_user.state.get_phys_pos() + best.bisector

    def update_loss(self, sim_state, egocentric_user):
        # placeholder: set losses to zero. Real implementation should compute
        # differences between phsyical and virtual visibility in directions.
        self.cur_loss = 0.0

    def set_gains(self, dx, dy, dtheta, sim_state, egocentric_user):
        redir = RedirectionUnit()
        user_dir = rad_2_vec(egocentric_user.state.get_phys_heading())
        user_to_target = (self.steer_target - egocentric_user.state.get_phys_pos()).normalized()
        angle_to_gradient = math.atan2(user_dir.cross(user_to_target), user_dir.dot(user_to_target))

        if abs(dtheta) > 1e-9:
            if math.copysign(1, angle_to_gradient) == -1:
                if dtheta < 0:
                    redir.rota_gain = self.max_rota_gain
                else:
                    redir.rota_gain = self.min_rota_gain
            else:
                if dtheta < 0:
                    redir.rota_gain = self.min_rota_gain
                else:
                    redir.rota_gain = self.max_rota_gain
            redir.apply_rota = True
        elif abs(dx) > 0 or abs(dy) > 0:
            if math.copysign(1, angle_to_gradient) == -1:
                redir.curve_gain = self.cur_curve_per_deg
                redir.curve_gain_dir = -1
            else:
                redir.curve_gain = self.cur_curve_per_deg
                redir.curve_gain_dir = 1

            virt_h = self.virt_vis_poly.slices[0].avg_height if self.virt_vis_poly.slices else 1.0
            phys_h = self.phys_vis_poly.slices[0].avg_height if self.phys_vis_poly.slices else 1.0
            trans_gain = phys_h / virt_h if virt_h != 0 else 1.0
            trans_gain = max(self.min_trans_gain, min(self.max_trans_gain, trans_gain))
            redir.trans_gain = trans_gain
            redir.apply_trans = True
            redir.apply_curve = True

        # update internal state
        self.cur_rota_gain = redir.rota_gain
        self.cur_trans_gain = redir.trans_gain
        self.cur_curve_gain = redir.curve_gain
        self.curve_dir = redir.curve_gain_dir
        return redir

    def update(self, dx, dy, dtheta, sim_state, egocentric_user):
        self.update_visibility_polygons(egocentric_user)
        self.set_steer_target(egocentric_user)
        self.update_loss(sim_state, egocentric_user)
        ru = self.set_gains(dx, dy, dtheta, sim_state, egocentric_user)
        self.prev_loss = self.cur_loss
        return ru
