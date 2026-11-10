
"""
Lane-change (cut-in) collision scenario.

Setup: the ego drives in its lane at a constant target speed. A second
vehicle starts `initial_gap_m` ahead in an adjacent same-direction lane
and, at `lane_change_start_time_s`, cuts into the ego lane over
`lane_change_duration_s`.

Both vehicles follow ONE precomputed reference route (the ego lane).
The cut-in vehicle tracks the same route with a lateral offset that
shrinks from one lane width to zero during the manoeuvre. This keeps the
trajectory consistent across road/lane-ID changes and junctions, which
is where the purely time-based waypoint stepping used to break down.
"""

import json
import math
import random

import carla
import pygame

from safety_metrics import get_speed_kmh
from camera_manager import (
    configure_camera_blueprint,
    spawn_ego_camera,
    spawn_drone_camera,
)
from result_logger import (
    create_results_directory,
    create_csv_writer,
    save_summary,
)


FIXED_DELTA_S = 0.05
ROUTE_STEP_M = 1.0
SPAWN_Z_OFFSET_M = 0.3
LANE_CHANGE_DONE_TOLERANCE_M = 0.5

LANE_CHANGE_CSV_FIELDS = [
    "time_s",
    "ego_speed_kmh",
    "cutin_speed_kmh",
    "lane_change_progress",
    "cutin_lateral_offset_from_ego_lane_m",
    "longitudinal_offset_m",
    "lateral_offset_m",
    "longitudinal_gap_m",
    "center_distance_m",
    "cutin_in_ego_path",
    "approx_ttc_s",
    "collision",
]


# ----------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------

def _dot2(a, b):
    return a.x * b.x + a.y * b.y


def _yaw_diff_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def _same_direction(wp_a, wp_b):
    return _dot2(
        wp_a.transform.get_forward_vector(),
        wp_b.transform.get_forward_vector(),
    ) > 0.9


def _neighbor_lane(wp, side):
    """Adjacent same-direction driving lane on `side`, or None."""
    nb = wp.get_left_lane() if side == "left" else wp.get_right_lane()
    if nb is None or nb.lane_type != carla.LaneType.Driving:
        return None
    if not _same_direction(wp, nb):
        return None
    return nb


def _next_straightest(wp, step):
    """Next waypoint, preferring the branch that keeps the heading."""
    options = wp.next(step)
    if not options:
        return None
    yaw = wp.transform.rotation.yaw
    return min(
        options,
        key=lambda o: abs(_yaw_diff_deg(o.transform.rotation.yaw, yaw)),
    )


def _clear_run_length(start_wp, side, max_len, step=5.0):
    """Distance along the lane over which the `side` neighbour lane
    keeps existing (and no junction is entered)."""
    travelled = 0.0
    wp = start_wp
    while travelled < max_len:
        if wp.is_junction or _neighbor_lane(wp, side) is None:
            break
        wp = _next_straightest(wp, step)
        if wp is None:
            break
        travelled += step
    return travelled


def _lifted(transform):
    loc = transform.location
    return carla.Transform(
        carla.Location(x=loc.x, y=loc.y, z=loc.z + SPAWN_Z_OFFSET_M),
        transform.rotation,
    )


# ----------------------------------------------------------------------
# Spawn selection
# ----------------------------------------------------------------------

def choose_lane_change_candidates(
    carla_map, gap_m, required_length_m, side_preference="adjacent_lane"
):
    """
    Return spawn candidates, best first.

    A candidate is an ego waypoint plus a cut-in waypoint `gap_m` ahead
    in the adjacent lane. Candidates whose adjacent lane exists for at
    least `required_length_m` (no junctions) come first, in random order.
    """
    if side_preference in ("left", "right"):
        sides = (side_preference,)
    else:
        sides = ("left", "right")

    good, fallback = [], []

    for spawn in carla_map.get_spawn_points():
        ego_wp = carla_map.get_waypoint(
            spawn.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if ego_wp is None or ego_wp.is_junction:
            continue

        for side in sides:
            if _neighbor_lane(ego_wp, side) is None:
                continue

            for ego_ahead in ego_wp.next(float(gap_m)):
                if (
                    ego_ahead.road_id != ego_wp.road_id
                    or ego_ahead.lane_id != ego_wp.lane_id
                ):
                    continue

                cutin_wp = _neighbor_lane(ego_ahead, side)
                if cutin_wp is None:
                    continue

                run = _clear_run_length(ego_wp, side, required_length_m)
                candidate = {
                    "ego_wp": ego_wp,
                    "cutin_wp": cutin_wp,
                    "cutin_side": side,
                    "clear_length_m": run,
                }
                if run >= required_length_m:
                    good.append(candidate)
                else:
                    fallback.append(candidate)

    if not good and not fallback:
        raise RuntimeError(
            "No adjacent same-direction lane pair found. "
            "Try another map or a different initial gap."
        )

    random.shuffle(good)
    fallback.sort(key=lambda c: c["clear_length_m"], reverse=True)

    print(
        f"[INFO] Lane-change spawn candidates: {len(good)} with "
        f">= {required_length_m:.0f} m of adjacent lane, "
        f"{len(fallback)} shorter."
    )
    if not good:
        print(
            "[WARN] No candidate has enough straight adjacent lane; "
            "using the longest available."
        )
    return good + fallback


# ----------------------------------------------------------------------
# Reference route and controllers
# ----------------------------------------------------------------------

class LaneRoute:
    """Dense polyline along one lane, with a nearest-point tracker."""

    def __init__(self, start_wp, length_m, step=ROUTE_STEP_M):
        self.step = step
        self.locations = []
        self.rights = []
        self.lane_widths = []

        wp = start_wp
        travelled = 0.0
        while wp is not None and travelled <= length_m:
            tf = wp.transform
            self.locations.append(tf.location)
            self.rights.append(tf.get_right_vector())
            self.lane_widths.append(wp.lane_width)
            wp = _next_straightest(wp, step)
            travelled += step

        if len(self.locations) < 2:
            raise RuntimeError("Reference route is too short.")

    def __len__(self):
        return len(self.locations)

    def nearest_index(self, location, start=0, window=None):
        end = len(self) if window is None else min(len(self), start + window)
        best_i, best_d = start, float("inf")
        for i in range(start, end):
            d = self.locations[i].distance(location)
            if d < best_d:
                best_i, best_d = i, d
        return best_i

    def point(self, index, lateral_offset=0.0):
        """Route point `index`, shifted right by `lateral_offset` m."""
        index = max(0, min(index, len(self) - 1))
        loc = self.locations[index]
        right = self.rights[index]
        return carla.Location(
            x=loc.x + right.x * lateral_offset,
            y=loc.y + right.y * lateral_offset,
            z=loc.z,
        )

    def lateral_deviation(self, index, location):
        """Signed lateral distance (right positive) of `location`."""
        loc = self.locations[index]
        right = self.rights[index]
        return (location.x - loc.x) * right.x + (location.y - loc.y) * right.y


class RouteFollower:
    """Pure-pursuit steering + PI speed control along a LaneRoute."""

    def __init__(self, vehicle, route, kp=0.08, ki=0.04):
        self.vehicle = vehicle
        self.route = route
        # A freshly spawned actor reports (0, 0, 0) until the next
        # world.tick() in synchronous mode, so locate it lazily.
        self.index = None
        self.kp = kp
        self.ki = ki
        self.integral = 0.0

        physics = vehicle.get_physics_control()
        wheels = physics.wheels
        self.max_steer_rad = math.radians(
            wheels[0].max_steer_angle if wheels else 70.0
        )
        if len(wheels) >= 4:
            front = wheels[0].position
            rear = wheels[2].position
            # Wheel positions are in centimetres.
            self.wheelbase = max(
                1.5,
                math.sqrt(
                    (front.x - rear.x) ** 2 + (front.y - rear.y) ** 2
                ) / 100.0,
            )
        else:
            self.wheelbase = 2.8

    def update_index(self):
        location = self.vehicle.get_location()
        if self.index is not None:
            self.index = self.route.nearest_index(
                location, start=max(0, self.index - 5), window=65
            )
        # (Re)localise over the whole route on the first call, or if the
        # windowed search has lost the vehicle.
        if (
            self.index is None
            or self.route.locations[self.index].distance(location) > 5.0
        ):
            self.index = self.route.nearest_index(location)
        return self.index

    def lookahead_m(self):
        speed_mps = get_speed_kmh(self.vehicle) / 3.6
        return max(6.0, min(20.0, 4.0 + 0.6 * speed_mps))

    def steer_towards(self, target):
        tf = self.vehicle.get_transform()
        fwd = tf.get_forward_vector()
        right = tf.get_right_vector()
        dx = target.x - tf.location.x
        dy = target.y - tf.location.y
        x_f = dx * fwd.x + dy * fwd.y
        y_r = dx * right.x + dy * right.y
        ld2 = max(1e-3, x_f * x_f + y_r * y_r)
        curvature = 2.0 * y_r / ld2
        delta = math.atan(self.wheelbase * curvature)
        return max(-1.0, min(1.0, delta / self.max_steer_rad))

    def speed_command(self, target_speed_kmh):
        error = target_speed_kmh - get_speed_kmh(self.vehicle)
        self.integral = max(
            -15.0, min(15.0, self.integral + error * FIXED_DELTA_S)
        )
        u = self.kp * error + self.ki * self.integral
        if u >= 0:
            return min(1.0, u), 0.0
        return 0.0, min(1.0, -u)

    def control(self, target_speed_kmh, lateral_offset=0.0):
        self.update_index()
        la = self.lookahead_m()
        target = self.route.point(
            self.index + int(round(la / self.route.step)),
            lateral_offset,
        )
        throttle, brake = self.speed_command(target_speed_kmh)
        self.vehicle.apply_control(
            carla.VehicleControl(
                throttle=throttle,
                brake=brake,
                steer=self.steer_towards(target),
            )
        )
        return la


def lane_change_alpha(t, start_s, duration_s):
    """Smoothstep lane-change progress in [0, 1] at time t."""
    if t <= start_s:
        return 0.0
    p = min(1.0, (t - start_s) / duration_s)
    return p * p * (3.0 - 2.0 * p)


# ----------------------------------------------------------------------
# Safety metrics (ego frame)
# ----------------------------------------------------------------------

def relative_kinematics(ego, other):
    """
    Position of `other` in the ego frame and derived surrogate metrics.

    TTC is only reported when `other` is ahead of the ego and laterally
    overlapping its footprint (i.e. it is in the ego's path).
    """
    etf = ego.get_transform()
    fwd = etf.get_forward_vector()
    right = etf.get_right_vector()
    e_loc = etf.location
    o_loc = other.get_location()

    d = carla.Vector3D(o_loc.x - e_loc.x, o_loc.y - e_loc.y, 0.0)
    lon = _dot2(d, fwd)
    lat = _dot2(d, right)

    e_ext = ego.bounding_box.extent
    o_ext = other.bounding_box.extent
    lon_gap = abs(lon) - (e_ext.x + o_ext.x)
    in_path = abs(lat) < (e_ext.y + o_ext.y)

    ev = ego.get_velocity()
    ov = other.get_velocity()
    closing = _dot2(carla.Vector3D(ev.x - ov.x, ev.y - ov.y, 0.0), fwd)

    ttc = None
    if in_path and lon > 0 and lon_gap > 0 and closing > 0.1:
        ttc = lon_gap / closing

    return {
        "longitudinal_offset_m": lon,
        "lateral_offset_m": lat,
        "longitudinal_gap_m": lon_gap,
        "center_distance_m": math.hypot(lon, lat),
        "in_path": in_path,
        "closing_speed_mps": closing,
        "ttc_s": ttc,
    }


# ----------------------------------------------------------------------
# Scenario runner
# ----------------------------------------------------------------------

def _find_blueprint(library, name, role):
    matches = library.filter(name)
    if not matches:
        raise RuntimeError(f"{role} blueprint not found: {name}")
    bp = matches[0]
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", role)
    return bp


def _set_initial_speed(vehicle, speed_kmh):
    fwd = vehicle.get_transform().get_forward_vector()
    v = speed_kmh / 3.6
    vehicle.set_target_velocity(carla.Vector3D(fwd.x * v, fwd.y * v, 0.0))


def _round_or_none(value, ndigits=3):
    if value is None or not math.isfinite(value):
        return None
    return round(value, ndigits)


def run_lane_change(args, cfg):
    """Execute the JSON-specified lane-change cut-in scenario."""
    ego_cfg = cfg["ego"]
    cut_cfg = cfg["cut_in_vehicle"]
    term_cfg = cfg.get("termination", {})

    ego_target = float(ego_cfg["speed_kmh"])
    cutin_target = float(cut_cfg["speed_kmh"])
    gap_m = float(cut_cfg["initial_gap_m"])
    lane_start = float(cut_cfg["lane_change_start_time_s"])
    lane_duration = float(cut_cfg["lane_change_duration_s"])
    duration = float(term_cfg.get("max_duration_s", 15))
    stop_on_collision = bool(term_cfg.get("stop_on_collision", True))
    start_at_speed = bool(cfg.get("start_at_speed", True))
    side_pref = cut_cfg.get("initial_lane", "adjacent_lane")

    if lane_duration <= 0:
        raise ValueError("lane_change_duration_s must be greater than zero.")
    if cut_cfg.get("target_lane", "ego_lane") != "ego_lane":
        raise ValueError("Only target_lane='ego_lane' is supported.")
    if side_pref not in ("adjacent_lane", "left", "right"):
        raise ValueError(
            "cut_in_vehicle.initial_lane must be "
            "'adjacent_lane', 'left' or 'right'."
        )

    # Adjacent lane must exist until the cut-in finishes (+ margin).
    required_length_m = (
        gap_m + cutin_target / 3.6 * (lane_start + lane_duration) + 20.0
    )
    route_length_m = (
        gap_m + max(ego_target, cutin_target) / 3.6 * duration * 1.3 + 50.0
    )

    pygame.init()
    display = pygame.display.set_mode(
        (args.width * 2, args.height),
        pygame.HWSURFACE | pygame.DOUBLEBUF,
    )
    pygame.display.set_caption("JSON-driven CARLA Lane-change Test")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 24)

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.load_world(cfg["map"])
    original_settings = world.get_settings()
    carla_map = world.get_map()

    actors = []
    ego = None
    cutin = None
    collision_events = []
    shared_data = {"ego_surface": None, "drone_surface": None}
    simulation_started = False

    results_dir = create_results_directory()
    csv_path = results_dir / f"{cfg['scenario_id']}.csv"
    summary_path = results_dir / f"{cfg['scenario_id']}_summary.json"

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = FIXED_DELTA_S
        world.apply_settings(settings)
        simulation_started = True

        library = world.get_blueprint_library()
        ego_bp = _find_blueprint(
            library, ego_cfg.get("blueprint", "vehicle.tesla.model3"), "hero"
        )
        cutin_bp = _find_blueprint(
            library, cut_cfg.get("blueprint", "vehicle.audi.tt"), "cut_in"
        )

        candidates = choose_lane_change_candidates(
            carla_map, gap_m, required_length_m, side_pref
        )

        selected = None
        for cand in candidates:
            ego = world.try_spawn_actor(ego_bp, _lifted(cand["ego_wp"].transform))
            if ego is None:
                continue
            cutin = world.try_spawn_actor(
                cutin_bp, _lifted(cand["cutin_wp"].transform)
            )
            if cutin is not None:
                selected = cand
                break
            ego.destroy()
            ego = None

        if selected is None:
            raise RuntimeError("Could not spawn ego and cut-in vehicles.")

        actors.extend([ego, cutin])
        ego.set_autopilot(False)
        cutin.set_autopilot(False)

        side = selected["cutin_side"]
        # Right vector points right, so a left-lane source is negative.
        side_sign = -1.0 if side == "left" else 1.0
        lane_offset_m = side_sign * 0.5 * (
            selected["ego_wp"].lane_width + selected["cutin_wp"].lane_width
        )

        route = LaneRoute(selected["ego_wp"], route_length_m)
        ego_follower = RouteFollower(ego, route)
        cutin_follower = RouteFollower(cutin, route)

        ego_wp = selected["ego_wp"]
        print("\n========== LANE-CHANGE SETUP ==========")
        print(f"Road / ego lane        : {ego_wp.road_id} / {ego_wp.lane_id}")
        print(f"Cut-in lane            : {selected['cutin_wp'].lane_id} ({side} of ego)")
        print(f"Clear adjacent lane    : {selected['clear_length_m']:.0f} m "
              f"(needed {required_length_m:.0f} m)")
        print(f"Reference route length : {len(route) * route.step:.0f} m")
        print(f"Lateral offset         : {lane_offset_m:+.2f} m")
        print("=======================================\n")

        if cfg["evaluation"].get("collision_detection", True):
            col_bp = library.find("sensor.other.collision")
            col_sensor = world.spawn_actor(
                col_bp, carla.Transform(), attach_to=ego
            )
            actors.append(col_sensor)
            cutin_id = cutin.id
            col_sensor.listen(
                lambda event: collision_events.append({
                    "frame": event.frame,
                    "other_actor_id": event.other_actor.id,
                    "other_actor_type": event.other_actor.type_id,
                    "with_cut_in_vehicle": event.other_actor.id == cutin_id,
                })
            )

        cam_bp = configure_camera_blueprint(library, args.width, args.height)
        actors.append(spawn_ego_camera(world, cam_bp, ego, shared_data))
        drone_cam = spawn_drone_camera(world, cam_bp, shared_data)
        actors.append(drone_cam)

        # Let vehicles settle onto the road, then give them their speeds.
        world.tick()
        if start_at_speed:
            _set_initial_speed(ego, ego_target)
            _set_initial_speed(cutin, cutin_target)
        world.tick()

        start_sim_time = world.get_snapshot().timestamp.elapsed_seconds

        elapsed = 0.0
        alpha = 0.0
        min_center_distance = float("inf")
        min_gap_in_path = float("inf")
        min_ttc = None
        cut_in_time = None
        gap_at_cut_in = None
        lane_change_completed = False
        lane_change_completion_time = None
        running = True

        print(f"[INFO] Scenario: {cfg['scenario_id']}")
        print(f"[INFO] Lane change: t={lane_start}s for {lane_duration}s")
        print("[INFO] Close the Pygame window or press ESC to stop.")

        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = create_csv_writer(f, LANE_CHANGE_CSV_FIELDS)

            while running and elapsed < duration:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif (
                        event.type == pygame.KEYDOWN
                        and event.key == pygame.K_ESCAPE
                    ):
                        running = False
                if not running:
                    break

                # --- control -------------------------------------------
                ego_follower.control(ego_target)

                # Preview the progress at the lookahead point so the
                # lane change completes on schedule rather than lagging.
                cut_speed = max(1.0, get_speed_kmh(cutin) / 3.6)
                preview_t = elapsed + cutin_follower.lookahead_m() / cut_speed
                alpha_preview = lane_change_alpha(
                    preview_t, lane_start, lane_duration
                )
                cutin_follower.control(
                    cutin_target, lane_offset_m * (1.0 - alpha_preview)
                )

                ego_loc = ego.get_location()
                drone_cam.set_transform(carla.Transform(
                    carla.Location(x=ego_loc.x, y=ego_loc.y, z=80.0),
                    carla.Rotation(pitch=-90.0),
                ))

                world.tick()
                elapsed = (
                    world.get_snapshot().timestamp.elapsed_seconds
                    - start_sim_time
                )
                alpha = lane_change_alpha(elapsed, lane_start, lane_duration)

                # --- metrics -------------------------------------------
                cut_idx = cutin_follower.update_index()
                cut_dev = route.lateral_deviation(cut_idx, cutin.get_location())

                if (
                    not lane_change_completed
                    and elapsed > lane_start
                    and abs(cut_dev) < LANE_CHANGE_DONE_TOLERANCE_M
                ):
                    lane_change_completed = True
                    lane_change_completion_time = elapsed
                    print(f"[INFO] Cut-in vehicle centred in ego lane at {elapsed:.2f}s")

                rel = relative_kinematics(ego, cutin)
                min_center_distance = min(
                    min_center_distance, rel["center_distance_m"]
                )
                if rel["in_path"] and rel["longitudinal_offset_m"] > 0:
                    min_gap_in_path = min(
                        min_gap_in_path, rel["longitudinal_gap_m"]
                    )
                    if cut_in_time is None:
                        cut_in_time = elapsed
                        gap_at_cut_in = rel["longitudinal_gap_m"]
                        print(
                            f"[INFO] Cut-in vehicle entered ego path at "
                            f"{elapsed:.2f}s, gap {gap_at_cut_in:.2f} m"
                        )
                ttc = rel["ttc_s"]
                if ttc is not None:
                    min_ttc = ttc if min_ttc is None else min(min_ttc, ttc)

                writer.writerow({
                    "time_s": round(elapsed, 3),
                    "ego_speed_kmh": round(get_speed_kmh(ego), 2),
                    "cutin_speed_kmh": round(get_speed_kmh(cutin), 2),
                    "lane_change_progress": round(alpha, 3),
                    "cutin_lateral_offset_from_ego_lane_m": round(cut_dev, 3),
                    "longitudinal_offset_m": round(rel["longitudinal_offset_m"], 3),
                    "lateral_offset_m": round(rel["lateral_offset_m"], 3),
                    "longitudinal_gap_m": round(rel["longitudinal_gap_m"], 3),
                    "center_distance_m": round(rel["center_distance_m"], 3),
                    "cutin_in_ego_path": rel["in_path"],
                    "approx_ttc_s": round(ttc, 3) if ttc is not None else "",
                    "collision": bool(collision_events),
                })

                # --- render --------------------------------------------
                display.fill((0, 0, 0))
                if shared_data["drone_surface"] is not None:
                    display.blit(shared_data["drone_surface"], (0, 0))
                if shared_data["ego_surface"] is not None:
                    display.blit(shared_data["ego_surface"], (args.width, 0))
                status = (
                    f"t={elapsed:.1f}s  Ego={get_speed_kmh(ego):.1f} km/h  "
                    f"Cut-in={get_speed_kmh(cutin):.1f} km/h  "
                    f"LC={alpha * 100:.0f}%  "
                    f"Gap={rel['longitudinal_gap_m']:.1f}m"
                    + (f"  TTC={ttc:.1f}s" if ttc is not None else "")
                )
                display.blit(
                    font.render(status, True, (255, 255, 255)), (10, 10)
                )
                pygame.display.flip()
                clock.tick(60)

                if collision_events and stop_on_collision:
                    print("[INFO] Collision detected.")
                    break

        summary = {
            "scenario_id": cfg["scenario_id"],
            "scenario_type": cfg["scenario_type"],
            "map": cfg["map"],
            "carla_server_version": client.get_server_version(),
            "duration_simulated_s": round(elapsed, 3),
            "cut_in_side": side,
            "collision": bool(collision_events),
            "collision_with_cut_in_vehicle": any(
                e["with_cut_in_vehicle"] for e in collision_events
            ),
            "collision_events": collision_events,
            "cut_in_time_s": _round_or_none(cut_in_time),
            "longitudinal_gap_at_cut_in_m": _round_or_none(gap_at_cut_in),
            "lane_change_completed": lane_change_completed,
            "lane_change_completion_time_s": _round_or_none(
                lane_change_completion_time
            ),
            "minimum_center_distance_m": _round_or_none(min_center_distance),
            "minimum_in_path_gap_m": _round_or_none(min_gap_in_path),
            "minimum_approx_ttc_s": _round_or_none(min_ttc),
            "csv_file": str(csv_path),
        }
        save_summary(summary_path, summary)

        print(json.dumps(summary, indent=2))
        print(f"[INFO] Saved: {csv_path}")
        print(f"[INFO] Saved: {summary_path}")

    finally:
        for actor in actors:
            if actor is not None and actor.is_alive and actor.type_id.startswith("sensor."):
                try:
                    actor.stop()
                except RuntimeError:
                    pass
        for actor in reversed(actors):
            if actor is not None and actor.is_alive:
                try:
                    actor.destroy()
                except RuntimeError:
                    pass
        if ego is not None and ego not in actors and ego.is_alive:
            ego.destroy()
        if simulation_started:
            world.apply_settings(original_settings)
        pygame.quit()
