
import argparse
import csv
import json
import math
import random
import time
from pathlib import Path

import carla
import numpy as np
import pygame


def get_speed_kmh(actor):
    v = actor.get_velocity()
    return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)


def get_forward_speed_kmh(actor):
    v = actor.get_velocity()
    f = actor.get_transform().get_forward_vector()
    return 3.6 * (v.x * f.x + v.y * f.y + v.z * f.z)


# def choose_spawn_pair(carla_map, gap_m):
#     """Find two driving waypoints on the same road and lane."""
#     for spawn in carla_map.get_spawn_points():
#         wp = carla_map.get_waypoint(
#             spawn.location,
#             project_to_road=True,
#             lane_type=carla.LaneType.Driving
#         )
#         if wp is None:
#             continue

#         for ahead in wp.next(gap_m):
#             if ahead.road_id == wp.road_id and ahead.lane_id == wp.lane_id:
#                 return wp.transform, ahead.transform

#     raise RuntimeError(
#         "No same-lane spawn pair found. Try a different map or gap."
#     )




def choose_spawn_pair(carla_map, gap_m):
    """Find several candidate pairs of positions on the same driving lane."""
    candidates = []

    for spawn in carla_map.get_spawn_points():
        wp = carla_map.get_waypoint(
            spawn.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        if wp is None:
            continue

        for ahead in wp.next(gap_m):
            if ahead.road_id == wp.road_id and ahead.lane_id == wp.lane_id:
                candidates.append((wp.transform, ahead.transform))

    if not candidates:
        raise RuntimeError(
            f"No same-lane spawn pairs found for gap={gap_m} m."
        )

    print(f"[INFO] Found {len(candidates)} candidate spawn pairs.")
    return candidates



def run(args):
    with open(args.scenario, encoding="utf-8") as f:
        cfg = json.load(f)

    if cfg.get("scenario_type") != "rear_end_collision":
        raise ValueError("This runner supports rear_end_collision only.")

    pygame.init()
    display = pygame.display.set_mode(
        (args.width * 2, args.height),
        pygame.HWSURFACE | pygame.DOUBLEBUF
    )
    pygame.display.set_caption("JSON-driven CARLA Rear-end Test")
    clock = pygame.time.Clock()

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)

    world = client.load_world(cfg["map"])
    original_settings = world.get_settings()

    actors = []
    ego = None
    lead = None
    ego_cam = None
    drone_cam = None
    collision_events = []
    shared_data = {"ego_surface": None, "drone_surface": None}
    simulation_started = False

    results_dir = Path("results")
    results_dir.mkdir(exist_ok=True)
    csv_path = results_dir / f"{cfg['scenario_id']}.csv"
    summary_path = results_dir / f"{cfg['scenario_id']}_summary.json"

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        simulation_started = True

        blueprint_library = world.get_blueprint_library()
        vehicle_bps = blueprint_library.filter("vehicle.*")
        if not vehicle_bps:
            raise RuntimeError("No vehicle blueprints found.")

        # Prefer a familiar vehicle model where available.
        tesla = blueprint_library.filter("vehicle.tesla.model3")
        ego_bp = tesla[0] if tesla else random.choice(vehicle_bps)
        lead_bp = random.choice(vehicle_bps)

        gap_m = float(cfg["lead_vehicle"]["initial_gap_m"])
        # ego_transform, lead_transform = choose_spawn_pair(
        #     world.get_map(), gap_m
        # )

        # ego = world.try_spawn_actor(ego_bp, ego_transform)
        # if ego is None:
        #     raise RuntimeError("Ego spawn failed; try another map/spawn pair.")
        # actors.append(ego)

        # lead = world.try_spawn_actor(lead_bp, lead_transform)
        # if lead is None:
        #     raise RuntimeError("Lead vehicle spawn failed.")
        # actors.append(lead)

        

        candidates = choose_spawn_pair(world.get_map(), gap_m)

        for attempt, (ego_transform, lead_transform) in enumerate(candidates, start=1):
            print(f"[INFO] Trying spawn pair {attempt}/{len(candidates)}")

            ego = world.try_spawn_actor(ego_bp, ego_transform)

            if ego is None:
                print("[WARN] Ego spawn failed at this position.")
                continue

            lead = world.try_spawn_actor(lead_bp, lead_transform)

            if lead is None:
                print("[WARN] Lead spawn failed; trying another pair.")
                ego.destroy()
                ego = None
                continue

            # Both vehicles spawned successfully.
            actors.extend([ego, lead])
            print("[SUCCESS] Ego and lead vehicles spawned.")
            break

        if ego is None or lead is None:
            raise RuntimeError(
                "Could not spawn both vehicles. "
                "Check whether spawn points are occupied or the map is available."
            )


        ego.set_autopilot(False)
        lead.set_autopilot(False)

        # Collision sensor attached to the ego vehicle.
        if cfg["evaluation"].get("collision_detection", True):
            collision_bp = blueprint_library.find("sensor.other.collision")
            collision_sensor = world.spawn_actor(
                collision_bp, carla.Transform(), attach_to=ego
            )
            actors.append(collision_sensor)
            collision_sensor.listen(
                lambda event: collision_events.append({
                    "frame": event.frame,
                    "other_actor_id": event.other_actor.id
                })
            )

        # Camera image callbacks.
        cam_bp = blueprint_library.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(args.width))
        cam_bp.set_attribute("image_size_y", str(args.height))
        cam_bp.set_attribute("fov", "90")

        def make_image_callback(key):
            def callback(image):
                image.convert(carla.ColorConverter.Raw)
                array = np.frombuffer(
                    image.raw_data, dtype=np.uint8
                ).reshape((image.height, image.width, 4))
                rgb = array[:, :, :3][:, :, ::-1]
                surface = pygame.surfarray.make_surface(
                    rgb.swapaxes(0, 1)
                )
                shared_data[key] = surface
            return callback

        ego_cam = world.spawn_actor(
            cam_bp,
            carla.Transform(
                carla.Location(x=-6.0, z=2.5),
                carla.Rotation(pitch=-15.0)
            ),
            attach_to=ego
        )
        actors.append(ego_cam)
        ego_cam.listen(make_image_callback("ego_surface"))

        drone_cam = world.spawn_actor(
            cam_bp,
            carla.Transform(
                carla.Location(z=80.0),
                carla.Rotation(pitch=-90.0)
            )
        )
        actors.append(drone_cam)
        drone_cam.listen(make_image_callback("drone_surface"))

        world.tick()
        start_sim_time = world.get_snapshot().timestamp.elapsed_seconds
        duration = float(cfg["termination"]["max_duration_s"])
        brake_time = float(cfg["lead_vehicle"]["brake_time_s"])
        ego_target = float(cfg["ego"]["speed_kmh"])
        lead_target = float(cfg["lead_vehicle"]["speed_kmh"])

        min_center_distance = float("inf")
        min_ttc = None
        elapsed = 0.0
        rows = []
        running = True

        print(f"Scenario: {cfg['scenario_id']}")
        print(f"Map: {world.get_map().name}")
        print("Close the Pygame window to stop the test.")

        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=[
                "time_s", "ego_speed_kmh", "lead_speed_kmh",
                "center_distance_m", "approx_ttc_s", "collision"
            ])
            writer.writeheader()

            while running and elapsed < duration:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            running = False

                if not running:
                    break

                # Ego: simple target-speed controller, not a full ADS.
                ego_speed = get_speed_kmh(ego)
                error = ego_target - ego_speed
                if error >= 0:
                    ego_control = carla.VehicleControl(
                        throttle=min(0.75, error * 0.06),
                        brake=0.0
                    )
                else:
                    ego_control = carla.VehicleControl(
                        throttle=0.0,
                        brake=min(0.6, -error * 0.04)
                    )
                ego.apply_control(ego_control)

                # Lead: follow target speed, then brake at the JSON time.
                if elapsed >= brake_time:
                    lead.apply_control(carla.VehicleControl(
                        throttle=0.0, brake=1.0
                    ))
                else:
                    lead_error = lead_target - get_speed_kmh(lead)
                    lead.apply_control(carla.VehicleControl(
                        throttle=min(0.7, max(0.0, lead_error * 0.06)),
                        brake=0.0
                    ))

                # Keep drone camera centered above ego.
                ego_loc = ego.get_location()
                drone_cam.set_transform(carla.Transform(
                    carla.Location(
                        x=ego_loc.x, y=ego_loc.y, z=80.0
                    ),
                    carla.Rotation(pitch=-90.0)
                ))

                world.tick()
                sim_time = (
                    world.get_snapshot().timestamp.elapsed_seconds
                )
                elapsed = sim_time - start_sim_time

                ego_loc = ego.get_location()
                lead_loc = lead.get_location()
                center_distance = ego_loc.distance(lead_loc)
                min_center_distance = min(
                    min_center_distance, center_distance
                )

                closing_speed = (
                    get_forward_speed_kmh(ego)
                    - get_forward_speed_kmh(lead)
                ) / 3.6
                ttc = None
                if closing_speed > 0.1:
                    ttc = center_distance / closing_speed
                    min_ttc = ttc if min_ttc is None else min(min_ttc, ttc)

                row = {
                    "time_s": round(elapsed, 3),
                    "ego_speed_kmh": round(get_speed_kmh(ego), 2),
                    "lead_speed_kmh": round(get_speed_kmh(lead), 2),
                    "center_distance_m": round(center_distance, 3),
                    "approx_ttc_s": (
                        round(ttc, 3) if ttc is not None else ""
                    ),
                    "collision": bool(collision_events)
                }
                writer.writerow(row)
                rows.append(row)

                # Render the same two views as your original code.
                display.fill((0, 0, 0))
                if shared_data["drone_surface"] is not None:
                    display.blit(shared_data["drone_surface"], (0, 0))
                if shared_data["ego_surface"] is not None:
                    display.blit(
                        shared_data["ego_surface"], (args.width, 0)
                    )

                font = pygame.font.Font(None, 24)
                status = (
                    f"t={elapsed:.1f}s  "
                    f"Ego={get_speed_kmh(ego):.1f} km/h  "
                    f"Lead={get_speed_kmh(lead):.1f} km/h  "
                    f"Gap(center)={center_distance:.1f}m"
                )
                display.blit(
                    font.render(status, True, (255, 255, 255)),
                    (10, 10)
                )
                pygame.display.flip()
                clock.tick(60)

                # Stop after a detected collision; still save results.
                if collision_events:
                    print("Collision detected.")
                    break

        summary = {
            "scenario_id": cfg["scenario_id"],
            "map": cfg["map"],
            "carla_server_version": client.get_server_version(),
            "duration_simulated_s": round(elapsed, 3),
            "collision": bool(collision_events),
            "collision_events": collision_events,
            "minimum_center_distance_m": (
                round(min_center_distance, 3)
                if math.isfinite(min_center_distance) else None
            ),
            "minimum_approx_ttc_s": (
                round(min_ttc, 3) if min_ttc is not None else None
            ),
            "csv_file": str(csv_path)
        }
        summary_path.write_text(
            json.dumps(summary, indent=2), encoding="utf-8"
        )
        print(json.dumps(summary, indent=2))
        print(f"Saved: {csv_path}")
        print(f"Saved: {summary_path}")

    finally:
        # Stop sensor callbacks before destroying actors.
        for actor in actors:
            if actor is not None and actor.is_alive:
                if actor.type_id.startswith("sensor."):
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

        if simulation_started:
            world.apply_settings(original_settings)

        pygame.quit()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument(
        "--scenario", default="scenarios/rear_end_001.json"
    )
    parser.add_argument("--res", default="640x480")
    args = parser.parse_args()
    args.width, args.height = map(int, args.res.split("x"))
    run(args)


if __name__ == "__main__":
    main()
