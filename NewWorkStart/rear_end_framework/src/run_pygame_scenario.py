
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

from safety_metrics import (
    get_speed_kmh,
    get_forward_speed_kmh,
    calculate_ttc,
    calculate_bumper_gap,
)

from scenario_config import load_scenario_config

from vehicle_spawner import (
    choose_spawn_pair,
    spawn_vehicle_pair,
)

from camera_manager import (
    configure_camera_blueprint,
    spawn_ego_camera,
    spawn_drone_camera,
)

from vehicle_control import (
    control_ego_vehicle,
    control_lead_vehicle,
)

from lane_change_scenario import run_lane_change

from result_logger import (
    create_results_directory,
    create_csv_writer,
    save_summary,
)


def run_rear_end(args, cfg):

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

    results_dir = create_results_directory()
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

        candidates = choose_spawn_pair(world.get_map(), gap_m)

        ego, lead = spawn_vehicle_pair(
            world,
            ego_bp,
            lead_bp,
            candidates,
        )

        actors.extend([ego, lead])

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

        # Configure the RGB camera blueprint.
        cam_bp = configure_camera_blueprint(
            blueprint_library,
            args.width,
            args.height,
        )

        # Spawn the ego-attached camera.
        ego_cam = spawn_ego_camera(
            world,
            cam_bp,
            ego,
            shared_data,
        )
        actors.append(ego_cam)

        # Spawn the overhead drone camera.
        drone_cam = spawn_drone_camera(
            world,
            cam_bp,
            shared_data,
        )
        actors.append(drone_cam)

        world.tick()
        start_sim_time = world.get_snapshot().timestamp.elapsed_seconds
        duration = float(cfg["termination"]["max_duration_s"])
        brake_time = float(cfg["lead_vehicle"]["brake_time_s"])
        ego_target = float(cfg["ego"]["speed_kmh"])
        lead_target = float(cfg["lead_vehicle"]["speed_kmh"])

        min_center_distance = float("inf")
        min_bumper_gap = float("inf")
        min_ttc = None
        elapsed = 0.0
        rows = []
        running = True

        print(f"Scenario: {cfg['scenario_id']}")
        print(f"Map: {world.get_map().name}")
        print("Close the Pygame window to stop the test.")

        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = create_csv_writer(f)

            while running and elapsed < duration:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            running = False

                if not running:
                    break

                # Control ego vehicle.
                control_ego_vehicle(ego, ego_target)

                # Control lead vehicle and apply braking at the configured time.
                control_lead_vehicle(
                    lead,
                    lead_target,
                    elapsed,
                    brake_time,
                )


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
                bumper_gap = calculate_bumper_gap(ego, lead)
                min_bumper_gap = min(
                    min_bumper_gap,
                    bumper_gap,
                )
                min_center_distance = min(
                    min_center_distance, center_distance
                )

                
                closing_speed = (
                    get_forward_speed_kmh(ego)
                    - get_forward_speed_kmh(lead)
                ) / 3.6

                ttc = calculate_ttc(
                    max(0.0, bumper_gap),
                    closing_speed,
                )

                if ttc is not None:
                    min_ttc = ttc if min_ttc is None else min(min_ttc, ttc)


                row = {
                    "time_s": round(elapsed, 3),
                    "ego_speed_kmh": round(get_speed_kmh(ego), 2),
                    "lead_speed_kmh": round(get_speed_kmh(lead), 2),
                    "center_distance_m": round(center_distance, 3),
                    "bumper_gap_m": round(bumper_gap, 3),
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
            "minimum_bumper_gap_m": (
                round(min_bumper_gap, 3)
                if math.isfinite(min_bumper_gap)
                else None
            ),
            "minimum_approx_ttc_s": (
                round(min_ttc, 3) if min_ttc is not None else None
            ),
            "csv_file": str(csv_path)
        }
        save_summary(summary_path, summary)

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



def run(args):
    if args.seed is not None:
        random.seed(args.seed)

    cfg = load_scenario_config(args.scenario)
    scenario_type = cfg["scenario_type"]

    print(f"[INFO] Dispatching scenario: {scenario_type}")

    if scenario_type == "rear_end_collision":
        run_rear_end(args, cfg)

    elif scenario_type == "lane_change_cut_in":
        run_lane_change(args, cfg)

    else:
        raise ValueError(
            f"No execution handler for scenario: {scenario_type}"
        )



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument(
        "--scenario", default="scenarios/rear_end_001.json"
    )
    parser.add_argument("--res", default="640x480")
    parser.add_argument(
        "--seed", type=int, default=None,
        help="Random seed for reproducible spawn-point selection",
    )
    args = parser.parse_args()
    args.width, args.height = map(int, args.res.split("x"))
    run(args)


if __name__ == "__main__":
    main()
