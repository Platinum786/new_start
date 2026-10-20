
import csv
import json
import math
import sys
from pathlib import Path

import carla

from validate_scenario import SCHEMA
from jsonschema import validate


HOST = "127.0.0.1"
PORT = 2000
DT = 0.05


def speed_kmh(actor):
    v = actor.get_velocity()
    return 3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def forward_speed_kmh(actor):
    v = actor.get_velocity()
    f = actor.get_transform().get_forward_vector()
    return 3.6 * (v.x * f.x + v.y * f.y + v.z * f.z)


def main(json_path):
    with open(json_path, encoding="utf-8") as f:
        cfg = json.load(f)
    validate(instance=cfg, schema=SCHEMA)

    out_dir = Path("results")
    out_dir.mkdir(exist_ok=True)
    csv_path = out_dir / f"{cfg['scenario_id']}.csv"
    summary_path = out_dir / f"{cfg['scenario_id']}_summary.json"

    client = carla.Client(HOST, PORT)
    client.set_timeout(20.0)

    expected = "0.9.15"
    server_version = client.get_server_version()
    if not server_version.startswith(expected):
        raise RuntimeError(
            f"Expected CARLA {expected}, got {server_version}"
        )

    world = client.load_world(cfg["map"])
    original_settings = world.get_settings()
    actors = []
    collision_events = []

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = DT
        world.apply_settings(settings)

        weather = carla.WeatherParameters.ClearNoon
        if cfg["environment"]["weather"] == "rain":
            weather = carla.WeatherParameters(
                cloudiness=80.0, precipitation=60.0,
                precipitation_deposits=50.0, wetness=70.0
            )
        elif cfg["environment"]["weather"] == "fog":
            weather = carla.WeatherParameters(
                cloudiness=20.0, fog_density=50.0, fog_distance=20.0
            )
        world.set_weather(weather)

        carla_map = world.get_map()
        blueprints = world.get_blueprint_library()
        vehicle_bp = blueprints.filter("vehicle.*model3*")
        if not vehicle_bp:
            vehicle_bp = blueprints.filter("vehicle.*")
        if not vehicle_bp:
            raise RuntimeError("No vehicle blueprints found.")

        ego_bp = vehicle_bp[0]
        lead_bp = vehicle_bp[0]

        # Find a valid pair on the same road and driving lane.
        gap = cfg["lead_vehicle"]["initial_gap_m"]
        pair = None
        for spawn in carla_map.get_spawn_points():
            wp = carla_map.get_waypoint(
                spawn.location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )
            if wp is None:
                continue

            next_wps = wp.next(gap)
            for lead_wp in next_wps:
                if (lead_wp.road_id == wp.road_id
                        and lead_wp.lane_id == wp.lane_id):
                    pair = (wp, lead_wp)
                    break
            if pair:
                break

        if pair is None:
            raise RuntimeError(
                "Could not find a same-lane spawn pair. "
                "Try another map or adjust the initial gap."
            )

        ego_wp, lead_wp = pair
        ego_transform = ego_wp.transform
        lead_transform = lead_wp.transform

        ego = world.try_spawn_actor(ego_bp, ego_transform)
        if ego is None:
            raise RuntimeError("Could not spawn ego vehicle.")
        actors.append(ego)

        lead = world.try_spawn_actor(lead_bp, lead_transform)
        if lead is None:
            raise RuntimeError(
                "Could not spawn lead vehicle; try another spawn pair."
            )
        actors.append(lead)

        ego.set_autopilot(False)
        lead.set_autopilot(False)

        # Collision sensor attached to ego.
        if cfg["evaluation"]["collision_detection"]:
            sensor_bp = blueprints.find("sensor.other.collision")
            sensor = world.spawn_actor(
                sensor_bp, carla.Transform(), attach_to=ego
            )
            actors.append(sensor)
            sensor.listen(lambda event: collision_events.append({
                "frame": event.frame,
                "other_actor_id": event.other_actor.id
            }))

        world.tick()

        ego_target = cfg["ego"]["speed_kmh"]
        lead_target = cfg["lead_vehicle"]["speed_kmh"]
        brake_time = cfg["lead_vehicle"]["brake_time_s"]
        duration = cfg["termination"]["max_duration_s"]

        min_center_distance = float("inf")
        min_ttc = None
        rows = []
        elapsed = 0.0

        print(f"Running {cfg['scenario_id']} on {cfg['map']}")

        with csv_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=[
                "time_s", "ego_speed_kmh", "lead_speed_kmh",
                "center_distance_m", "approx_ttc_s", "collision"
            ])
            writer.writeheader()

            while elapsed < duration and not collision_events:
                # Simple ego speed controller; not a production ADS.
                ego_speed = speed_kmh(ego)
                ego_error = ego_target - ego_speed
                if ego_error >= 0:
                    ego_control = carla.VehicleControl(
                        throttle=min(0.75, max(0.0, ego_error * 0.06)),
                        brake=0.0
                    )
                else:
                    ego_control = carla.VehicleControl(
                        throttle=0.0,
                        brake=min(0.6, -ego_error * 0.04)
                    )
                ego.apply_control(ego_control)

                # Lead follows its target speed, then brakes.
                lead_speed = speed_kmh(lead)
                if elapsed >= brake_time:
                    lead.apply_control(carla.VehicleControl(
                        throttle=0.0, brake=1.0
                    ))
                else:
                    lead_error = lead_target - lead_speed
                    lead.apply_control(carla.VehicleControl(
                        throttle=min(0.7, max(0.0, lead_error * 0.06)),
                        brake=0.0
                    ))

                world.tick()
                elapsed += DT

                ego_loc = ego.get_location()
                lead_loc = lead.get_location()
                center_distance = ego_loc.distance(lead_loc)
                min_center_distance = min(
                    min_center_distance, center_distance
                )

                closing_speed = (
                    forward_speed_kmh(ego) - forward_speed_kmh(lead)
                ) / 3.6
                ttc = None
                if closing_speed > 0.1:
                    ttc = center_distance / closing_speed
                    if min_ttc is None or ttc < min_ttc:
                        min_ttc = ttc

                row = {
                    "time_s": round(elapsed, 3),
                    "ego_speed_kmh": round(speed_kmh(ego), 2),
                    "lead_speed_kmh": round(speed_kmh(lead), 2),
                    "center_distance_m": round(center_distance, 3),
                    "approx_ttc_s": (
                        round(ttc, 3) if ttc is not None else ""
                    ),
                    "collision": bool(collision_events)
                }
                writer.writerow(row)
                rows.append(row)

        summary = {
            "scenario_id": cfg["scenario_id"],
            "map": cfg["map"],
            "carla_server_version": server_version,
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

    finally:
        for actor in reversed(actors):
            if actor is not None and actor.is_alive:
                actor.destroy()
        world.apply_settings(original_settings)


if __name__ == "__main__":
    path = (
        sys.argv[1] if len(sys.argv) > 1
        else "scenarios/rear_end_001.json"
    )
    main(path)
