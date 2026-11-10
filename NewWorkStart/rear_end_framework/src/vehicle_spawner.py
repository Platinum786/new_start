
import carla


def choose_spawn_pair(carla_map, gap_m):
    """Find candidate spawn pairs on the same driving lane."""
    candidates = []

    for spawn in carla_map.get_spawn_points():
        waypoint = carla_map.get_waypoint(
            spawn.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )

        if waypoint is None:
            continue

        for ahead in waypoint.next(gap_m):
            if (
                ahead.road_id == waypoint.road_id
                and ahead.lane_id == waypoint.lane_id
            ):
                candidates.append(
                    (waypoint.transform, ahead.transform)
                )

    if not candidates:
        raise RuntimeError(
            f"No same-lane spawn pairs found for gap={gap_m} m."
        )

    print(f"[INFO] Found {len(candidates)} candidate spawn pairs.")
    return candidates


def spawn_vehicle_pair(
    world,
    ego_blueprint,
    lead_blueprint,
    candidate_pairs,
):
    """Try candidate pairs until both vehicles spawn successfully."""
    for index, (ego_transform, lead_transform) in enumerate(
        candidate_pairs, start=1
    ):
        ego_vehicle = None
        lead_vehicle = None

        print(
            f"[INFO] Trying spawn pair "
            f"{index}/{len(candidate_pairs)}"
        )

        try:
            ego_vehicle = world.try_spawn_actor(
                ego_blueprint, ego_transform
            )

            if ego_vehicle is None:
                print("[WARN] Ego spawn failed; trying next pair.")
                continue

            lead_vehicle = world.try_spawn_actor(
                lead_blueprint, lead_transform
            )

            if lead_vehicle is None:
                print("[WARN] Lead spawn failed; trying next pair.")
                ego_vehicle.destroy()
                continue

            print("[SUCCESS] Both vehicles spawned.")
            return ego_vehicle, lead_vehicle

        except Exception:
            if lead_vehicle is not None:
                lead_vehicle.destroy()
            if ego_vehicle is not None:
                ego_vehicle.destroy()
            raise

    raise RuntimeError(
        "Could not spawn both vehicles. Check the map and spawn points."
    )
