
def spawn_vehicle_pair(world, ego_blueprint, lead_blueprint, candidate_pairs):
    """
    Try candidate spawn-point pairs until both vehicles spawn successfully.

    candidate_pairs should contain (ego_transform, lead_transform) tuples.
    Returns (ego_vehicle, lead_vehicle).
    """
    for index, (ego_transform, lead_transform) in enumerate(
        candidate_pairs, start=1
    ):
        ego_vehicle = None
        lead_vehicle = None

        try:
            ego_vehicle = world.try_spawn_actor(
                ego_blueprint, ego_transform
            )

            if ego_vehicle is None:
                print(
                    f"Spawn pair {index}: ego spawn failed; "
                    "trying next pair."
                )
                continue

            lead_vehicle = world.try_spawn_actor(
                lead_blueprint, lead_transform
            )

            if lead_vehicle is None:
                print(
                    f"Spawn pair {index}: lead spawn failed; "
                    "trying next pair."
                )
                ego_vehicle.destroy()
                continue

            print(f"Vehicles spawned successfully using pair {index}.")
            return ego_vehicle, lead_vehicle

        except Exception:
            if lead_vehicle is not None:
                lead_vehicle.destroy()
            if ego_vehicle is not None:
                ego_vehicle.destroy()
            raise

    raise RuntimeError(
        "Could not spawn both vehicles using any candidate pair. "
        "Try another map or spawn configuration."
    )
