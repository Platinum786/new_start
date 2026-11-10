
import math


def get_speed_kmh(actor):
    """Return the total speed of a CARLA actor in km/h."""
    velocity = actor.get_velocity()

    speed_mps = math.sqrt(
        velocity.x ** 2
        + velocity.y ** 2
        + velocity.z ** 2
    )

    return 3.6 * speed_mps


def get_forward_speed_kmh(actor):
    """Return the actor's forward-direction speed in km/h."""
    velocity = actor.get_velocity()
    forward = actor.get_transform().get_forward_vector()

    forward_speed_mps = (
        velocity.x * forward.x
        + velocity.y * forward.y
        + velocity.z * forward.z
    )

    return 3.6 * forward_speed_mps


def calculate_ttc(distance_m, closing_speed_mps):
    """
    Estimate time to collision in seconds.

    Returns None when the vehicles are not closing in
    or when the closing speed is too small.
    """
    if distance_m < 0 or closing_speed_mps <= 0.1:
        return None

    return distance_m / closing_speed_mps


def calculate_bumper_gap(ego, lead):
    """
    Estimate the bumper-to-bumper gap for vehicles
    aligned in the same direction and lane.

    Returns the gap in meters.
    """
    ego_location = ego.get_location()
    lead_location = lead.get_location()

    center_distance = ego_location.distance(lead_location)

    ego_half_length = ego.bounding_box.extent.x
    lead_half_length = lead.bounding_box.extent.x

    bumper_gap = (
        center_distance
        - ego_half_length
        - lead_half_length
    )

    return bumper_gap