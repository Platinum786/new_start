
import carla

from safety_metrics import get_speed_kmh


def control_ego_vehicle(ego, target_speed_kmh):
    """Maintain the ego vehicle's target speed."""
    ego_speed = get_speed_kmh(ego)
    error = target_speed_kmh - ego_speed

    if error >= 0:
        control = carla.VehicleControl(
            throttle=min(0.75, error * 0.06),
            brake=0.0,
        )
    else:
        control = carla.VehicleControl(
            throttle=0.0,
            brake=min(0.6, -error * 0.04),
        )

    ego.apply_control(control)


def control_lead_vehicle(
    lead,
    target_speed_kmh,
    elapsed,
    brake_time_s,
):
    """Maintain lead speed until the configured braking time."""
    if elapsed >= brake_time_s:
        lead.apply_control(
            carla.VehicleControl(
                throttle=0.0,
                brake=1.0,
            )
        )
    else:
        lead_error = target_speed_kmh - get_speed_kmh(lead)

        lead.apply_control(
            carla.VehicleControl(
                throttle=min(
                    0.7,
                    max(0.0, lead_error * 0.06),
                ),
                brake=0.0,
            )
        )
