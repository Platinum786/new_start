
import numpy as np
import pygame
import carla


def configure_camera_blueprint(blueprint_library, width, height):
    """Configure the RGB camera blueprint."""
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(width))
    camera_bp.set_attribute("image_size_y", str(height))
    camera_bp.set_attribute("fov", "90")
    return camera_bp


def make_image_callback(shared_data, key):
    """Convert CARLA camera images into Pygame surfaces."""

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


def spawn_ego_camera(world, camera_bp, ego, shared_data):
    """Spawn the ego-attached camera using the original transform."""
    camera = world.spawn_actor(
        camera_bp,
        carla.Transform(
            carla.Location(x=-6.0, z=2.5),
            carla.Rotation(pitch=-15.0),
        ),
        attach_to=ego,
    )

    camera.listen(make_image_callback(shared_data, "ego_surface"))
    return camera


def spawn_drone_camera(world, camera_bp, shared_data):
    """Spawn the overhead drone camera using the original transform."""
    camera = world.spawn_actor(
        camera_bp,
        carla.Transform(
            carla.Location(z=80.0),
            carla.Rotation(pitch=-90.0),
        ),
    )

    camera.listen(make_image_callback(shared_data, "drone_surface"))
    return camera
