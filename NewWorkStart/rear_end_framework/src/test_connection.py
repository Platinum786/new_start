
import carla

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(10.0)

print("Server version:", client.get_server_version())
print("Client version:", client.get_client_version())

world = client.get_world()
print("Current map:", world.get_map().name)
print("Connection successful.")
