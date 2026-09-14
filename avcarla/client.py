import carla
import numpy as np

from .config import CARLA


@CARLA.register_module()
class CarlaClient:
    def __init__(
        self,
        connect_ip: str,
        connect_port: int,
        traffic_manager_port: int,
        traffic_manager_seed: int,
        synchronous: bool,
        rate: float,
        disable_static_actors: bool = True,
        randomize_lights: bool = True,
        prob_light_green: float = 0.35,
        seed: int = None,
        rng: np.random.RandomState = None,
        map_name: str = None,
        reset_world: bool = False,
        weather: dict = None,
        traffic_lights: dict = None,
        strict_spawn: bool = False,
        substepping: bool = True,
        max_substep_delta_time: float = 0.01,
        max_substeps: int = 10,
    ) -> None:
        if reset_world and not synchronous:
            raise ValueError("Repeatable world resets require synchronous mode")
        self.rng = rng if rng is not None else np.random.RandomState(seed)
        self._prob_light_green = prob_light_green
        self.strict_spawn = strict_spawn
        self.traffic_manager_port = traffic_manager_port
        self.client = carla.Client(connect_ip, connect_port)
        self.client.set_timeout(30.0 if reset_world else 4.0)
        self.world = self.client.get_world()
        self._orig_settings = self.world.get_settings()
        if (
            map_name
            and self.world.get_map().name.split("/")[-1] != map_name.split("/")[-1]
        ):
            self.world = self.client.load_world(map_name)
        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous
        settings.fixed_delta_seconds = 1.0 / rate
        settings.substepping = substepping
        settings.max_substep_delta_time = max_substep_delta_time
        settings.max_substeps = max_substeps
        self.world.apply_settings(settings)
        if reset_world:
            # Reload only after enabling synchrony, retaining the fixed-step settings.
            self.world = self.client.reload_world(False)
        self.traffic_manager = self.client.get_trafficmanager(traffic_manager_port)
        self.traffic_manager.set_synchronous_mode(synchronous)
        if (traffic_manager_seed is None) and (seed is not None):
            traffic_manager_seed = seed
        if traffic_manager_seed is not None:
            print(f"Setting traffic manager seed as {traffic_manager_seed}")
            self.traffic_manager.set_random_device_seed(traffic_manager_seed)
        self.map = self.world.get_map()
        self.spawn_points = self.map.get_spawn_points()
        self.spawns_chosen = []
        if weather is not None:
            parameters = self.world.get_weather()
            for name, value in weather.items():
                if not hasattr(parameters, name):
                    raise ValueError(f"Unknown weather parameter: {name}")
                setattr(parameters, name, value)
            self.world.set_weather(parameters)
        if traffic_lights is not None:
            for light in self._traffic_lights():
                values = traffic_lights[light.get_opendrive_id()]
                light.set_green_time(values["green_time"])
                light.set_yellow_time(values["yellow_time"])
                light.set_red_time(values["red_time"])
                light.set_state(getattr(carla.TrafficLightState, values["state"]))
                light.freeze(values["frozen"])
        elif randomize_lights:
            self.set_traffic_lights()
        if disable_static_actors:
            self.world.unload_map_layer(carla.MapLayer.ParkedVehicles)

    def set_traffic_lights(self):
        list_actor = self._traffic_lights()
        for actor_ in list_actor:
            if isinstance(actor_, carla.TrafficLight):
                # for any light, first set the light state, then set time. for yellow it is
                # carla.TrafficLightState.Yellow and Red it is carla.TrafficLightState.Red
                if self.rng.rand() < self._prob_light_green:
                    actor_.set_state(carla.TrafficLightState.Green)
                else:
                    actor_.set_state(carla.TrafficLightState.Red)
                actor_.set_green_time(5.0)
                actor_.set_yellow_time(1.0)
                actor_.set_red_time(4.0)

    def _traffic_lights(self):
        return sorted(
            self.world.get_actors().filter("traffic.traffic_light"),
            key=lambda light: light.get_opendrive_id(),
        )

    def scene_settings(self):
        """Capture concrete external settings to reuse after a world reset."""
        weather = self.world.get_weather()
        return {
            "map_name": self.map.name,
            "weather": {
                name: getattr(weather, name)
                for name in dir(weather)
                if not name.startswith("_")
                and isinstance(getattr(weather, name), (int, float))
            },
            "traffic_lights": {
                light.get_opendrive_id(): {
                    "state": str(light.get_state()).split(".")[-1],
                    "green_time": light.get_green_time(),
                    "yellow_time": light.get_yellow_time(),
                    "red_time": light.get_red_time(),
                    "frozen": light.is_frozen(),
                }
                for light in self._traffic_lights()
            },
        }

    def close(self):
        self.traffic_manager.set_synchronous_mode(False)
        self.world.apply_settings(self._orig_settings)

    def client_npcs(self):
        raise

    def tick(self):
        return self.world.tick()

    def on_tick(self, func):
        self.world.on_tick(func)
