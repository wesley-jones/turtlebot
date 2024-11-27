import json
import pkg_resources

def get_port_from_config(key, config_file="zeromq_ports.json"):
    """
    Retrieve the port number for a given key from a JSON configuration file.

    :param key: The sensor key to look up (e.g., "cliff_sensor").
    :param config_file: The path to the configuration file (default: "sensor_ports.json").
    :return: The port number associated with the key.
    :raises ValueError: If the key is not found in the configuration.
    """
    try:
        # Locate the JSON file within the package
        config_path = pkg_resources.resource_filename(__name__, config_file)
        with open(config_path, "r") as file:
            config = json.load(file)
        port = config.get(key)
        if port is None:
            raise ValueError(f"Key '{key}' not found in configuration file.")
        return port
    except FileNotFoundError:
        raise FileNotFoundError(f"Configuration file '{config_file}' not found.")
    except json.JSONDecodeError:
        raise ValueError(f"Configuration file '{config_file}' is not valid JSON.")
