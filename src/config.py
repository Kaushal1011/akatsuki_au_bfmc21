import json
from pprint import pprint

# TODO: add argparser for config
global config_path
config_path = ""


def get_config():
    global config_path
    assert config_path, "Please provide a config_path"
    print(f"Configured using config file : {config_path}")
    with open(config_path) as f:
        config = json.load(f)
    # pprint(config)
    return config
