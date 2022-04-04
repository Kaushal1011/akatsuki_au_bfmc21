import json
from pprint import pprint

# TODO: add argparser for config
global config_path
config_path = ""


def get_config():
    global config_path
    assert config_path, "Please provide a config_path"
    print(f"Configured using config file : {config_path}")
    try:
        # TODO: parse config path as an arg
        with open(config_path) as f:
            config = json.load(f)

    except FileNotFoundError as e:
        print(e)
        print("Switching to default config, see `Brain/src/config.py`")

        config = {
            "enableStream": False,
            "enableCameraSpoof": False,
            "enableRc": False,
            "enableLaneKeeping": True,
            "enableSIM": True,
            "enableIntersectionDet": True,
            "using_server": False,
            "start_idx": "86",
            "end_idx": "27",
            "home_loc": False,
        }

    required_keys = [
        "enableStream",
        "enableCameraSpoof",
        "enableRc",
        "enableLaneKeeping",
        "enableSIM",
        "enableIntersectionDet",
        "enableSignDet",
        "using_server",
        "start_idx",
        "end_idx",
        "pc_ip",
        "pi_ip",
        "home_loc",
    ]

    missing_keys = []
    for key in required_keys:
        if key not in config:
            missing_keys.append(key)

    assert not missing_keys, f"Missing :{missing_keys} in config.json"

    print("Started with config: ")

    pprint(config)
    return config
