import json
from pprint import pprint

try:
    # TODO: parse config path as an arg
    with open("./config_car.json") as f:
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
    }

required_keys = [
    "enableStream",
    "enableCameraSpoof",
    "enableRc",
    "enableLaneKeeping",
    "enableSIM",
    "enableIntersectionDet",
    "using_server",
    "start_idx",
    "end_idx",
    "pc_ip",
    "pi_ip",
]


missing_keys = []
for key in required_keys:
    if key not in config:
        missing_keys.append(key)

assert not missing_keys, f"Missing :{missing_keys} in config.json"

print("Started with config: ")
pprint(config)
