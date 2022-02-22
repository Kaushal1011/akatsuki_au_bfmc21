import json
from pprint import pprint

try:
    with open("../../config.json") as f:
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
        "start_idx":'86',
        "end_idx":'27'
    }

required_keys = [
    "enableStream",
    "enableCameraSpoof",
    "enableRc",
    "enableLaneKeeping",
    "enableSIM",
    "enableIntersectionDet",
    "start_idx",
    "end_idx",
]


missing_keys = []
for key in required_keys:
    if key not in config:
        missing_keys.append(key)

assert not missing_keys, f"Missing :{missing_keys} in config.json"

print("Started with config: ")
pprint(config)
