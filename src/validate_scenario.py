
import json
import sys
from pathlib import Path

from jsonschema import validate, ValidationError

SCHEMA = {
    "type": "object",
    "required": [
        "scenario_id", "scenario_type", "map", "ego",
        "lead_vehicle", "environment", "evaluation", "termination"
    ],
    "properties": {
        "scenario_id": {"type": "string", "minLength": 1},
        "scenario_type": {"const": "rear_end_collision"},
        "map": {"type": "string", "minLength": 1},
        "ego": {
            "type": "object",
            "required": ["speed_kmh"],
            "properties": {
                "speed_kmh": {"type": "number", "exclusiveMinimum": 0,
                              "maximum": 150}
            },
            "additionalProperties": False
        },
        "lead_vehicle": {
            "type": "object",
            "required": [
                "speed_kmh", "initial_gap_m",
                "brake_time_s", "target_deceleration_mps2"
            ],
            "properties": {
                "speed_kmh": {"type": "number", "minimum": 0, "maximum": 150},
                "initial_gap_m": {"type": "number", "exclusiveMinimum": 0,
                                  "maximum": 100},
                "brake_time_s": {"type": "number", "minimum": 0, "maximum": 60},
                "target_deceleration_mps2": {
                    "type": "number", "exclusiveMinimum": 0, "maximum": 12
                }
            },
            "additionalProperties": False
        },
        "environment": {
            "type": "object",
            "required": ["weather"],
            "properties": {
                "weather": {"enum": ["clear", "rain", "fog"]}
            },
            "additionalProperties": False
        },
        "evaluation": {
            "type": "object",
            "required": [
                "collision_detection",
                "record_minimum_distance",
                "record_ttc"
            ],
            "properties": {
                "collision_detection": {"type": "boolean"},
                "record_minimum_distance": {"type": "boolean"},
                "record_ttc": {"type": "boolean"}
            },
            "additionalProperties": False
        },
        "termination": {
            "type": "object",
            "required": ["max_duration_s"],
            "properties": {
                "max_duration_s": {
                    "type": "number", "exclusiveMinimum": 0, "maximum": 120
                }
            },
            "additionalProperties": False
        }
    },
    "additionalProperties": False
}


def main():
    path = Path(sys.argv[1] if len(sys.argv) > 1
                else "scenarios/rear_end_001.json")

    with path.open(encoding="utf-8") as f:
        scenario = json.load(f)

    validate(instance=scenario, schema=SCHEMA)

    if scenario["lead_vehicle"]["initial_gap_m"] < 5:
        raise ValueError("Initial gap < 5 m: review spawn safety.")

    print(f"VALID: {scenario['scenario_id']}")
    print(f"Map: {scenario['map']}")
    print("JSON schema and basic constraints passed.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, json.JSONDecodeError, ValidationError, ValueError) as exc:
        print(f"INVALID SCENARIO: {exc}", file=sys.stderr)
        sys.exit(1)
