
import json
from pathlib import Path


def load_scenario_config(scenario_path):
    """Load a scenario JSON file and validate its scenario type."""
    scenario_path = Path(scenario_path)

    if not scenario_path.is_file():
        raise FileNotFoundError(
            f"Scenario file not found: {scenario_path}"
        )

    with scenario_path.open("r", encoding="utf-8") as file:
        config = json.load(file)

    if not isinstance(config, dict):
        raise ValueError(
            "Scenario JSON must contain a JSON object."
        )

    if config.get("scenario_type") != "rear_end_collision":
        raise ValueError(
            "This runner supports rear_end_collision only."
        )

    required_sections = [
        "scenario_id",
        "map",
        "lead_vehicle",
        "evaluation",
    ]

    missing_sections = [
        key for key in required_sections if key not in config
    ]

    if missing_sections:
        raise ValueError(
            "Scenario JSON is missing required fields: "
            + ", ".join(missing_sections)
        )

    if "initial_gap_m" not in config["lead_vehicle"]:
        raise ValueError(
            "Scenario JSON is missing lead_vehicle.initial_gap_m."
        )

    return config
