
import json
from pathlib import Path


SUPPORTED_SCENARIOS = {
    "rear_end_collision": {
        "required_sections": [
            "scenario_id",
            "map",
            "ego",
            "lead_vehicle",
            "evaluation",
        ],
        "required_fields": [
            ("lead_vehicle", "initial_gap_m"),
        ],
    },
    "lane_change_cut_in": {
        "required_sections": [
            "scenario_id",
            "map",
            "ego",
            "cut_in_vehicle",
            "evaluation",
        ],
        "required_fields": [
            ("cut_in_vehicle", "initial_gap_m"),
            ("cut_in_vehicle", "initial_lane"),
            ("cut_in_vehicle", "target_lane"),
            ("cut_in_vehicle", "lane_change_start_time_s"),
            ("cut_in_vehicle", "lane_change_duration_s"),
        ],
    },
}


def load_scenario_config(scenario_path):
    """Load and validate a supported JSON scenario."""
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

    scenario_type = config.get("scenario_type")

    if scenario_type not in SUPPORTED_SCENARIOS:
        supported = ", ".join(SUPPORTED_SCENARIOS)
        raise ValueError(
            f"Unsupported scenario_type: {scenario_type!r}. "
            f"Supported types: {supported}"
        )

    schema = SUPPORTED_SCENARIOS[scenario_type]

    missing_sections = [
        key
        for key in schema["required_sections"]
        if key not in config
    ]

    if missing_sections:
        raise ValueError(
            "Scenario JSON is missing required fields: "
            + ", ".join(missing_sections)
        )

    for section_name, field_name in schema["required_fields"]:
        section = config.get(section_name)

        if not isinstance(section, dict):
            raise ValueError(
                f"'{section_name}' must be a JSON object."
            )

        if field_name not in section:
            raise ValueError(
                f"Scenario JSON is missing "
                f"{section_name}.{field_name}."
            )

    print(f"[INFO] Loaded scenario: {config['scenario_id']}")
    print(f"[INFO] Scenario type: {scenario_type}")

    return config
