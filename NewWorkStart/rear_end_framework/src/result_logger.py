
import csv
import json
from pathlib import Path


CSV_FIELDS = [
    "time_s",
    "ego_speed_kmh",
    "lead_speed_kmh",
    "center_distance_m",
    "bumper_gap_m",
    "approx_ttc_s",
    "collision",
]


def create_results_directory(directory="results"):
    """Create the results directory if it does not exist."""
    results_dir = Path(directory)
    results_dir.mkdir(parents=True, exist_ok=True)
    return results_dir


def create_csv_writer(file_handle, fieldnames=None):
    """Create a CSV writer (standard columns unless `fieldnames` given)."""
    writer = csv.DictWriter(
        file_handle,
        fieldnames=fieldnames or CSV_FIELDS
    )
    writer.writeheader()
    return writer


def save_summary(summary_path, summary):
    """Save the scenario summary as a JSON file."""
    summary_path = Path(summary_path)
    summary_path.parent.mkdir(parents=True, exist_ok=True)

    summary_path.write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8"
    )
