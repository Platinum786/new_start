
import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
RUNNER = PROJECT_ROOT / "src" / "run_pygame_scenario.py"
BATCH_CONFIG = PROJECT_ROOT / "experiments" / "batch_config.json"
BASE_SCENARIO = PROJECT_ROOT / "scenarios" / "rear_end_001.json"
RESULTS_DIR = PROJECT_ROOT / "results"
GENERATED_DIR = RESULTS_DIR / "batch_scenarios"


def load_json(path):
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def main():
    parser = argparse.ArgumentParser(
        description="Run a batch of CARLA rear-end experiments."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--res", default="640x480")
    args = parser.parse_args()

    batch_config = load_json(BATCH_CONFIG)
    base_config = load_json(BASE_SCENARIO)

    experiments = batch_config.get("experiments", [])
    if not experiments:
        raise ValueError("No experiments found in batch_config.json.")

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    GENERATED_DIR.mkdir(parents=True, exist_ok=True)

    master_csv = RESULTS_DIR / "batch_results.csv"

    fieldnames = [
        "experiment_id",
        "scenario_id",
        "initial_gap_m",
        "ego_speed_kmh",
        "lead_speed_kmh",
        "brake_time_s",
        "collision",
        "minimum_center_distance_m",
        "minimum_bumper_gap_m",
        "minimum_approx_ttc_s",
        "duration_simulated_s",
        "status",
    ]

    rows = []

    for index, experiment in enumerate(experiments, start=1):
        experiment_id = experiment["experiment_id"]
        scenario_id = f"batch_{experiment_id}"

        print(f"\n[{index}/{len(experiments)}] Running {experiment_id}")

        config = json.loads(json.dumps(base_config))
        config["scenario_id"] = scenario_id
        config["lead_vehicle"]["initial_gap_m"] = float(
            experiment["initial_gap_m"]
        )
        config["ego"]["speed_kmh"] = float(
            experiment["ego_speed_kmh"]
        )
        config["lead_vehicle"]["speed_kmh"] = float(
            experiment["lead_speed_kmh"]
        )
        config["lead_vehicle"]["brake_time_s"] = float(
            experiment["brake_time_s"]
        )

        scenario_path = GENERATED_DIR / f"{scenario_id}.json"
        with scenario_path.open("w", encoding="utf-8") as file:
            json.dump(config, file, indent=2)

        command = [
            sys.executable,
            str(RUNNER),
            "--host", args.host,
            "--port", str(args.port),
            "--scenario", str(scenario_path),
            "--res", args.res,
        ]

        try:
            subprocess.run(
                command,
                cwd=PROJECT_ROOT,
                check=True,
            )

            summary_path = RESULTS_DIR / f"{scenario_id}_summary.json"
            if not summary_path.exists():
                raise FileNotFoundError(
                    f"Expected summary not found: {summary_path}"
                )

            summary = load_json(summary_path)

            rows.append({
                "experiment_id": experiment_id,
                "scenario_id": scenario_id,
                "initial_gap_m": experiment["initial_gap_m"],
                "ego_speed_kmh": experiment["ego_speed_kmh"],
                "lead_speed_kmh": experiment["lead_speed_kmh"],
                "brake_time_s": experiment["brake_time_s"],
                "collision": summary.get("collision"),
                "minimum_center_distance_m": summary.get(
                    "minimum_center_distance_m"
                ),
                "minimum_bumper_gap_m": summary.get(
                    "minimum_bumper_gap_m"
                ),
                "minimum_approx_ttc_s": summary.get(
                    "minimum_approx_ttc_s"
                ),
                "duration_simulated_s": summary.get(
                    "duration_simulated_s"
                ),
                "status": "completed",
            })

        except (subprocess.CalledProcessError, Exception) as error:
            print(f"[ERROR] {experiment_id} failed: {error}")
            rows.append({
                "experiment_id": experiment_id,
                "scenario_id": scenario_id,
                "initial_gap_m": experiment["initial_gap_m"],
                "ego_speed_kmh": experiment["ego_speed_kmh"],
                "lead_speed_kmh": experiment["lead_speed_kmh"],
                "brake_time_s": experiment["brake_time_s"],
                "collision": None,
                "minimum_center_distance_m": None,
                "minimum_bumper_gap_m": None,
                "minimum_approx_ttc_s": None,
                "duration_simulated_s": None,
                "status": "failed",
            })

    with master_csv.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"\nBatch finished. Master results: {master_csv}")
    print(f"Completed: {sum(r['status'] == 'completed' for r in rows)}")
    print(f"Failed: {sum(r['status'] == 'failed' for r in rows)}")


if __name__ == "__main__":
    main()
