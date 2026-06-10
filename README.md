# Radar Target Simulation and Visualization Tool

This repository provides tools to simulate radar targets and visualize radar coverage maps.

## Simulate radar targets

### Input file

`input.json`

### Output file

`outputs/<YYYYMMDD_hhmmss>/radar_target_data.csv`

### Installation

```bash
pip install geopy
```

### Usage

```bash
python3 simulate_radar_targets.py
```

## Visualize radars

### Input file

`input.json`

### Installation

```bash
pip install matplotlib geographiclib
```

### Usage

```bash
python3 visualize_radars.py
```

## Sample `input.json`

```json
{
    "radars": [
        {"id": 1, "lat": 20.849, "lon": 106.711, "range_km": 300.0, "scan_period_s": 3},
        {"id": 2, "lat": 20.705, "lon": 106.785, "range_km": 300.0, "scan_period_s": 4}
    ],
    "num_targets": 40,
    "simulation_duration_s": 60
}
```
