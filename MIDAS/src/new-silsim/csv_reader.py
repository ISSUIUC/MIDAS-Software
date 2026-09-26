import csv
from pathlib import Path
import pandas as pd

BASE_DIR = Path(__file__).resolve().parent
csv_path = BASE_DIR / "data" / "midas_sustainer_flight.csv"

# with open(csv_path, mode='r', encoding='utf-8') as file:
#     csv_reader = csv.reader(file)
    
#     header = next(csv_reader) 
    
#     for row in csv_reader:
#         print(row)

try:
    df = pd.read_csv(csv_path, skipinitialspace=True)
    
    print("da--- Data Summary ---")
    print(f"Total telemetry data points: {len(df)}")
    print(f"Log start time: {df['timestamp_ms'].min()} ms")
    print(f"Log end time: {df['timestamp_ms'].max()} ms")
    
    print("\n--- Data ---")
    columns_to_show = ['timestamp_ms', 'sensor', 'barometer.altitude', 'kalman.position.pz', 'fsm.state']
    print(df[columns_to_show].head(10))


except FileNotFoundError:
    print(f"Error: Could not locate the telemetry file at {csv_path}")
except KeyError as e:
    print(f"Error: Column mismatch or unexpected missing column: {e}")


# loop through the headers and find all the sensors we care about
with open(csv_path, mode="r", encoding="utf-8") as f:
    csv_reader = csv.reader(f)

    headers = next(csv_reader)
    sensors = set()
    for header in headers:
        if "." not in header:
            continue
        sensor = header.split(".")[0]
        sensors.add(sensor)
    sensors = list(sensors)
    print(sensors)

# for each sensor, we need to generate a new csv file within data/outputs

for sensor in sensors:
    with open(csv_path, mode="r", encoding="utf-8") as f:
        csv_reader = csv.reader(f)
        headers = next(csv_reader)
        relevant_cols = []
        for i, header in enumerate(headers):
            if header.startswith(sensor):
                relevant_cols.append(i)
        relevant_headers = [headers[0]] + [headers[i] for i in relevant_cols]

        

        line = next(csv_reader)


