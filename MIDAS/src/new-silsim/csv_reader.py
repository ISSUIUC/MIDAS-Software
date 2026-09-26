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
