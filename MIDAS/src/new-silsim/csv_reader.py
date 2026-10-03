import csv
from pathlib import Path
# import pandas as pd

BASE_DIR = Path(__file__).resolve().parent
csv_path = BASE_DIR / "data" / "midas_sustainer_flight copy.csv"

# with open(csv_path, mode='r', encoding='utf-8') as file:
#     csv_reader = csv.reader(file)
    
#     header = next(csv_reader) 
    
#     for row in csv_reader:
#         print(row)

# try:
#     df = pd.read_csv(csv_path, skipinitialspace=True)
    
#     print("da--- Data Summary ---")
#     print(f"Total telemetry data points: {len(df)}")
#     print(f"Log start time: {df['timestamp_ms'].min()} ms")
#     print(f"Log end time: {df['timestamp_ms'].max()} ms")
    
#     print("\n--- Data ---")
#     columns_to_show = ['timestamp_ms', 'sensor', 'barometer.altitude', 'kalman.position.pz', 'fsm.state']
#     print(df[columns_to_show].head(10))


# except FileNotFoundError:
#     print(f"Error: Could not locate the telemetry file at {csv_path}")
# except KeyError as e:
#     print(f"Error: Column mismatch or unexpected missing column: {e}")


# loop through the headers and find all the sensors we care about
with open(csv_path, mode="r", encoding="utf-8") as f:
    csv_reader = csv.reader(f)

    headers = next(csv_reader)
    sensors = set()
    for header in headers:
        if "." not in header: # sensors have periods in their names.
            continue
        sensor = header.split(".")[0]
        sensors.add(sensor)
    sensors = list(sensors)
    print(sensors)

# for each sensor, we need to generate a new csv file within data/outputs

for sensor in sensors:
    with open(csv_path, mode="r", encoding="utf-8") as f: # reopen file
        csv_reader = csv.reader(f)
        headers = next(csv_reader)
        relevant_cols = []
        relevant_cols.append(0)
        for i, header in enumerate(headers):
            if header.startswith(sensor):
                relevant_cols.append(i)
        relevant_headers = [headers[i] for i in relevant_cols]
        csv_table = []


        # with open(BASE_DIR / "data" / "sensors" / f"{sensor}.csv", mode="w",newline='', encoding='utf-8') as f2:
        #     csv_writer = csv.writer(f2)
        #     csv_writer.writerow(relevant_headers)
        csv_table.append(relevant_headers)

        line = next(csv_reader)
        csv_data = []
        c = 0
        prev_relevant_values = []
        for line in csv_reader:
            c+= 1
            if c % 1000 == 0:
                print(c)
            # filtered_line = line[*relevant_cols]
            relevant_values = []
            n : bool = False
            for i, col in enumerate(relevant_cols[1:]):
                # filtered_line = line[col]
                relevant_values.append(line[col])
                if len(prev_relevant_values) > 0:
                    if line[col] != prev_relevant_values[i-1]:
                        n = True
                        # print("Howdys")
                else:
                    n = True
                    # print("Howdys2")
                    
            if (n):
                csv_table.append(relevant_values)
                prev_relevant_values = [val for val in relevant_values]
        with open(BASE_DIR / "data" / "sensors" / f"{sensor}.csv", mode="w",newline='', encoding='utf-8') as f2:
            csv_writer = csv.writer(f2)
            csv_writer.writerows(csv_table)