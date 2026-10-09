import csv
from pathlib import Path
# import pandas as pd

# THIS CODE TAKES A LARGE CSV WITH LOTS OF SENSOR DATA AND SORTS INTO SEPARATE CSV's PER SENSOR
BASE_DIR = Path(__file__).resolve().parent
csv_path = (BASE_DIR / "data" / "midas_sustainer_flight.csv")

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
    flight_start_time = int(next(csv_reader)[0])
    sensors = set()
    for header in headers:
        if "." not in header: # sensors have periods in their names.
            continue
        sensor = header.split(".")[0]
        sensors.add(sensor)
    sensors = list(sensors)
    # print(sensors)

# for each sensor, we need to generate a new csv file within data/outputs

(BASE_DIR / "data" / "sensors").mkdir(parents=True, exist_ok=True)
for sensor in sensors: 
    with open(csv_path, mode="r", encoding="utf-8") as f: # reopen file
        csv_reader = csv.reader(f)
        headers = next(csv_reader)
        relevant_cols = []
        relevant_cols.append(0)
        for i, header in enumerate(headers):
            if header.startswith(sensor + "."):
                relevant_cols.append(i)
        relevant_headers = [headers[i] for i in relevant_cols]
        csv_table = []

        csv_table.append(relevant_headers)

        # line = next(csv_reader)
        csv_data = []
        c = 0
        # init_time = int(line[0]) # the first timestamp in the csv for this sensor because we subtract it so it starts at zero.
        prev_relevant_values = []
        for line in csv_reader:
            c+= 1 # this is for sanity
            if c % 10000 == 0:
                print(c)
            relevant_values = []
            n : bool = False
            for i, col in enumerate(relevant_cols[1:]):
                # n denotes whether to actually add to csv and this only occurs if some value actually changed.
                relevant_values.append(line[col])
                if len(prev_relevant_values) > 0:
                    if line[col] != prev_relevant_values[i]:
                        n = True
                else:
                    n = True
                    init_time = int(line[0])

            if (n): # adds to the csv_table with timestamp.
                relevant_values_and_timestamp = [str(int(line[0]) - flight_start_time)] # appending the timestamp to the front.
                relevant_values_and_timestamp.extend(relevant_values)
                csv_table.append(relevant_values_and_timestamp)
                prev_relevant_values = [val for val in relevant_values]
        with open(BASE_DIR / "data" / "sensors" / f"{sensor}.csv", mode="w",newline='', encoding='utf-8') as f2:
            csv_writer = csv.writer(f2)
            csv_writer.writerows(csv_table)

print("done all sensor CSVs written")
