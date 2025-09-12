import pandas as pd
import re
# THIS IS CHATGPT CODE I AM SUPREMELY LAZY ABOUT THIS
txt_path = r"C:\Users\davis\OneDrive\Documents\GitHub\UGASpaceport\Data Analysis\telemmytree.txt"
def parse_telemetry(txt_path):
    packets = []
    current = {}
    mode = None  # will be 'accel', 'gyro', or 'mag'
    
    with open(txt_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            # packet #
            m = re.match(r'^packet\s*#\s*:\s*(\d+)', line, re.IGNORECASE)
            if m:
                current['packet'] = int(m.group(1))
                continue
            
            # section headers
            if line.upper() in ('ACCEL', 'GYRO', 'MAG'):
                mode = line.lower()
                continue
            
            # coordinate lines: X: value, Y: value, Z: value
            m = re.match(r'^([XYZ])\s*:\s*([-\d\.]+)', line, re.IGNORECASE)
            if m and mode:
                axis = m.group(1).lower()
                val  = float(m.group(2))
                current[f"{mode}_{axis}"] = val
                continue
            
            # separator
            if line.startswith('---'):
                # finished one packet
                if current:
                    packets.append(current)
                current = {}
                mode = None
    
    # in case there's no trailing separator
    if current:
        packets.append(current)
    
    # turn into DataFrame
    df = pd.DataFrame(packets)
    return df

# Usage:
df = parse_telemetry('telemmytree.txt')
df.to_csv('telemetry.csv', index=False)
print("Wrote", len(df), "packets to telemetry.csv")
