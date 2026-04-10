# Dependencies
# pip install robotpy-wpiutil
# DO NOT USE pip install dslogparser -- it is obsolete.  Use 
# pip install git+https://github.com/ligerbots/dslogparser@updateTo2026
# 
# pip install -U google-genai
#
# usage:
# python frc_log_analyzer.py <log file>

import os
import sys
import json
import argparse
from google import genai
from wpiutil.log import DataLogReader
import struct

# Allow importing DSLogParser from the local child directory.
# sys.path.insert(0, os.path.join(os.path.dirname(__file__), "dslogparser"))
# from dslogparser import DSLogParser



# Import the analyze function from the previous step or include it here

#model = genai.GenerativeModel('gemini-pro')

# --- DSLog Parsing Logic ---
def parse_dslog(file_path: str, voltage_threshold: float = 6.3):
    """
    Analyzes an FRC DS log file for brownout-style events.
    Uses local dslogparser's DSLogParser.read_records() API.
    """
    brownout_events = []
    ds_parser = DSLogParser(file_path)
    try:
        for rec in ds_parser.read_records():
            voltage = float(rec.get("voltage", 0.0))
            is_brownout = bool(rec.get("brownout", False))
            is_low_voltage = voltage < voltage_threshold
            if not (is_brownout or is_low_voltage):
                continue

            timestamp = rec.get("time")
            timestamp_str = timestamp.isoformat() if timestamp else None
            round_trip_ms = rec.get("round_trip_time")
            file_time = rec.get("file_time")

            # Debounce on monotonic DS file time if available.
            # If not available, fall back to round-trip-time domain.
            if brownout_events:
                if file_time is not None:
                    prev = brownout_events[-1].get("file_time")
                    if prev is not None and (float(file_time) - float(prev)) < 0.5:
                        continue
                elif round_trip_ms is not None:
                    prev = brownout_events[-1].get("round_trip_time_ms")
                    if prev is not None and (float(round_trip_ms) - float(prev)) < 0.5:
                        continue

            pd_currents = rec.get("pd_currents", [])
            significant_loads = {}
            for idx, amps in enumerate(pd_currents):
                if amps > 15.0:
                    significant_loads[f"PDP:{idx}"] = round(amps, 2)

            brownout_events.append({
                "timestamp": timestamp_str,
                "file_time": round(float(file_time), 3) if file_time is not None else None,
                "round_trip_time_ms": round(float(round_trip_ms), 3) if round_trip_ms is not None else None,
                "battery_voltage": round(voltage, 2),
                "rio_cpu": round(float(rec.get("rio_cpu", 0.0)), 3),
                "can_usage": round(float(rec.get("can_usage", 0.0)), 3),
                "packet_loss": round(float(rec.get("packet_loss", 0.0)), 3),
                "brownout_flag": is_brownout,
                "total_current": round(float(rec.get("pd_total_current", 0.0)), 2),
                "significant_loads": significant_loads,
            })
    finally:
        ds_parser.close()

    return {
        "summary": f"Detected {len(brownout_events)} brownout events.",
        "events": brownout_events
    }

# --- Parse WPILog Logic (Modified for compatibility) ---
def parse_wpilog(file_path: str, voltage_threshold: float = 6.3):
    """
    Analyzes a WPILib log file for brownout events and identifies high-current causes.
    """
    reader = DataLogReader(file_path)
    if not reader.isValid():
        return {"error": "Invalid or corrupt .wpilog file"}

    # Mapping and state tracking
    entries = {}        # Map: Entry ID -> Entry Name
    current_values = {} # Map: Entry Name -> Last known value
    brownout_events = []
    
    # Common FRC power keys
    VOLTAGE_KEY = "DS:BatteryVoltage"
    CURRENT_PREFIX = "PowerDistribution:Current"

    for record in reader:
        timestamp = record.getTimestamp() / 1000000.0 # Convert us to seconds
        
        # 1. Handle Metadata (Mapping Names to IDs)
        if record.isControl():
            try:
                start_data = record.getStartData()
                entries[start_data.entry] = start_data.name
            except Exception:
                continue
            continue

        # 2. Extract Data
        entry_id = record.getEntry()
        entry_name = entries.get(entry_id)
        if not entry_name:
            continue

        # WPILib doubles are stored as 8-byte IEEE 754
        try:
            raw_data = record.getData()
            if len(raw_data) == 8:
                value = struct.unpack("<d", raw_data)[0]
                current_values[entry_name] = value
            else:
                continue
        except Exception:
            continue

        # 3. Detect Brownout and Correlate
        if entry_name == VOLTAGE_KEY and value < voltage_threshold:
            # Check if we already logged a brownout in the last 0.5s (debounce)
            if not brownout_events or (timestamp - brownout_events[-1]["timestamp"] > 0.5):
                # Snapshot all current-drawing channels > 15A
                culprits = {
                    k: round(v, 2) for k, v in current_values.items() 
                    if CURRENT_PREFIX in k and v > 15.0
                }
                
                brownout_events.append({
                    "timestamp": round(timestamp, 3),
                    "battery_voltage": round(value, 2),
                    "total_current": round(current_values.get("PowerDistribution:TotalCurrent", 0), 2),
                    "significant_loads": culprits
                })

    return {
        "summary": f"Detected {len(brownout_events)} brownout events.",
        "events": brownout_events
    }

# Usage example for Gemini CLI integration:
# log_analysis = analyze_frc_brownouts("logs/match_1.wpilog")
# print(json.dumps(log_analysis, indent=2))

def get_gemini_diagnosis(log_data):
    """
    Sends brownout data to Gemini using the new google-genai SDK.
    """
    # Initialize the client. 
    # It automatically looks for the GEMINI_API_KEY environment variable.
    client = genai.Client(api_key=os.environ.get("GOOGLE_API_KEY"))

    prompt_content = f"""
    Act as a Lead FRC Control Systems Engineer. Analyze these brownout events 
    from a .wpilog file. For each event, provide a "Root Cause Analysis" 
    and a "Suggested Mechanical, Electrical or Software Fix."

    Data:
    {json.dumps(log_data, indent=2)}
    """
    
    # Use the newer model 'gemini-2.0-flash' for fast, efficient telemetry analysis
    response = client.models.generate_content(
        model="gemini-2.0-flash", 
        contents=prompt_content
    )
    
    return response.text

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file")
    args = parser.parse_args()

    ext = os.path.splitext(args.file)[1].lower()
    
    if ext == ".wpilog":
        data = parse_wpilog(args.file)
    elif ext == ".dslog":
        data = parse_dslog(args.file)
    else:
        print("Unsupported file format.")
        return

    print(json.dumps(data, indent=2));

    # if data["events"]:
    #     print(get_gemini_diagnosis(data))
    # else:
    #     print("No issues detected.")

if __name__ == "__main__":
    main()