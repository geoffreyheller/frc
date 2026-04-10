# Before you can run this script, you need to install the following dependencies:
# pip install robotpy-wpiutil
# pip install git+https://github.com/ligerbots/dslogparser@updateTo2026
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
from dslogparser import DSLogParser


# not sure if this is needed
#model = genai.GenerativeModel('gemini-pro')

# --- DSLog Parsing Logic ---
def _dslog_power_snapshot(rec: dict) -> dict:
    """
    Collect all power-related fields exposed by dslogparser for one DSLog record.
    (Battery voltage is kept at the event level as battery_voltage.)
    """
    currents = rec.get("pd_currents")
    out = {
        "pd_id": rec.get("pd_id"),
        "pd_type": rec.get("pd_type"),
        "pd_currents": [round(float(a), 2) for a in (currents or [])],
        "pd_total_current": round(float(rec.get("pd_total_current", 0.0)), 2),
    }
    # Legacy CTRE PDP (DSLog v3): encoded auxiliary telemetry from the PDP block.
    if "pd_resistance" in rec:
        out["pd_resistance"] = rec.get("pd_resistance")
    if "pd_voltage" in rec:
        out["pd_voltage"] = rec.get("pd_voltage")
    if "pd_temp" in rec:
        out["pd_temp"] = rec.get("pd_temp")
    # Robot mode affects whether outputs are commanded (context for current draw).
    out["robot_disabled"] = bool(rec.get("robot_disabled", False))
    out["robot_auto"] = bool(rec.get("robot_auto", False))
    out["robot_tele"] = bool(rec.get("robot_tele", False))
    return out


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

            power = _dslog_power_snapshot(rec)
            pd_currents_raw = rec.get("pd_currents") or []
            significant_loads = {}
            for idx, amps in enumerate(pd_currents_raw):
                if amps > 15.0:
                    significant_loads[f"channel_{idx}"] = round(float(amps), 2)

            brownout_events.append({
                "timestamp": timestamp_str,
                "file_time": round(float(file_time), 3) if file_time is not None else None,
                "round_trip_time_ms": round(float(round_trip_ms), 3) if round_trip_ms is not None else None,
                "battery_voltage": round(voltage, 2),
                "rio_cpu": round(float(rec.get("rio_cpu", 0.0)), 3),
                "can_usage": round(float(rec.get("can_usage", 0.0)), 3),
                "packet_loss": round(float(rec.get("packet_loss", 0.0)), 3),
                "brownout_flag": is_brownout,
                "total_current": power["pd_total_current"],
                **power,
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
    Sends brownout data to Gemini using the google-genai SDK.
    """
    # Initialize the client. 
    # It automatically looks for the GEMINI_API_KEY environment variable.
    client = genai.Client(api_key=os.environ.get("GOOGLE_API_KEY"))

    prompt_content = f"""
    Role: Act as a Lead FRC Control Systems Engineer and Mechanical Mentor. Your goal is to diagnose robot "brownouts" (voltage drops) from telemetry data.

    Context:

    Brownout Limit: The roboRIO disables motor outputs at 6.3V.

    Battery Health: A healthy, fully charged battery stays above 12V at rest and shouldn't dip below 7V under a standard 100A load.

    The Data: I will provide a JSON list of "Brownout Events" containing timestamps, voltage, and a list of "Significant Loads" (motor controllers drawing >15A at the time of the dip).

    Diagnosis Logic:

    Stall Detection: If one or two motors (e.g., Drivetrain or Intake) have extremely high current (>40A each) during a voltage drop, suggest a mechanical stall or "pushing match."

    Battery Health: If the TotalCurrent is relatively low (<80A) but the voltage is still dropping below 6.5V, suggest the battery is end-of-life or has high internal resistance.

    Wiring Check: If the voltage recovery is slow, suggest checking the battery terminal tightness or the main 120A breaker.

    Task:
    Analyze the following log data. For each event, provide a 1-2 sentence "Root Cause Analysis" and a "Suggested Mechanical, Electrical or Software Fix."

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