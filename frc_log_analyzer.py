# Before you can run this script, you need to install the following dependencies:
# pip install robotpy-wpiutil
# pip install git+https://github.com/ligerbots/dslogparser@updateTo2026
# pip install -U google-genai
#
# usage:
# python frc_log_analyzer.py <log file>
# python frc_log_analyzer.py match.dslog --dslog-profile dslogprofiles22.xml
# python frc_log_analyzer.py match.dslog --dslog-profile dslogprofiles22.xml --profile-name Arrakis

import os
import sys
import json
import argparse
import re
import xml.etree.ElementTree as ET
from google import genai
from wpiutil.log import DataLogReader
import struct
from dslogparser import DSLogParser

_PDP_NAME_RE = re.compile(r"^pdp(\d+)$", re.IGNORECASE)


# not sure if this is needed
#model = genai.GenerativeModel('gemini-pro')

def load_dslog_profile_channel_names(
    profile_path: str, profile_name: str | None = None
) -> dict[int, str]:
    """
    Parse a Driver Station dslogprofiles XML (e.g. dslogprofiles22.xml).
    Returns mapping PDP channel index -> SeriesChildNode Name (e.g. 0 -> "pdp0").
    Only nodes whose Name matches pdp<N> are included; totals like total0 are ignored.
    If profile_name is set, selects the GroupProfile with matching <Name>; otherwise
    uses the first GroupProfile in the file.
    """
    tree = ET.parse(profile_path)
    root = tree.getroot()
    profiles = root.findall("GroupProfile")
    if not profiles:
        return {}
    chosen = None
    if profile_name is not None:
        for gp in profiles:
            n = gp.findtext("Name", default="")
            if (n or "").strip() == profile_name.strip():
                chosen = gp
                break
        if chosen is None:
            raise ValueError(
                f"No GroupProfile named {profile_name!r} in {profile_path!r}"
            )
    else:
        chosen = profiles[0]

    out: dict[int, str] = {}
    groups = chosen.find("Groups")
    if groups is None:
        return out
    for group in groups.findall("SeriesGroupNode"):
        # Driver Station XML uses the typo "Childern".
        for child in group.findall("Childern/SeriesChildNode"):
            name_el = child.find("Name")
            if name_el is None or name_el.text is None:
                continue
            name = name_el.text.strip()
            m = _PDP_NAME_RE.match(name)
            if not m:
                continue
            ch = int(m.group(1))
            if ch not in out:
                out[ch] = name
    return out


def _channel_label(profile_index_to_name: dict[int, str] | None, idx: int) -> str:
    """JSON key for PDP channel idx: profile Name (e.g. pdp0) when present, else pdp<N>."""
    if profile_index_to_name and idx in profile_index_to_name:
        return profile_index_to_name[idx]
    return f"pdp{idx}"


# --- DSLog Parsing Logic ---
def _dslog_power_snapshot(
    rec: dict, profile_index_to_name: dict[int, str] | None = None
) -> dict:
    """
    Collect all power-related fields exposed by dslogparser for one DSLog record.
    (Battery voltage is kept at the event level as battery_voltage.)
    """
    currents = rec.get("pd_currents") or []
    pd_currents_dict = {
        _channel_label(profile_index_to_name, i): round(float(a), 2)
        for i, a in enumerate(currents)
    }
    out = {
        "pd_id": rec.get("pd_id"),
        "pd_type": rec.get("pd_type"),
        "pd_currents": pd_currents_dict,
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


def parse_dslog(
    file_path: str,
    voltage_threshold: float = 6.3,
    profile_index_to_name: dict[int, str] | None = None,
):
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

            power = _dslog_power_snapshot(rec, profile_index_to_name)
            pd_currents_raw = rec.get("pd_currents") or []
            significant_loads = {}
            for idx, amps in enumerate(pd_currents_raw):
                if amps > 15.0:
                    significant_loads[_channel_label(profile_index_to_name, idx)] = (
                        round(float(amps), 2)
                    )

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
    parser = argparse.ArgumentParser(
        description="Analyze FRC .dslog or .wpilog for brownout-style events."
    )
    parser.add_argument("file", help="Path to .dslog or .wpilog")
    parser.add_argument(
        "--dslog-profile",
        metavar="XML",
        help=(
            "Driver Station profile XML (e.g. dslogprofiles22.xml). "
            "When used with .dslog, names pd_currents keys from SeriesChildNode "
            "<Name> (pdp0..pdp23); unlisted channels use pdp<N>."
        ),
    )
    parser.add_argument(
        "--profile-name",
        metavar="NAME",
        help=(
            "GroupProfile <Name> to use when the XML contains multiple profiles "
            "(default: first profile in file)."
        ),
    )
    args = parser.parse_args()
    print(args)

    profile_map: dict[int, str] | None = None
    if args.dslog_profile:
        try:
            profile_map = load_dslog_profile_channel_names(
                args.dslog_profile, args.profile_name
            )
        except (ET.ParseError, OSError, ValueError) as e:
            print(f"Error loading dslog profile: {e}", file=sys.stderr)
            sys.exit(1)

    ext = os.path.splitext(args.file)[1].lower()

    if ext == ".wpilog":
        data = parse_wpilog(args.file)
    elif ext == ".dslog":
        data = parse_dslog(args.file, profile_index_to_name=profile_map)
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