# Batch-convert FRC Driver Station .dslog files in a folder to the same JSON shape
# as frc_log_analyzer.parse_dslog (summary + events). No WPILog parsing, no Gemini.
#
# Dependencies (same as dslog path in frc_log_analyzer.py):
#   pip install robotpy-wpiutil
#   pip install git+https://github.com/ligerbots/dslogparser@updateTo2026
#
# Usage:
#   python frc_dslog_folder_to_json.py C:\logs\event1 --min-duration-seconds 90
#   python frc_dslog_folder_to_json.py ./logs --dslog-profile dslogprofiles22.xml --profile-name Arrakis

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

from dslogparser import DSLogParser

_PDP_NAME_RE = re.compile(r"^pdp(\d+)$", re.IGNORECASE)


def load_dslog_profile_channel_names(
    profile_path: str, profile_name: str | None = None
) -> dict[int, str]:
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
    if profile_index_to_name and idx in profile_index_to_name:
        return profile_index_to_name[idx]
    return f"pdp{idx}"


def _dslog_power_snapshot(
    rec: dict, profile_index_to_name: dict[int, str] | None = None
) -> dict:
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
    if "pd_resistance" in rec:
        out["pd_resistance"] = rec.get("pd_resistance")
    if "pd_voltage" in rec:
        out["pd_voltage"] = rec.get("pd_voltage")
    if "pd_temp" in rec:
        out["pd_temp"] = rec.get("pd_temp")
    out["robot_disabled"] = bool(rec.get("robot_disabled", False))
    out["robot_auto"] = bool(rec.get("robot_auto", False))
    out["robot_tele"] = bool(rec.get("robot_tele", False))
    return out


def _duration_from_bounds(
    min_ft: float | None,
    max_ft: float | None,
    min_dt: datetime | None,
    max_dt: datetime | None,
) -> float:
    if min_ft is not None and max_ft is not None:
        return max(0.0, max_ft - min_ft)
    if min_dt is not None and max_dt is not None:
        return max(0.0, (max_dt - min_dt).total_seconds())
    return 0.0


def parse_dslog(
    file_path: str,
    voltage_threshold: float = 6.3,
    profile_index_to_name: dict[int, str] | None = None,
) -> tuple[float, dict]:
    """
    Single pass: compute approximate duration (seconds) and the same JSON-shaped
    result as frc_log_analyzer.parse_dslog.
    """
    brownout_events: list[dict] = []
    min_ft: float | None = None
    max_ft: float | None = None
    min_dt: datetime | None = None
    max_dt: datetime | None = None

    ds_parser = DSLogParser(file_path)
    try:
        for rec in ds_parser.read_records():
            ft = rec.get("file_time")
            if ft is not None:
                v = float(ft)
                min_ft = v if min_ft is None else min(min_ft, v)
                max_ft = v if max_ft is None else max(max_ft, v)
            ts = rec.get("time")
            if isinstance(ts, datetime):
                min_dt = ts if min_dt is None else min(min_dt, ts)
                max_dt = ts if max_dt is None else max(max_dt, ts)

            voltage = float(rec.get("voltage", 0.0))
            is_brownout = bool(rec.get("brownout", False))
            is_low_voltage = voltage < voltage_threshold
            if not (is_brownout or is_low_voltage):
                continue

            timestamp = rec.get("time")
            timestamp_str = timestamp.isoformat() if timestamp else None
            round_trip_ms = rec.get("round_trip_time")
            file_time = rec.get("file_time")

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

    duration = _duration_from_bounds(min_ft, max_ft, min_dt, max_dt)
    return duration, {
        "summary": f"Detected {len(brownout_events)} brownout events.",
        "events": brownout_events,
    }


def _json_key_for_dslog(path: Path) -> str:
    """Stable object key for combined JSON (full file name, e.g. qual12.dslog)."""
    return path.name


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Parse all .dslog files in a folder to per-file JSON plus one combined JSON "
            "(same schema as frc_log_analyzer.parse_dslog). Skips logs shorter than "
            "--min-duration-seconds."
        )
    )
    parser.add_argument(
        "folder",
        help="Directory containing .dslog files",
    )
    parser.add_argument(
        "--min-duration-seconds",
        type=float,
        default=0.0,
        metavar="SEC",
        help=(
            "Skip .dslog files whose sampled duration is below this many seconds "
            "(default: 0, process all)."
        ),
    )
    parser.add_argument(
        "--dslog-profile",
        metavar="XML",
        help=(
            "Driver Station profile XML (e.g. dslogprofiles22.xml). "
            "Names pd_currents keys from SeriesChildNode <Name> (pdp0..pdp23)."
        ),
    )
    parser.add_argument(
        "--profile-name",
        metavar="NAME",
        help="GroupProfile <Name> when the XML has multiple profiles (default: first).",
    )
    parser.add_argument(
        "--voltage-threshold",
        type=float,
        default=6.3,
        help="Voltage below which a sample counts as a brownout-style event (default: 6.3).",
    )
    parser.add_argument(
        "--output-dir",
        metavar="DIR",
        help="Directory for per-file .json outputs (default: same folder as each .dslog).",
    )
    parser.add_argument(
        "--combined-output",
        metavar="PATH",
        help="Path for combined JSON (default: <folder>/dslog_combined.json).",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Find .dslog files recursively; combined JSON keys use paths relative to folder.",
    )
    args = parser.parse_args()

    folder = Path(args.folder).resolve()
    if not folder.is_dir():
        print(f"Not a directory: {folder}", file=sys.stderr)
        sys.exit(1)

    profile_map: dict[int, str] | None = None
    if args.dslog_profile:
        try:
            profile_map = load_dslog_profile_channel_names(
                args.dslog_profile, args.profile_name
            )
        except (ET.ParseError, OSError, ValueError) as e:
            print(f"Error loading dslog profile: {e}", file=sys.stderr)
            sys.exit(1)

    if args.recursive:
        dslogs = sorted(folder.rglob("*.dslog"))
    else:
        dslogs = sorted(folder.glob("*.dslog"))

    if not dslogs:
        print(f"No .dslog files in {folder}", file=sys.stderr)
        sys.exit(0)

    out_dir = Path(args.output_dir).resolve() if args.output_dir else None
    if out_dir is not None:
        out_dir.mkdir(parents=True, exist_ok=True)

    combined: dict[str, dict] = {}
    skipped_short: list[tuple[str, float]] = []

    for dslog_path in dslogs:
        duration, data = parse_dslog(
            str(dslog_path),
            voltage_threshold=args.voltage_threshold,
            profile_index_to_name=profile_map,
        )
        if duration < args.min_duration_seconds:
            skipped_short.append((dslog_path.name, duration))
            continue

        if args.recursive:
            try:
                rel = dslog_path.relative_to(folder)
            except ValueError:
                rel = dslog_path
            key = rel.as_posix()
        else:
            key = _json_key_for_dslog(dslog_path)

        combined[key] = data

        if out_dir is not None:
            if args.recursive:
                try:
                    rel = dslog_path.relative_to(folder)
                    json_path = out_dir / rel.with_suffix(".json")
                except ValueError:
                    json_path = out_dir / (dslog_path.stem + ".json")
                json_path.parent.mkdir(parents=True, exist_ok=True)
            else:
                json_path = out_dir / (dslog_path.stem + ".json")
        else:
            json_path = dslog_path.with_suffix(".json")

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
            f.write("\n")

    combined_path = (
        Path(args.combined_output).resolve()
        if args.combined_output
        else folder / "dslog_combined.json"
    )
    with open(combined_path, "w", encoding="utf-8") as f:
        json.dump(combined, f, indent=2)
        f.write("\n")

    print(
        f"Processed {len(combined)} .dslog file(s); combined: {combined_path}",
        file=sys.stderr,
    )
    if skipped_short:
        print(
            f"Skipped {len(skipped_short)} file(s) shorter than "
            f"{args.min_duration_seconds} s:",
            file=sys.stderr,
        )
        for name, d in skipped_short:
            print(f"  {name} (duration ~{d:.2f} s)", file=sys.stderr)


if __name__ == "__main__":
    main()
