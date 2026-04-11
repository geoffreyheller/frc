# FRC log analyzer

Python tool that scans **FRC Driver Station** (`.dslog`) or **WPILib** (`.wpilog`) logs for brownout-style events (low battery voltage and/or brownout flags). It prints a JSON report to stdout and can optionally send that data to **Google Gemini** for a short engineering-style diagnosis.

## Features

- **`.dslog`**: Uses [dslogparser](https://github.com/ligerbots/dslogparser) to read records; debounces events; includes PDP current snapshots and per-channel loads over 15 A.
- **`.wpilog`**: Uses WPILib `DataLogReader` (via `robotpy-wpiutil`) to correlate battery voltage with `PowerDistribution:Current*` entries.
- **Driver Station profile XML** (optional): Map PDP channel indices to the same `pdp0`…`pdp23` names as in a `dslogprofiles*.xml` export so `pd_currents` in JSON is a **dictionary** keyed by those names (with `pdp<N>` for any channel not listed in the profile).

## Requirements

- **Python** 3.10 or newer (recommended; the script uses modern type hints).
- A **Google AI API key** if you want Gemini analysis (see [Configuration](#configuration)).

## Installation

Clone or copy this repository, then install dependencies:

```bash
pip install robotpy-wpiutil
pip install git+https://github.com/ligerbots/dslogparser@updateTo2026
pip install -U google-genai
```

The `dslogparser` branch above targets current FRC DS log formats; if it changes, check the [ligerbots/dslogparser](https://github.com/ligerbots/dslogparser) repository for the appropriate branch or release.

## Configuration

The Gemini client reads **`GOOGLE_API_KEY`** from the environment (see `get_gemini_diagnosis` in `frc_log_analyzer.py`).

- **Linux / macOS** (example):

  ```bash
  export GOOGLE_API_KEY="your-key-here"
  ```

- **Windows**: set the variable in System Properties, or use a local env file (see [Windows batch helper](#windows-batch-helper)).

Do **not** commit API keys. This repo lists `.env.txt` in `.gitignore` for local secrets.

## Usage

### Basic

```bash
python frc_log_analyzer.py path/to/match.dslog
python frc_log_analyzer.py path/to/match.wpilog
```

### DS log with a Driver Station profile

Export a profile from the Driver Station log viewer (XML containing `GroupProfile` / `SeriesChildNode` entries with `pdp0`…`pdp23` names), then:

```bash
python frc_log_analyzer.py match.dslog --dslog-profile dslogprofiles22.xml
```

If the file contains multiple profiles, pick one by name (matches `<Name>` under `GroupProfile`):

```bash
python frc_log_analyzer.py match.dslog --dslog-profile dslogprofiles22.xml --profile-name "2026 Arakis"
```

### Command-line options

| Argument | Description |
| -------- | ----------- |
| `file` | Path to a `.dslog` or `.wpilog` file. |
| `--dslog-profile XML` | Optional. Only affects `.dslog` parsing; keys `pd_currents` by profile channel names. |
| `--profile-name NAME` | Optional. Selects which `GroupProfile` to use when the XML has several (default: first profile). |

### Output

- **stdout**: Parsed arguments are printed, then a JSON object (summary string, `events` array with timestamps, voltages, `pd_currents` as an object, `significant_loads`, etc.).
- If **`events`** is non-empty, the script calls **Gemini** and prints the model’s diagnosis (requires `GOOGLE_API_KEY` and network access). If there are no events, it prints `No issues detected.` instead.

To capture only JSON, you may need to edit `main()` or split stdout, since Gemini text follows the JSON block.

```bash
python frc_log_analyzer.py match.dslog > analysis.json
```

## Windows batch helper

`analyze.bat` runs the analyzer from the script directory and loads **`KEY=value`** lines from **`.env.txt`** (if present) into the environment for the Python process. Put **`GOOGLE_API_KEY=...`** in `.env.txt` so Gemini can run (see [Configuration](#configuration)).

It then invokes:

`python frc_log_analyzer.py <first argument> --dslog-profile .\dslogprofiles22.xml --profile-name="2026 Arakis"`

Example:

```bat
analyze.bat C:\logs\match.dslog
```

Adjust paths or profile name inside `analyze.bat` to match your machine. Remove any debug `echo` of secrets before sharing the batch file.
