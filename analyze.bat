@echo off
cd /d "%~dp0"
if exist ".env.txt" (
  for /f "usebackq tokens=1* delims==" %%a in (".env.txt") do set "%%a=%%b"
)
echo %GOOGLE_API_KEY%
python .\frc_log_analyzer.py %1 --dslog-profile .\dslogprofiles22.xml --profile-name="2026 Arakis"
