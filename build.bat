@echo off
setlocal enabledelayedexpansion

echo Start building Koolo
echo Cleaning up previous artifacts...
::if exist build rmdir /s /q build > NUL || goto :error

:: Generate unique identifiers
for /f "delims=" %%a in ('powershell "[guid]::NewGuid().ToString()"') do set "BUILD_ID=%%a"
for /f "delims=" %%b in ('powershell "Get-Date -Format 'o'"') do set "BUILD_TIME=%%b"

echo Building Koolo binary...
if "%1"=="" (set VERSION=dev) else (set VERSION=%1)
garble -literals -tiny -seed=random build -a -trimpath -tags static --ldflags "-s -w -H windowsgui -X 'main.buildID=%BUILD_ID%' -X 'main.buildTime=%BUILD_TIME%' -X 'github.com/hectorgimenez/koolo/internal/config.Version=%VERSION%'" -o "build\%BUILD_ID%.exe" ./cmd/koolo > NUL || goto :error

echo Copying assets...
mkdir build\config > NUL || goto :error
copy config\koolo.yaml.dist build\config\koolo.yaml  > NUL || goto :error
copy config\Settings.json build\config\Settings.json  > NUL || goto :error
xcopy /q /E /I /y config\template build\config\template  > NUL || goto :error
xcopy /q /E /I /y tools build\tools > NUL || goto :error
xcopy /q /y README.md build > NUL || goto :error

echo Done! Artifacts are in build directory.

:error
if %errorlevel% neq 0 (
    echo Error occurred #%errorlevel%.
    exit /b %errorlevel%
)
