@echo off
setlocal enabledelayedexpansion

:: Preserve UI and critical packages
set GOGARBLE=!github.com/hectorgimenez/koolo/internal/server*,!github.com/hectorgimenez/koolo/internal/event*,!github.com/inkeliz/gowebview*

:: Required versions
set REQUIRED_GO_VERSION=1.24
set REQUIRED_GARBLE_VERSION=0.14.2
set GIT_AVAILABLE=0

:: Upstream repository (override here for testing)
set "UPSTREAM_OWNER=kwader2k"
set "UPSTREAM_REPO=Koolo"
set "UPSTREAM_URL=https://github.com/%UPSTREAM_OWNER%/%UPSTREAM_REPO%.git"

:: Change to the script's directory
cd /d "%~dp0"

:: Use a static build folder to avoid temp paths being flagged by AV
set "STATIC_BUILD_DIR=%cd%\build\tmp"
if not exist "%STATIC_BUILD_DIR%" mkdir "%STATIC_BUILD_DIR%"
set "GOCACHE=%STATIC_BUILD_DIR%\gocache"
set "GOTMPDIR=%STATIC_BUILD_DIR%"
call :print_info "Using static build folder: %STATIC_BUILD_DIR%"

call :print_header "Starting Koolo Resurrected Build Process"

:: Check for Git installation
call :check_git_installation

:: Check for Go installation
call :check_go_installation
if !errorlevel! neq 0 call :pause_and_exit !errorlevel!

:: Check for Garble installation
call :check_garble_installation
if !errorlevel! neq 0 call :pause_and_exit !errorlevel!

:: Ensure git repository exists (for updater support)
if "%GIT_AVAILABLE%"=="1" (
    call :ensure_git_repo
    if !errorlevel! neq 0 call :pause_and_exit !errorlevel!
) else (
    call :print_warning "Git not available; skipping repository setup."
)

:: Main script execution
call :main %*
if !errorlevel! neq 0 (
    call :print_error "Build process failed with error code !errorlevel!"
    call :pause_and_exit !errorlevel!
)
echo.
powershell -Command "Write-Host 'Press any key to exit...' -ForegroundColor Yellow"
pause > nul
exit /b 0

:check_git_installation
call :print_info "Checking if Git is installed"
where git >nul 2>&1
if %errorlevel% neq 0 (
    call :print_warning "Git is not installed or not in the system PATH."
    call :print_info "You can download Git from https://git-scm.com/downloads"
    call :print_info "Updater features will be skipped without Git."
    set GIT_AVAILABLE=0
    goto :eof
)
set GIT_AVAILABLE=1
call :print_success "Git is installed."
goto :eof

:check_go_installation
call :print_info "Checking if Go is installed"
where go >nul 2>&1
if %errorlevel% neq 0 (
    call :print_error "Go is not installed or not in the system PATH."
    call :print_info "You can download Go from https://golang.org/dl/"
    call :get_user_input "Do you want to attempt automatic installation using Chocolatey? (Y/N) " install_go
    if /i "!install_go!"=="Y" (
        call :install_go_with_chocolatey
    ) else (
        call :print_info "Please install Go manually and run this script again."
        call :pause_and_exit 1
    )
) else (
    :: Extract Go version and check if it matches required version
    for /f "tokens=3" %%v in ('go version') do set go_version_full=%%v
    set go_version=!go_version_full:go=!

    :: Extract major.minor version (e.g., 1.24 from 1.24.3)
    for /f "tokens=1,2 delims=." %%a in ("!go_version!") do set go_major_minor=%%a.%%b

    if "!go_major_minor!"=="%REQUIRED_GO_VERSION%" (
        call :print_success "Go version !go_version! is installed."
    ) else (
        call :print_warning "You are currently using Go version !go_version!"
        call :print_warning "The recommended version for this build is Go %REQUIRED_GO_VERSION%"
        call :print_info "Please consider downloading Go %REQUIRED_GO_VERSION% from https://golang.org/dl/"
        call :print_info "Using a different Go version may cause compatibility issues."
        echo.
        call :get_user_input "Do you want to continue anyway? (Y/N) " continue_with_go
        if /i "!continue_with_go!" neq "Y" (
            call :print_info "Build process cancelled. Please install Go %REQUIRED_GO_VERSION% and try again."
            call :pause_and_exit 1
        )
    )
)
if !errorlevel! neq 0 call :pause_and_exit !errorlevel!
goto :eof

:check_garble_installation
call :print_info "Checking if Garble is installed"
where garble >nul 2>&1
if %errorlevel% neq 0 (
    call :print_error "Garble is not installed or not in the system PATH."
    call :print_info "You can install Garble using: go install mvdan.cc/garble@v%REQUIRED_GARBLE_VERSION%"
    call :get_user_input "Do you want to attempt automatic installation? (Y/N) " install_garble
    if /i "!install_garble!"=="Y" (
        call :install_garble
    ) else (
        call :print_info "Please install Garble manually and run this script again."
        call :pause_and_exit 1
    )
) else (
    :: Check Garble version
    for /f "tokens=1,2" %%a in ('garble version 2^>^&1') do (
        if "%%a"=="mvdan.cc/garble" set garble_version_with_v=%%b
    )

    :: Remove the 'v' prefix
    set garble_version=!garble_version_with_v:~1!

    :: Extract exact version
    for /f "tokens=1,2,3 delims=." %%a in ("!garble_version!") do set garble_major_minor=%%a.%%b.%%c

    if "!garble_major_minor!"=="%REQUIRED_GARBLE_VERSION%" (
        call :print_success "Garble version !garble_version! is installed."
    ) else (
        call :print_warning "You are currently using Garble version !garble_version!"
        call :print_warning "The recommended version for this build is Garble %REQUIRED_GARBLE_VERSION%"
        call :print_info "Please consider installing Garble %REQUIRED_GARBLE_VERSION% using:"
        call :print_info "go install mvdan.cc/garble@v%REQUIRED_GARBLE_VERSION%"
        call :print_info "Using a different Garble version may cause build issues."
        echo.
        call :get_user_input "Do you want to continue anyway? (Y/N) " continue_with_garble
        if /i "!continue_with_garble!" neq "Y" (
            call :print_info "Build process cancelled. Please install Garble %REQUIRED_GARBLE_VERSION% and try again."
            call :pause_and_exit 1
        )
    )
)
if !errorlevel! neq 0 call :pause_and_exit !errorlevel!
goto :eof

:install_garble
call :print_step "Attempting to install Garble %REQUIRED_GARBLE_VERSION%..."
go install mvdan.cc/garble@v%REQUIRED_GARBLE_VERSION% >nul 2>&1
where garble >nul 2>&1
if %errorlevel% neq 0 (
    call :print_error "Failed to install Garble. Please install it manually."
    call :print_info "Run: go install mvdan.cc/garble@v%REQUIRED_GARBLE_VERSION%"
    call :pause_and_exit 1
) else (
    call :print_success "Garble %REQUIRED_GARBLE_VERSION% has been successfully installed."
)
goto :eof

:ensure_git_repo
if "%GIT_AVAILABLE%"=="0" (
    call :print_info "Skipping git repository initialization (git not available)."
    goto :eof
)
if exist ".git" (
    call :print_info "Git repository already exists."
    goto :eof
)
call :print_header "Initializing Git Repository"
call :print_info "No .git directory found. Setting up repository for updater support..."
git init >nul 2>&1
if !errorlevel! neq 0 (
    call :print_error "Failed to initialize git repository"
    exit /b 1
)
git remote add origin "!UPSTREAM_URL!" >nul 2>&1
call :print_step "Fetching upstream repository (this may take a moment)..."
git fetch origin main >nul 2>&1
if !errorlevel! neq 0 (
    call :print_error "Failed to fetch from upstream repository"
    call :print_info "Please check your internet connection and try again."
    exit /b 1
)
git checkout -b main >nul 2>&1
:: Use .commit file from release if available; do not reset when missing or mismatched
if exist ".commit" (
    set /p RESET_TARGET=<.commit
    call :print_info "Using commit from .commit file: !RESET_TARGET:~0,7!"
    git reset !RESET_TARGET! >nul 2>&1
    if !errorlevel! neq 0 (
        call :print_warning "Commit from .commit not found in history; leaving HEAD unset."
    )
) else (
    call :print_info "No .commit file found; leaving HEAD unset."
)

call :print_success "Git repository initialized successfully."
goto :eof

:install_go_with_chocolatey
call :print_step "Attempting to install Go using Chocolatey..."
where choco >nul 2>&1
if %errorlevel% neq 0 (
    call :print_error "Chocolatey is not installed. Please install Go manually."
    call :print_info "You can install Chocolatey from https://chocolatey.org/install"
    call :pause_and_exit 1
)
powershell -Command "Start-Process powershell -Verb runAs -ArgumentList 'choco install golang -y' -Wait"
where go >nul 2>&1
if %errorlevel% neq 0 (
    call :print_error "Failed to install Go. Please install it manually."
    call :pause_and_exit 1
) else (
    call :print_success "Go has been successfully installed."
)
goto :eof

:main
:: Initial validation checks
call :validate_environment
if !errorlevel! neq 0 call :pause_and_exit !errorlevel!

:: Build Koolo binary with Garble
call :print_header "Building Koolo Binary"
if "%1"=="" (set VERSION=dev) else (set VERSION=%1)
call :print_info "Building %VERSION%"
:: Generate unique build identifiers
for /f "delims=" %%a in ('powershell -Command "[guid]::NewGuid().ToString()"') do set "BUILD_ID=%%a"
for /f "delims=" %%b in ('powershell -Command "Get-Date -Format 'o'"') do set "BUILD_TIME=%%b"

:: Extract commit metadata from git for updater version tracking
set "COMMIT_HASH="
set "COMMIT_TIME="
if "%GIT_AVAILABLE%"=="1" (
    for /f "delims=" %%h in ('git rev-parse HEAD 2^>nul') do set "COMMIT_HASH=%%h"
    for /f "delims=" %%t in ('git show -s --format^=%%cI HEAD 2^>nul') do set "COMMIT_TIME=%%t"
)
if not "!COMMIT_HASH!"=="" (
    call :print_info "Commit: !COMMIT_HASH:~0,7!"
) else if "%GIT_AVAILABLE%"=="1" (
    call :print_warning "Could not determine commit hash. Updater version tracking may not work."
) else (
    call :print_info "Git not available; skipping commit metadata."
)

:: Set the expected output executable path
set "OUTPUT_EXE=build\%BUILD_ID%.exe"

:: Build ldflags with commit metadata
set "LDFLAGS=-s -w -H windowsgui -X 'main.buildID=%BUILD_ID%' -X 'main.buildTime=%BUILD_TIME%' -X 'github.com/hectorgimenez/koolo/internal/config.Version=%VERSION%'"
if not "!COMMIT_HASH!"=="" set "LDFLAGS=!LDFLAGS! -X 'github.com/hectorgimenez/koolo/internal/updater.buildCommitHash=!COMMIT_HASH!'"
if not "!COMMIT_TIME!"=="" set "LDFLAGS=!LDFLAGS! -X 'github.com/hectorgimenez/koolo/internal/updater.buildCommitTime=!COMMIT_TIME!'"

:: Build an obfuscated Koolo binary
call :print_step "Compiling Obfuscated Koolo executable"
(
    garble -literals=false -seed=random build -a -trimpath -tags static --ldflags "!LDFLAGS!" -o "%OUTPUT_EXE%" ./cmd/koolo 2>&1
) > garble.log
set "GARBLE_EXIT_CODE=!errorlevel!"

if !GARBLE_EXIT_CODE! neq 0 (
    call :print_error "Garble build failed. These logs may be useful:"
    for /f "usebackq delims=" %%l in (`type garble.log`) do (
        call :print_error "%%l"
    )
) else (
    :: Capture and style seed information
    for /f "tokens=4" %%s in ('findstr /C:"-seed chosen at random:" garble.log') do (
        call :print_step "Obfuscation seed: !BUILD_ID!"
    )
)
del garble.log
if exist "%STATIC_BUILD_DIR%" (
    call :print_step "Cleaning up temporary build folder"
    rmdir /s /q "%STATIC_BUILD_DIR%"
)

:: Check if the executable was actually created
if exist "%OUTPUT_EXE%" (
    call :print_success "Successfully built obfuscated executable: %BUILD_ID%.exe"
) else (
    call :print_error "Failed to build Koolo binary - executable was not created"
    echo.
    call :print_warning "Please verify the following:"
    call :print_info "- Are you using the correct Go version? (Recommended: %REQUIRED_GO_VERSION%)"
    call :print_info "- Are you using the correct Garble version? (Recommended: %REQUIRED_GARBLE_VERSION%)"
    call :print_info "- Have you added your Koolo folder to the exclusion list in your Anti-Virus software?"
    call :print_info "- Have you tried temporarily disabling your Anti-Virus completely?"
    echo.
    call :print_info "Anti-Virus software can sometimes interfere with the compilation process."
    call :print_info "If the issue persists, please check the compilation errors above."
    call :pause_and_exit 1
)

:: Handle tools folder first
call :print_header "Handling Tools"
if exist build\tools (
    call :print_step "Removing existing tools folder"
    rmdir /s /q build\tools
    if exist build\tools (
        call :print_error "Failed to delete tools folder"
        call :check_folder_permissions "build\tools"
        call :pause_and_exit 1
    )
)
call :print_step "Copying tools folder"
xcopy /q /E /I /y tools build\tools > nul
if !errorlevel! neq 0 (
    call :print_error "Failed to copy tools folder"
    call :check_folder_permissions "tools"
    call :check_folder_permissions "build"
    call :pause_and_exit 1
)
call :print_success "Tools folder successfully copied"

:: Handle Settings.json
call :print_header "Handling Configuration Files"
if not exist build\config mkdir build\config
if exist build\config\Settings.json (
    call :print_step "Checking Settings.json"
    call :print_info "Settings.json found in %cd%\build\config"
    call :get_user_input "    Do you want to replace it? (Y/N) " replace_settings
    if /i "!replace_settings!"=="Y" (
        call :print_step "Replacing Settings.json"
        copy /y config\Settings.json build\config\Settings.json > nul
        if !errorlevel! equ 0 (
            call :print_success "Settings.json successfully replaced"
        ) else (
            call :print_error "Failed to copy Settings.json"
            call :pause_and_exit 1
        )
    ) else (
        call :print_info "Keeping existing Settings.json"
    )
) else (
    call :print_info "No existing Settings.json found in %cd%\build\config"
    call :print_step "Copying Settings.json"
    copy /y config\Settings.json build\config\Settings.json > nul
    if !errorlevel! neq 0 (
        call :print_error "Failed to copy Settings.json"
        call :pause_and_exit 1
    )
    call :print_success "Settings.json successfully copied"
)

:: Handle koolo.yaml
if not exist build\config\koolo.yaml (
    call :print_step "Copying koolo.yaml.dist"
    copy config\koolo.yaml.dist build\config\koolo.yaml > nul
    if !errorlevel! neq 0 (
        call :print_error "Failed to copy koolo.yaml.dist"
        call :pause_and_exit 1
    )
    call :print_success "koolo.yaml.dist successfully copied"
) else (
    call :print_info "koolo.yaml already exists in build\config, skipping copy"
)

:: Copy template folder
call :print_step "Copying template folder"
if exist build\config\template rmdir /s /q build\config\template
xcopy /q /E /I /y config\template build\config\template > nul
if !errorlevel! neq 0 (
    call :print_error "Failed to copy template folder"
    call :pause_and_exit 1
)
call :print_success "Template folder successfully copied"

:: Copy README
call :print_step "Copying README.md"
copy README.md build > nul
if !errorlevel! neq 0 (
    call :print_error "Failed to copy README.md"
    call :pause_and_exit 1
)
call :print_success "README.md successfully copied"

call :print_header "Build Process Completed"
call :print_success "Artifacts are in the build directory"
goto :eof

:: Function to pause and exit with error code
:pause_and_exit
echo.
powershell -Command "Write-Host 'Press any key to exit...' -ForegroundColor Yellow"
pause > nul
exit %1

:: Function to get user input
:get_user_input
setlocal enabledelayedexpansion
call :print_prompt "%~1"
set /p "user_input="
endlocal & set "%~2=%user_input%"
goto :eof

:: Function to print a colored prompt
:print_prompt
powershell -Command "Write-Host '%~1' -ForegroundColor Yellow -NoNewline"
goto :eof

:: Function to print a header
:print_header
echo.
powershell -Command "Write-Host '=== %~1 ===' -ForegroundColor Magenta"
echo.
goto :eof

:: Function to print a step
:print_step
powershell -Command "Write-Host '  - %~1' -ForegroundColor Cyan"
goto :eof

:: Function to print a success message
:print_success
powershell -Command "Write-Host '    SUCCESS: %~1' -ForegroundColor Green"
goto :eof

:: Function to print an error message
:print_error
powershell -Command "Write-Host '    ERROR: %~1' -ForegroundColor Red"
goto :eof

:: Function to print an info message
:print_info
powershell -Command "Write-Host '    INFO: %~1' -ForegroundColor Yellow"
goto :eof

:: Function to print a warning message
:print_warning
powershell -Command "Write-Host '    WARNING: %~1' -ForegroundColor Yellow"
goto :eof

:: Function to check folder permissions
:check_folder_permissions
dir "%~1\*" >nul 2>&1
if !errorlevel! neq 0 (
    call :print_error "Cannot access directory: %~1"
) else (
    call :print_info "Directory %~1 is accessible"
)
goto :eof

:: Function to validate environment
:validate_environment
call :print_header "Validating Environment"

:: Check for required source files and folders
if not exist config (
    call :print_error "Config directory is missing"
    exit /b 1
)

if not exist config\koolo.yaml.dist (
    call :print_error "koolo.yaml.dist is missing from config directory"
    exit /b 1
)

if not exist config\Settings.json (
    call :print_error "Settings.json is missing from config directory"
    exit /b 1
)

if not exist tools (
    call :print_error "Tools directory is missing"
    exit /b 1
)

:: Check for required tools
if not exist tools\handle64.exe (
    call :print_error "handle64.exe is missing from tools directory"
    exit /b 1
)

if not exist tools\koolo-map.exe (
    call :print_error "koolo-map.exe is missing from tools directory"
    exit /b 1
)

:: Check for required build dependencies
call :print_step "Checking build dependencies"
go version >nul 2>&1
if !errorlevel! neq 0 (
    call :print_error "Go is not installed or not in PATH"
    exit /b 1
)

:: Verify write permissions in current directory
call :print_step "Checking write permissions"
echo. > test_write.tmp 2>nul
if !errorlevel! neq 0 (
    call :print_error "No write permissions in current directory"
    exit /b 1
)
del test_write.tmp >nul 2>&1

call :print_success "Environment validation completed"
goto :eof
