package stealth

import (
	"fmt"
	"math/rand"
	"sync"
	"time"
)

var (
	once sync.Once

	// Generated values — stable for the lifetime of the process.
	gameWindowTitle string
	appWindowTitle  string
	modName         string
)

var windowTitlePool = []string{
	// Productivity / Office
	"Microsoft Teams",
	"Microsoft Word",
	"Microsoft Excel",
	"Microsoft PowerPoint",
	"Microsoft Outlook",
	"Microsoft OneNote",
	"Microsoft Access",
	"Microsoft Publisher",
	"Microsoft Visio",
	"Microsoft Project",
	"Google Docs",
	"Google Sheets",
	"Google Slides",
	"LibreOffice Writer",
	"LibreOffice Calc",
	"LibreOffice Impress",
	"Apache OpenOffice",
	"WPS Office",
	"Notion",
	"Obsidian",
	"Evernote",
	"Trello",
	"Asana",
	"Jira",
	"Monday.com",
	"ClickUp",
	"Todoist",

	// Communication
	"Slack",
	"Discord",
	"Zoom Meeting",
	"Zoom Workplace",
	"Skype",
	"Telegram Desktop",
	"Signal",
	"WhatsApp",
	"Microsoft Teams (work or school)",
	"Webex",
	"Google Meet",
	"GoTo Meeting",
	"RingCentral",
	"Lark",
	"Mattermost",
	"Rocket.Chat",
	"Element",

	// Browsers
	"Google Chrome",
	"Mozilla Firefox",
	"Microsoft Edge",
	"Opera GX",
	"Opera",
	"Brave",
	"Vivaldi",
	"Arc",
	"Safari",
	"Tor Browser",
	"Waterfox",
	"Pale Moon",

	// Development / Editors
	"Visual Studio Code",
	"Visual Studio 2022",
	"JetBrains IntelliJ IDEA",
	"JetBrains PyCharm",
	"JetBrains WebStorm",
	"JetBrains Rider",
	"JetBrains GoLand",
	"JetBrains CLion",
	"JetBrains DataGrip",
	"Sublime Text",
	"Notepad++",
	"Atom",
	"Vim",
	"Neovim",
	"Eclipse IDE",
	"NetBeans IDE",
	"Android Studio",
	"Xcode",
	"Arduino IDE",
	"Cursor",
	"Zed",
	"Fleet",

	// Terminals
	"Windows Terminal",
	"PowerShell",
	"Command Prompt",
	"Git Bash",
	"PuTTY",
	"MobaXterm",
	"WinSCP",
	"Termius",
	"Hyper",
	"Alacritty",
	"Warp",
	"Tabby",

	// System / Utilities
	"Windows Explorer",
	"File Explorer",
	"Task Manager",
	"Resource Monitor",
	"Event Viewer",
	"Registry Editor",
	"Device Manager",
	"Disk Management",
	"Control Panel",
	"Windows Settings",
	"Windows Security",
	"Windows Update",
	"System Information",
	"Performance Monitor",
	"Services",
	"Component Services",
	"Computer Management",
	"Sysinternals Process Explorer",
	"Sysinternals Process Monitor",
	"Autoruns",
	"WinRAR",
	"7-Zip",
	"PeaZip",
	"Everything",
	"PowerToys",
	"Snipping Tool",
	"Greenshot",
	"ShareX",
	"f.lux",
	"Rainmeter",

	// Media
	"Spotify",
	"Apple Music",
	"YouTube Music",
	"Deezer",
	"Tidal",
	"Foobar2000",
	"Winamp",
	"AIMP",
	"MusicBee",
	"VLC Media Player",
	"MPC-HC",
	"PotPlayer",
	"KMPlayer",
	"mpv",
	"Plex",
	"Kodi",
	"Jellyfin",
	"Audacity",
	"Adobe Premiere Pro",
	"DaVinci Resolve",
	"Handbrake",
	"FFmpeg",

	// Gaming Platforms
	"Steam",
	"Epic Games Launcher",
	"Battle.net",
	"GOG Galaxy",
	"EA App",
	"Ubisoft Connect",
	"Xbox",
	"GeForce Experience",
	"GeForce NOW",
	"Lutris",
	"Playnite",
	"GamePass",
	"Riot Client",
	"Rockstar Games Launcher",
	"Amazon Games",
	"itch.io",

	// Creative
	"Adobe Photoshop",
	"Adobe Illustrator",
	"Adobe Acrobat Reader",
	"Adobe After Effects",
	"Adobe Lightroom",
	"Adobe XD",
	"Figma",
	"Canva",
	"GIMP",
	"Inkscape",
	"Paint.NET",
	"Krita",
	"Blender",
	"SketchUp",
	"AutoCAD",
	"SolidWorks",
	"Fusion 360",

	// Streaming / Recording
	"OBS Studio",
	"Streamlabs",
	"XSplit Broadcaster",
	"NVIDIA ShadowPlay",
	"Medal.tv",
	"Action!",
	"Bandicam",

	// Cloud / Backup
	"Google Drive",
	"OneDrive",
	"Dropbox",
	"iCloud",
	"MEGA",
	"pCloud",
	"Backblaze",
	"Restic",

	// Security / VPN
	"Norton 360",
	"Malwarebytes",
	"Bitdefender",
	"Kaspersky",
	"Avast",
	"AVG",
	"ESET NOD32",
	"Windows Defender",
	"NordVPN",
	"ExpressVPN",
	"ProtonVPN",
	"Mullvad VPN",
	"Surfshark",
	"WireGuard",
	"OpenVPN",
	"Tailscale",

	// Remote Desktop / Virtualization
	"Remote Desktop Connection",
	"AnyDesk",
	"TeamViewer",
	"Parsec",
	"Chrome Remote Desktop",
	"VMware Workstation",
	"VirtualBox",
	"Hyper-V Manager",
	"Docker Desktop",
	"WSL",

	// Database / DevOps
	"pgAdmin",
	"MySQL Workbench",
	"MongoDB Compass",
	"Redis Insight",
	"DBeaver",
	"Azure Data Studio",
	"Postman",
	"Insomnia",
	"Grafana",
	"Lens",
	"Terraform",
	"Ansible",

	// Finance / Misc
	"QuickBooks",
	"Mint",
	"TurboTax",
	"Calculator",
	"Notepad",
	"WordPad",
	"Calendar",
	"Clock",
	"Weather",
	"Maps",
	"Photos",
	"Paint",
	"Xbox Game Bar",
}

// appTitlePool holds plausible names for the management webview window.
var appTitlePool = []string{
	"Settings",
	"Preferences",
	"Configuration",
	"Options",
	"System Tray",
	"Control Panel",
	"Dashboard",
	"Monitoring",
	"Resource Monitor",
	"Performance",
	"Properties",
	"System Configuration",
	"Setup Wizard",
	"Update Manager",
	"Task Scheduler",
	"Service Manager",
	"Account Settings",
	"Network Settings",
	"Display Settings",
	"Sound Settings",
	"Privacy Settings",
	"Security Center",
	"Backup Settings",
	"Storage Settings",
	"Device Settings",
	"Accessibility",
	"Notifications",
	"Language Settings",
	"Time & Date",
	"Power Options",
	"User Accounts",
	"Startup Manager",
	"Extension Manager",
	"Plugin Manager",
	"Add-on Manager",
	"Theme Settings",
	"Font Settings",
	"Proxy Settings",
	"Advanced Options",
	"General Settings",
}

// modNamePool holds short, innocuous mod folder names.
var modNamePool = []string{
	"data",
	"cache",
	"pref",
	"core",
	"base",
	"ext",
	"res",
	"lib",
	"sys",
	"user",
	"temp",
	"conf",
	"meta",
	"cfg",
	"run",
	"var",
	"opt",
	"pkg",
	"mod",
	"app",
	"env",
	"src",
	"bin",
	"log",
	"tmp",
	"loc",
	"net",
	"srv",
	"hub",
	"api",
}

func init() {
	initialize()
}

func initialize() {
	once.Do(func() {
		rng := rand.New(rand.NewSource(time.Now().UnixNano()))

		gameWindowTitle = windowTitlePool[rng.Intn(len(windowTitlePool))]
		appWindowTitle = appTitlePool[rng.Intn(len(appTitlePool))]
		modName = modNamePool[rng.Intn(len(modNamePool))] + fmt.Sprintf("%d", rng.Intn(100))
	})
}

// GameWindowTitle returns a stable randomized window title for the process lifetime.
func GameWindowTitle(pid int64, supervisorName, realm string) string {
	return fmt.Sprintf("%s (%d)", gameWindowTitle, pid%1000)
}

// AppWindowTitle returns a randomized title for the webview management window.
func AppWindowTitle() string {
	return appWindowTitle
}

// ModName returns a stable randomized mod folder name for the process lifetime.
func ModName() string {
	return modName
}
