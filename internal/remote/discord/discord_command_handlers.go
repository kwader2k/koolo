package discord

import (
	"fmt"
	"slices"
	"strings"
	"time"

	"github.com/bwmarrin/discordgo"
	"github.com/hectorgimenez/koolo/internal/bot"
)

func (b *Bot) supervisorExists(supervisor string) bool {
	supervisors := b.manager.AvailableSupervisors()
	return slices.Contains(supervisors, supervisor)
}

func (b *Bot) handleStartRequest(s *discordgo.Session, m *discordgo.MessageCreate) {

	// Start supervisor(s) provided as in the stop command
	words := strings.Fields(m.Content)

	if len(words) > 1 {
		// Iterate through the supervisors specified
		for _, supervisor := range words[1:] {

			if !b.supervisorExists(supervisor) {
				s.ChannelMessageSend(m.ChannelID, fmt.Sprintf("Supervisor '%s' not found.", supervisor))
				continue
			}

			// Attempt to start the specified supervisor
			b.manager.Start(supervisor, false, false)

			// Wait for the supervisor to start
			time.Sleep(1 * time.Second)

			// Send a confirmation message
			s.ChannelMessageSend(m.ChannelID, fmt.Sprintf("Supervisor '%s' has been started.", supervisor))
		}
	} else {
		// If no supervisors were specified, send a usage message
		s.ChannelMessageSend(m.ChannelID, "Usage: !start <supervisor1> [supervisor2] ...")
	}
}

func (b *Bot) handleStopRequest(s *discordgo.Session, m *discordgo.MessageCreate) {

	// Split the message content into words
	words := strings.Fields(m.Content)

	// Check if there are any supervisors specified after "!stop"
	if len(words) > 1 {
		// Iterate through the supervisors specified
		for _, supervisor := range words[1:] {

			if !b.supervisorExists(supervisor) {
				s.ChannelMessageSend(m.ChannelID, fmt.Sprintf("Supervisor '%s' not found.", supervisor))
				continue
			}

			// Check if the supervisor is running
			if b.manager.Status(supervisor).SupervisorStatus == bot.NotStarted || b.manager.Status(supervisor).SupervisorStatus == "" {
				s.ChannelMessageSend(m.ChannelID, fmt.Sprintf("Supervisor '%s' is not running.", supervisor))
				continue
			}

			// Attempt to stop the specified supervisor
			b.manager.Stop(supervisor)

			// Wait for the supervisor to stop
			time.Sleep(1 * time.Second)

			// Send a confirmation message
			s.ChannelMessageSend(m.ChannelID, fmt.Sprintf("Supervisor '%s' has been stopped.", supervisor))
		}
	} else {
		// If no supervisors were specified, send a usage message
		s.ChannelMessageSend(m.ChannelID, "Usage: !stop <supervisor1> [supervisor2] ...")
	}
}

func (b *Bot) handleStatusRequest(s *discordgo.Session, m *discordgo.MessageCreate) {

	// Split the message content into words
	words := strings.Fields(m.Content)

	if len(words) > 1 {
		for _, supervisor := range words[1:] {
			if !b.supervisorExists(supervisor) {
				s.ChannelMessageSend(m.ChannelID, fmt.Sprintf("Supervisor '%s' not found.", supervisor))
				continue
			}

			status := b.manager.Status(supervisor)
			if status.SupervisorStatus == bot.NotStarted || status.SupervisorStatus == "" {
				s.ChannelMessageSend(m.ChannelID, fmt.Sprintf("Supervisor '%s' is offline.", supervisor))
				continue
			}

			s.ChannelMessageSend(m.ChannelID, fmt.Sprintf("Supervisor '%s' is %s", supervisor, status.SupervisorStatus))
		}
	} else {
		// If no supervisors were specified, send a usage message
		s.ChannelMessageSend(m.ChannelID, "Usage: !status <supervisor1> [supervisor2] ...")
	}
}

func (b *Bot) handleStatsRequest(s *discordgo.Session, m *discordgo.MessageCreate) {

	// Split the message content into words
	words := strings.Fields(m.Content)

	// Check if there are any supervisors specified after "!stats"
	if len(words) > 1 {
		// Iterate through the supervisors specified
		for _, supervisor := range words[1:] {

			if !b.supervisorExists(supervisor) {
				s.ChannelMessageSend(m.ChannelID, fmt.Sprintf("Supervisor '%s' not found.", supervisor))
				continue
			}

			// Fix for the status not being started
			supStatus := string(b.manager.Status(supervisor).SupervisorStatus)
			if supStatus == string(bot.NotStarted) || supStatus == "" {
				supStatus = "Offline"
			}
			// Create the embed
			embed := &discordgo.MessageEmbed{
				Title: fmt.Sprintf("Stats for %s", supervisor),
				Fields: []*discordgo.MessageEmbedField{
					{
						Name:   "Status",
						Value:  supStatus,
						Inline: true,
					},
					{
						Name:   "Uptime",
						Value:  time.Since(b.manager.Status(supervisor).StartedAt).String(),
						Inline: true,
					},

					// Runs data

					{
						Name:   "Games",
						Value:  fmt.Sprintf("%d", b.manager.GetSupervisorStats(supervisor).TotalGames()),
						Inline: true,
					},
					{
						Name:   "Drops",
						Value:  fmt.Sprintf("%d", len(b.manager.GetSupervisorStats(supervisor).Drops)),
						Inline: true,
					},
					{
						Name:   "Deaths",
						Value:  fmt.Sprintf("%d", b.manager.GetSupervisorStats(supervisor).TotalDeaths()),
						Inline: true,
					},
					{
						Name:   "Chickens",
						Value:  fmt.Sprintf("%d", b.manager.GetSupervisorStats(supervisor).TotalChickens()),
						Inline: true,
					},
					{
						Name:   "Errors",
						Value:  fmt.Sprintf("%d", b.manager.GetSupervisorStats(supervisor).TotalErrors()),
						Inline: true,
					},
				},
			}

			// Send the embed to the channel
			s.ChannelMessageSendEmbed(m.ChannelID, embed)
		}
	} else {
		// If no supervisors were specified, send a usage message
		s.ChannelMessageSend(m.ChannelID, "Usage: !stats <supervisor1> [supervisor2] ...")
	}
}

func (b *Bot) handleListRequest(s *discordgo.Session, m *discordgo.MessageCreate) {
	supervisors := b.manager.AvailableSupervisors()

	if len(supervisors) == 0 {
		s.ChannelMessageSend(m.ChannelID, "No supervisors available.")
		return
	}

	var fields []*discordgo.MessageEmbedField

	for _, supervisor := range supervisors {
		status := b.manager.Status(supervisor)
		var statusText, uptimeText string

		if status.SupervisorStatus == bot.NotStarted || status.SupervisorStatus == "" {
			statusText = "❌ Offline"
			uptimeText = "-"
		} else {
			statusText = fmt.Sprintf("✅ %s", status.SupervisorStatus)
			uptime := time.Since(status.StartedAt)
			if uptime < time.Minute {
				uptimeText = fmt.Sprintf("%ds", int(uptime.Seconds()))
			} else if uptime < time.Hour {
				uptimeText = fmt.Sprintf("%dm", int(uptime.Minutes()))
			} else {
				uptimeText = fmt.Sprintf("%dh %dm", int(uptime.Hours()), int(uptime.Minutes())%60)
			}
		}

		fields = append(fields, &discordgo.MessageEmbedField{
			Name:   supervisor,
			Value:  fmt.Sprintf("Status: %s\nUptime: %s", statusText, uptimeText),
			Inline: true,
		})
	}

	embed := &discordgo.MessageEmbed{
		Title:  "📋 Available Supervisors",
		Fields: fields,
		Color:  0x5865F2, // Discord blurple
	}

	s.ChannelMessageSendEmbed(m.ChannelID, embed)
}

func (b *Bot) handleHelpRequest(s *discordgo.Session, m *discordgo.MessageCreate) {
	embed := &discordgo.MessageEmbed{
		Title:       "🤖 Koolo Discord Bot Commands",
		Description: "Control and monitor your Diablo II bot supervisors",
		Color:       0x5865F2,
		Fields: []*discordgo.MessageEmbedField{
			{
				Name:   "!list",
				Value:  "Show all available supervisors with their status and uptime",
				Inline: false,
			},
			{
				Name:   "!start <supervisor1> [supervisor2] ...",
				Value:  "Start one or more supervisors\nExample: `!start Koza Ovca`",
				Inline: false,
			},
			{
				Name:   "!stop <supervisor1> [supervisor2] ...",
				Value:  "Stop one or more supervisors\nExample: `!stop Koza`",
				Inline: false,
			},
			{
				Name:   "!status <supervisor1> [supervisor2] ...",
				Value:  "Check the current status of supervisors\nExample: `!status Koza Ovca`",
				Inline: false,
			},
			{
				Name:   "!stats <supervisor1> [supervisor2] ...",
				Value:  "Get detailed statistics for supervisors\nExample: `!stats Koza`",
				Inline: false,
			},
			{
				Name:   "!drops <supervisor> [count]",
				Value:  "Show recent drops for a supervisor\nExample: `!drops Koza 10`\nDefault count: 5",
				Inline: false,
			},
			{
				Name:   "!help",
				Value:  "Show this help message",
				Inline: false,
			},
		},
		Footer: &discordgo.MessageEmbedFooter{
			Text: "💡 Tip: You can control multiple supervisors at once with most commands",
		},
	}

	s.ChannelMessageSendEmbed(m.ChannelID, embed)
}

func (b *Bot) handleDropsRequest(s *discordgo.Session, m *discordgo.MessageCreate) {
	words := strings.Fields(m.Content)

	if len(words) < 2 {
		s.ChannelMessageSend(m.ChannelID, "Usage: !drops <supervisor> [count]\nExample: `!drops Koza 10`")
		return
	}

	supervisor := words[1]

	if !b.supervisorExists(supervisor) {
		s.ChannelMessageSend(m.ChannelID, fmt.Sprintf("Supervisor '%s' not found.", supervisor))
		return
	}

	// Default count is 5, max is 20
	count := 5
	if len(words) > 2 {
		fmt.Sscanf(words[2], "%d", &count)
		if count < 1 {
			count = 5
		}
		if count > 20 {
			count = 20
		}
	}

	stats := b.manager.GetSupervisorStats(supervisor)
	drops := stats.Drops

	if len(drops) == 0 {
		s.ChannelMessageSend(m.ChannelID, fmt.Sprintf("No drops recorded for '%s' yet.", supervisor))
		return
	}

	// Get the last N drops (reverse order to show most recent first)
	startIdx := len(drops) - count
	if startIdx < 0 {
		startIdx = 0
	}
	recentDrops := drops[startIdx:]

	// Build the embed
	var description strings.Builder

	// Reverse to show newest first
	for i := len(recentDrops) - 1; i >= 0; i-- {
		drop := recentDrops[i]
		item := drop.Item

		// Determine emoji based on quality
		emoji := "⚪"
		quality := strings.ToLower(item.Quality.ToString())

		switch quality {
		case "unique":
			emoji = "🟠"
		case "set":
			emoji = "🟢"
		case "rare":
			emoji = "🟡"
		case "magic":
			emoji = "🔵"
		case "superior":
			emoji = "⚪"
		}

		// Check if it's a rune based on item name
		if strings.Contains(strings.ToLower(string(item.Name)), "rune") {
			emoji = "🟣"
		}

		// Format the drop entry
		itemName := string(item.Name)
		if item.Quality.ToString() != "" && item.Quality.ToString() != "Normal" {
			itemName = fmt.Sprintf("%s %s", item.Quality.ToString(), string(item.Name))
		}

		description.WriteString(fmt.Sprintf("%s **%s**", emoji, itemName))

		// Add base item description if available and different from name
		desc := item.Desc()
		if desc.Name != "" && desc.Name != string(item.Name) {
			description.WriteString(fmt.Sprintf(" (%s)", desc.Name))
		}

		description.WriteString("\n")
	}

	embed := &discordgo.MessageEmbed{
		Title:       fmt.Sprintf("💎 Recent Drops for %s", supervisor),
		Description: description.String(),
		Color:       0xFFD700, // Gold color
		Footer: &discordgo.MessageEmbedFooter{
			Text: fmt.Sprintf("Showing last %d of %d total drops", len(recentDrops), len(drops)),
		},
	}

	s.ChannelMessageSendEmbed(m.ChannelID, embed)
}
