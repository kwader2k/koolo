package discord

import (
	"context"
	"fmt"
	"log/slog"
	"slices"
	"strings"

	"github.com/bwmarrin/discordgo"
	"github.com/hectorgimenez/koolo/internal/bot"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/utils/obs"
	"github.com/hectorgimenez/koolo/internal/event"
)

type Bot struct {
	discordSession *discordgo.Session
	channelID      string
	itemChannelID  string
	manager        *bot.SupervisorManager
	useWebhook     bool
	webhookClient  *webhookClient
	itemWebhook    *webhookClient
}

func NewBot(token, channelID, itemChannelID string, manager *bot.SupervisorManager, useWebhook bool, webhookURL, itemWebhookURL string) (*Bot, error) {
	botInstance := &Bot{
		channelID:     channelID,
		itemChannelID: strings.TrimSpace(itemChannelID),
		manager:       manager,
		useWebhook:    useWebhook,
		webhookClient: nil,
		itemWebhook:   nil,
	}

	if useWebhook {
		if webhookURL == "" {
			return nil, fmt.Errorf("webhook URL is required when using webhook mode")
		}
		botInstance.webhookClient = newWebhookClient(webhookURL)
		if strings.TrimSpace(itemWebhookURL) != "" {
			botInstance.itemWebhook = newWebhookClient(itemWebhookURL)
		}
		return botInstance, nil
	}

	dg, err := discordgo.New("Bot " + token)
	if err != nil {
		return nil, fmt.Errorf("error creating Discord session: %w", err)
	}

	botInstance.discordSession = dg

	return botInstance, nil
}

func (b *Bot) Start(ctx context.Context) error {
	if b.useWebhook {
		<-ctx.Done()
		return nil
	}

	//b.discordSession.Debug = true
	b.discordSession.AddHandler(b.onMessageCreated)
	// Add MESSAGE_CONTENT intent to read message content (required by Discord)
	b.discordSession.Identify.Intents = discordgo.IntentsGuildMessages | discordgo.IntentMessageContent
	err := b.discordSession.Open()
	if err != nil {
		return fmt.Errorf("error opening connection: %w", err)
	}

	// Wait until context is finished
	<-ctx.Done()

	return b.discordSession.Close()
}

func (b *Bot) onMessageCreated(s *discordgo.Session, m *discordgo.MessageCreate) {
	// Ignore messages from the bot itself
	if m.Author.ID == s.State.User.ID {
		return
	}

	// Debug: Log all received messages (uncomment to debug)
	// fmt.Printf("[Discord] Message from %s (ID: %s): %s\n", m.Author.Username, m.Author.ID, m.Content)

	// Check if the message is from a bot admin
	if !slices.Contains(config.Koolo.Discord.BotAdmins, m.Author.ID) {
		// Debug: Uncomment to see who is trying to use commands
		// fmt.Printf("[Discord] User %s (ID: %s) not in admin list. Admins: %v\n", m.Author.Username, m.Author.ID, config.Koolo.Discord.BotAdmins)
		return
	}

	// Only process messages that start with !
	if !strings.HasPrefix(m.Content, "!") {
		return
	}

	prefix := strings.Split(m.Content, " ")[0]
	switch prefix {
	case "!start":
		b.handleStartRequest(s, m)
	case "!stop":
		b.handleStopRequest(s, m)
	case "!stats":
		b.handleStatsRequest(s, m)
	case "!status":
		b.handleStatusRequest(s, m)
	case "!list":
		b.handleListRequest(s, m)
	case "!help":
		b.handleHelpRequest(s, m)
	case "!drops":
		b.handleDropsRequest(s, m)
	case "!obscapture":
    if !config.Koolo.OBS.Enabled {
        s.ChannelMessageSend(m.ChannelID, "OBS recording is not enabled.")
        break
    }
    filePath := obs.SaveReplay(slog.Default(), "test")
    if filePath != "" {
        event.Send(event.ReplayClip(event.Text("test", "Replay clip saved"), filePath))
        s.ChannelMessageSend(m.ChannelID, "OBS replay save triggered, uploading clip...")
    } else {
        s.ChannelMessageSend(m.ChannelID, "OBS replay save triggered but no file found")
    }
	default:
		// Unknown command - send help
		s.ChannelMessageSend(m.ChannelID, fmt.Sprintf("Unknown command: `%s`. Type `!help` for available commands.", prefix))
	}

}
