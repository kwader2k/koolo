package discord

import (
	"context"
	"fmt"
	"slices"
	"strings"

	"github.com/bwmarrin/discordgo"
	"github.com/hectorgimenez/koolo/internal/bot"
	"github.com/hectorgimenez/koolo/internal/config"
)

type Bot struct {
	discordSession *discordgo.Session
	channelID      string
	manager        *bot.SupervisorManager
}

func NewBot(token, channelID string, manager *bot.SupervisorManager) (*Bot, error) {
	dg, err := discordgo.New("Bot " + token)
	if err != nil {
		return nil, fmt.Errorf("error creating Discord session: %w", err)
	}

	return &Bot{
		discordSession: dg,
		channelID:      channelID,
		manager:        manager,
	}, nil
}

func (b *Bot) Start(ctx context.Context) error {
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
	default:
		// Unknown command - send help
		s.ChannelMessageSend(m.ChannelID, fmt.Sprintf("Unknown command: `%s`. Type `!help` for available commands.", prefix))
	}

}
