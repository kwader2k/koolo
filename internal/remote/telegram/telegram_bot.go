package telegram

import (
	"context"
	"log/slog"
	"strings"

	tgbotapi "github.com/go-telegram-bot-api/telegram-bot-api/v5"
)

type Bot struct {
	bot    *tgbotapi.BotAPI
	chatID int64
	logger *slog.Logger
}

func (b *Bot) Start(ctx context.Context) error {
	offset, err := b.getLatestOffset()
	if err != nil { return err }

	u := tgbotapi.NewUpdate(offset)
	u.Timeout = 5
	updates := b.bot.GetUpdatesChan(u)

	for {
		select {
		case <-ctx.Done():
			b.bot.StopReceivingUpdates()
			for range updates { }
			return nil
		case update, ok := <-updates:
			if !ok { return nil }
			if update.Message != nil && update.Message.Chat != nil && update.Message.Chat.ID == b.chatID {
				switch strings.ToLower(update.Message.Text) {
				case "stats":
					// add stats handling if needed
				}
			}
		}
	}
}

func (b *Bot) getLatestOffset() (int, error) {
	upds, err := b.bot.GetUpdates(tgbotapi.NewUpdate(-1))
	if err != nil { return 0, err }
	offset := 0
	if len(upds) > 0 { offset = upds[0].UpdateID + 1 }
	return offset, nil
}
