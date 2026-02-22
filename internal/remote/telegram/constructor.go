package telegram

import (
	"log/slog"

	tgbotapi "github.com/go-telegram-bot-api/telegram-bot-api/v5"
)

// NewBot matches main.go usage: NewBot(token string, chatID int64, logger *slog.Logger)
func NewBot(token string, chatID int64, logger *slog.Logger) (*Bot, error) {
	api, err := tgbotapi.NewBotAPI(token)
	if err != nil {
		return nil, err
	}
	return &Bot{bot: api, chatID: chatID, logger: logger}, nil
}
