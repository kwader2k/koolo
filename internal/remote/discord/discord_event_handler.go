package discord

import (
	"bytes"
	"context"
	"fmt"
	"image/jpeg"

	"github.com/bwmarrin/discordgo"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/event"
)

func (b *Bot) Handle(ctx context.Context, e event.Event) error {
	if !b.shouldPublish(e) {
		return nil
	}

	switch evt := e.(type) {
	case event.GameCreatedEvent:
		message := fmt.Sprintf("**[%s]** %s\nGame: %s\nPassword: %s", evt.Supervisor(), evt.Message(), evt.Name, evt.Password)
		return b.sendEventMessage(ctx, message)
	case event.GameFinishedEvent:
		message := fmt.Sprintf("**[%s]** %s", evt.Supervisor(), evt.Message())
		return b.sendEventMessage(ctx, message)
	case event.RunStartedEvent:
		message := fmt.Sprintf("**[%s]** started a new run: **%s**", evt.Supervisor(), evt.RunName)
		return b.sendEventMessage(ctx, message)
	case event.RunFinishedEvent:
		message := fmt.Sprintf("**[%s]** finished run: **%s** (%s)", evt.Supervisor(), evt.RunName, evt.Reason)
		return b.sendEventMessage(ctx, message)
	default:
		break
	}

	if e.Image() == nil {
		return nil
	}

	buf := new(bytes.Buffer)
	if err := jpeg.Encode(buf, e.Image(), &jpeg.Options{Quality: 80}); err != nil {
		return err
	}

	message := fmt.Sprintf("**[%s]** %s", e.Supervisor(), e.Message())
	return b.sendScreenshot(ctx, message, buf.Bytes())
}

func (b *Bot) sendEventMessage(ctx context.Context, message string) error {
	if b.useWebhook {
		return b.webhookClient.Send(ctx, message, "", nil)
	}

	_, err := b.discordSession.ChannelMessageSend(b.channelID, message)
	return err
}

func (b *Bot) sendScreenshot(ctx context.Context, message string, image []byte) error {
	if b.useWebhook {
		return b.webhookClient.Send(ctx, message, "Screenshot.jpeg", image)
	}

	reader := bytes.NewReader(image)
	_, err := b.discordSession.ChannelMessageSendComplex(b.channelID, &discordgo.MessageSend{
		File:    &discordgo.File{Name: "Screenshot.jpeg", ContentType: "image/jpeg", Reader: reader},
		Content: message,
	})
	return err
}

func (b *Bot) shouldPublish(e event.Event) bool {

	switch evt := e.(type) {
	case event.GameFinishedEvent:
		if evt.Reason == event.FinishedError {
			return config.Koolo.Discord.EnableDiscordErrorMessages
		}
		if evt.Reason == event.FinishedChicken || evt.Reason == event.FinishedMercChicken || evt.Reason == event.FinishedDied {
			return config.Koolo.Discord.EnableDiscordChickenMessages
		}
		if evt.Reason == event.FinishedOK {
			return false // supress game finished messages until we add proper option for it
		}
		return true
	case event.GameCreatedEvent:
		return config.Koolo.Discord.EnableGameCreatedMessages
	case event.RunStartedEvent:
		return config.Koolo.Discord.EnableNewRunMessages
	case event.RunFinishedEvent:
		return config.Koolo.Discord.EnableRunFinishMessages
	default:
		break
	}

	return e.Image() != nil
}
