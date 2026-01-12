package discord

import (
	"bytes"
	"context"
	"fmt"
	"image/jpeg"
	"path/filepath"
	"strings"
	"time"

	"github.com/bwmarrin/discordgo"
	d2stat "github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/event"
)

var excludedStatIDs = map[int]bool{
	17:  true,
	18:  true,
	21:  true,
	22:  true,
	23:  true,
	24:  true,
	31:  true,
	48:  true,
	49:  true,
	50:  true,
	51:  true,
	52:  true,
	53:  true,
	54:  true,
	55:  true,
	57:  true,
	58:  true,
	67:  true,
	68:  true,
	72:  true,
	73:  true,
	92:  true,
	118: true,
	134: true,
	326: true,
}

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
	case event.NgrokTunnelEvent:
		return b.sendEventMessage(ctx, evt.Message())
	case event.ItemStashedEvent:
		if config.Koolo.Discord.DisableItemStashScreenshots {
			if b.useWebhook {
				embed := buildItemStashEmbed(evt)
				return b.itemWebhookClient().SendEmbed(ctx, embed)
			}
			return b.sendItemStashEmbed(evt)
		}
		if e.Image() == nil {
			return nil
		}
		buf := new(bytes.Buffer)
		if err := jpeg.Encode(buf, e.Image(), &jpeg.Options{Quality: 80}); err != nil {
			return err
		}
		message := fmt.Sprintf("**[%s]** %s", e.Supervisor(), e.Message())
		return b.sendItemScreenshot(ctx, message, buf.Bytes())
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

func (b *Bot) sendItemStashEmbed(evt event.ItemStashedEvent) error {
	embed := buildItemStashEmbed(evt)
	_, err := b.discordSession.ChannelMessageSendEmbed(b.itemChannel(), embed)
	return err
}

func (b *Bot) sendItemScreenshot(ctx context.Context, message string, image []byte) error {
	if b.useWebhook {
		return b.itemWebhookClient().Send(ctx, message, "Screenshot.jpeg", image)
	}

	reader := bytes.NewReader(image)
	_, err := b.discordSession.ChannelMessageSendComplex(b.itemChannel(), &discordgo.MessageSend{
		File:    &discordgo.File{Name: "Screenshot.jpeg", ContentType: "image/jpeg", Reader: reader},
		Content: message,
	})
	return err
}

func (b *Bot) itemChannel() string {
	if strings.TrimSpace(b.itemChannelID) != "" {
		return b.itemChannelID
	}
	return b.channelID
}

func (b *Bot) itemWebhookClient() *webhookClient {
	if b.itemWebhook != nil {
		return b.itemWebhook
	}
	return b.webhookClient
}

func buildItemStashEmbed(evt event.ItemStashedEvent) *discordgo.MessageEmbed {
	item := evt.Item.Item
	quality := item.Quality.ToString()
	return &discordgo.MessageEmbed{
		Description: buildItemStashDescription(evt),
		Color:       getCategoryColor(quality),
	}
}

func buildItemStashDescription(evt event.ItemStashedEvent) string {
	item := evt.Item.Item
	quality := item.Quality.ToString()
	itemType := item.Desc().Name
	isEthereal := item.Ethereal
	socketCount := len(item.Sockets)
	hasSocketStat := false

	itemName := string(item.Name)
	if item.IdentifiedName != "" {
		itemName = item.IdentifiedName
	}

	var description strings.Builder
	description.WriteString(fmt.Sprintf("## **%s**\n", itemName))
	switch {
	case itemType != "" && quality != "":
		description.WriteString(fmt.Sprintf("▪️ %s [%s]\n", itemType, quality))
	case itemType != "":
		description.WriteString(fmt.Sprintf("▪️ %s\n", itemType))
	case quality != "":
		description.WriteString(fmt.Sprintf("▪️ [%s]\n", quality))
	}

	if item.Identified && len(item.Stats) > 0 {
		var defense int
		var eMin, eMax int
		var fMin, fMax, lMin, lMax int
		var cMin, cMax, mMin, mMax int
		var pMin, pMax int
		var strVal, energyVal, dexVal, vitVal int
		var hasStr, hasEnergy, hasDex, hasVit bool
		var frVal, crVal, lrVal, prVal int
		var hasFr, hasCr, hasLr, hasPr bool

		for _, s := range item.Stats {
			switch s.ID {
			case d2stat.Strength:
				strVal = s.Value
				hasStr = true
			case d2stat.Energy:
				energyVal = s.Value
				hasEnergy = true
			case d2stat.Dexterity:
				dexVal = s.Value
				hasDex = true
			case d2stat.Vitality:
				vitVal = s.Value
				hasVit = true
			case d2stat.FireResist:
				frVal = s.Value
				hasFr = true
			case d2stat.ColdResist:
				crVal = s.Value
				hasCr = true
			case d2stat.LightningResist:
				lrVal = s.Value
				hasLr = true
			case d2stat.PoisonResist:
				prVal = s.Value
				hasPr = true
			case 31:
				defense = s.Value
			case 17:
				eMin = s.Value
			case 18:
				eMax = s.Value
			case 48:
				fMin = s.Value
			case 49:
				fMax = s.Value
			case 50:
				lMin = s.Value
			case 51:
				lMax = s.Value
			case 52:
				mMin = s.Value
			case 53:
				mMax = s.Value
			case 54:
				cMin = s.Value
			case 55:
				cMax = s.Value
			case 57:
				pMin = s.Value
			case 58:
				pMax = s.Value
			}
		}

		allStatsCombined := hasStr && hasEnergy && hasDex && hasVit &&
			strVal == energyVal && strVal == dexVal && strVal == vitVal
		allResistsPresent := hasFr && hasCr && hasLr && hasPr
		allResistsCombined := allResistsPresent && frVal == crVal && frVal == lrVal && frVal == prVal
		partialResistsCombined, partialResistValue, partialResistID := false, 0, d2stat.ID(0)
		if !allResistsCombined && allResistsPresent {
			partialResistsCombined, partialResistValue, partialResistID = findPartialAllResists(frVal, crVal, lrVal, prVal)
		}

		if defense > 0 {
			description.WriteString(fmt.Sprintf("Defense: %d\n", defense))
		}
		if allStatsCombined && strVal != 0 {
			description.WriteString(fmt.Sprintf("+%d to All Attributes\n", strVal))
		}
		if allResistsCombined && frVal != 0 {
			description.WriteString(fmt.Sprintf("All Resistances %+d\n", frVal))
		} else if partialResistsCombined && partialResistValue != 0 {
			description.WriteString(fmt.Sprintf("All Resistances %+d\n", partialResistValue))
			description.WriteString(fmt.Sprintf("%s %+d\n", resistLabel(partialResistID), resistValueForID(partialResistID, frVal, crVal, lrVal, prVal)))
		}
		if eMin > 0 || eMax > 0 {
			if eMin > 0 && eMax > 0 {
				description.WriteString(fmt.Sprintf("+%d-%d%% Enhanced Damage\n", eMin, eMax))
			} else if eMax > 0 {
				description.WriteString(fmt.Sprintf("+%d%% Enhanced Damage\n", eMax))
			} else {
				description.WriteString(fmt.Sprintf("+%d%% Enhanced Damage\n", eMin))
			}
		}

		description.WriteString(formatDamageLine(fMin, fMax, "Fire"))
		description.WriteString(formatDamageLine(lMin, lMax, "Lightning"))
		description.WriteString(formatDamageLine(cMin, cMax, "Cold"))
		description.WriteString(formatDamageLine(mMin, mMax, "Magic"))
		description.WriteString(formatDamageLine(pMin, pMax, "Poison"))

		for _, s := range item.Stats {
			if allStatsCombined && (s.ID == d2stat.Strength || s.ID == d2stat.Energy || s.ID == d2stat.Dexterity || s.ID == d2stat.Vitality) {
				continue
			}
			if (allResistsCombined || partialResistsCombined) &&
				(s.ID == d2stat.FireResist || s.ID == d2stat.ColdResist || s.ID == d2stat.LightningResist || s.ID == d2stat.PoisonResist) {
				continue
			}
			if excludedStatIDs[int(s.ID)] {
				continue
			}
			statText := s.String()
			if statText != "" {
				if strings.Contains(statText, "Socketed") || strings.Contains(statText, "Sockets") {
					hasSocketStat = true
				}
				description.WriteString(fmt.Sprintf("%s\n", statText))
			}
		}
	}

	if isEthereal {
		description.WriteString("Ethereal\n")
	}
	if socketCount > 0 && !hasSocketStat {
		description.WriteString(fmt.Sprintf("Sockets: %d\n", socketCount))
	}

	if config.Koolo.Discord.IncludePickitInfoInItemText && (evt.Item.RuleFile != "" || evt.Item.Rule != "") {
		location := formatRuleLocation(evt.Item.RuleFile)
		ruleLine := strings.TrimSpace(evt.Item.Rule)
		description.WriteString("\n")
		if location != "" {
			description.WriteString(fmt.Sprintf("> *%s*\n", location))
		}
		if ruleLine != "" {
			description.WriteString(fmt.Sprintf("> %s\n", ruleLine))
		}
	}

	description.WriteString(fmt.Sprintf("\n`%s | %s`", evt.Supervisor(), time.Now().Format("2006-01-02 15:04:05")))
	return strings.TrimSpace(description.String())
}

func getCategoryColor(category string) int {
	switch category {
	case "LowQuality":
		return 0x666666
	case "Normal":
		return 0xffffff
	case "Superior":
		return 0xc0c0c0
	case "Magic":
		return 0x6969ff
	case "Set":
		return 0x00ff00
	case "Rare":
		return 0xffff77
	case "Unique":
		return 0xbfa969
	case "Crafted":
		return 0xff8000
	default:
		return 0x999999
	}
}

func formatDamageLine(min, max int, damageType string) string {
	if min > 0 || max > 0 {
		return fmt.Sprintf("Adds %d-%d %s Damage\n", min, max, damageType)
	}
	return ""
}

func findPartialAllResists(frVal, crVal, lrVal, prVal int) (bool, int, d2stat.ID) {
	values := []int{frVal, crVal, lrVal, prVal}
	counts := map[int]int{}
	for _, v := range values {
		counts[v]++
	}
	for val, count := range counts {
		if count == 3 {
			switch {
			case frVal != val:
				return true, val, d2stat.FireResist
			case crVal != val:
				return true, val, d2stat.ColdResist
			case lrVal != val:
				return true, val, d2stat.LightningResist
			case prVal != val:
				return true, val, d2stat.PoisonResist
			}
		}
	}
	return false, 0, 0
}

func resistLabel(id d2stat.ID) string {
	switch id {
	case d2stat.FireResist:
		return "Fire Resist"
	case d2stat.ColdResist:
		return "Cold Resist"
	case d2stat.LightningResist:
		return "Lightning Resist"
	case d2stat.PoisonResist:
		return "Poison Resist"
	default:
		return "Resist"
	}
}

func resistValueForID(id d2stat.ID, frVal, crVal, lrVal, prVal int) int {
	switch id {
	case d2stat.FireResist:
		return frVal
	case d2stat.ColdResist:
		return crVal
	case d2stat.LightningResist:
		return lrVal
	case d2stat.PoisonResist:
		return prVal
	default:
		return 0
	}
}

func formatRuleLocation(ruleFile string) string {
	trimmed := strings.TrimSpace(ruleFile)
	if trimmed == "" {
		return ""
	}

	pathPart := trimmed
	line := ""
	if idx := strings.LastIndex(trimmed, ":"); idx != -1 && idx+1 < len(trimmed) {
		tail := strings.TrimSpace(trimmed[idx+1:])
		if isAllDigits(tail) {
			line = tail
			pathPart = strings.TrimSpace(trimmed[:idx])
		}
	}

	base := filepath.Base(pathPart)
	if base == "." || base == string(filepath.Separator) {
		base = pathPart
	}

	if line != "" {
		return fmt.Sprintf("%s : line %s", base, line)
	}
	return base
}

func isAllDigits(val string) bool {
	if val == "" {
		return false
	}
	for i := 0; i < len(val); i++ {
		if val[i] < '0' || val[i] > '9' {
			return false
		}
	}
	return true
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
	case event.NgrokTunnelEvent:
		return true
	default:
		break
	}

	return e.Image() != nil
}
