package action

import (
	"fmt"
	"strings"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/area"
	"github.com/hectorgimenez/d2go/pkg/data/item"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/action/step"
	"github.com/hectorgimenez/koolo/internal/config"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/ui"
	"github.com/hectorgimenez/koolo/internal/utils"
	"github.com/lxn/win"
)

func AutoRespecIfNeeded() error {
	ctx := context.Get()
	ctx.SetLastAction("AutoRespecIfNeeded")

	if _, isLevelingChar := ctx.Char.(context.LevelingCharacter); isLevelingChar {
		return nil
	}

	autoCfg := ctx.CharacterCfg.Character.AutoStatSkill
	if !autoCfg.Enabled || !autoCfg.Respec.Enabled || autoCfg.Respec.Applied {
		return nil
	}

	if autoCfg.Respec.TargetLevel > 0 {
		level, ok := ctx.Data.PlayerUnit.FindStat(stat.Level, 0)
		if !ok || level.Value != autoCfg.Respec.TargetLevel {
			return nil
		}
	}

	if !ctx.Data.PlayerUnit.Area.IsTown() {
		return nil
	}

	statTargets := make([]string, 0, len(autoCfg.Stats))
	for _, entry := range autoCfg.Stats {
		if entry.Stat == "" || entry.Target <= 0 {
			continue
		}
		statTargets = append(statTargets, fmt.Sprintf("%s=%d", entry.Stat, entry.Target))
	}
	skillTargets := make([]string, 0, len(autoCfg.Skills))
	for _, entry := range autoCfg.Skills {
		if entry.Skill == "" || entry.Target <= 0 {
			continue
		}
		skillTargets = append(skillTargets, fmt.Sprintf("%s=%d", entry.Skill, entry.Target))
	}
	ctx.Logger.Info("Auto respec: applying targets", "targetLevel", autoCfg.Respec.TargetLevel, "stats", statTargets, "skills", skillTargets)

	beforeStatPoints := getStatValue(stat.StatPoints)
	beforeSkillPoints := getStatValue(stat.SkillPoints)

	usedAkara := false
	tokenUsed := false

	tryAkara := func() bool {
		if err := respecAtAkara(); err != nil {
			ctx.Logger.Warn("Auto respec: Akara respec failed", "error", err)
		}
		ctx.RefreshGameData()
		afterAkaraStatPoints := getStatValue(stat.StatPoints)
		afterAkaraSkillPoints := getStatValue(stat.SkillPoints)
		return afterAkaraStatPoints != beforeStatPoints || afterAkaraSkillPoints != beforeSkillPoints
	}

	tryToken := func() bool {
		used, err := tryConsumeRespecToken()
		if err != nil {
			ctx.Logger.Warn("Auto respec: token use failed", "error", err)
		}
		if used {
			return true
		}
		ctx.RefreshGameData()
		afterTokenStatPoints := getStatValue(stat.StatPoints)
		afterTokenSkillPoints := getStatValue(stat.SkillPoints)
		return afterTokenStatPoints != beforeStatPoints || afterTokenSkillPoints != beforeSkillPoints
	}

	if autoCfg.Respec.TokenFirst {
		tokenUsed = tryToken()
		if !tokenUsed {
			usedAkara = tryAkara()
		}
	} else {
		usedAkara = tryAkara()
		if !usedAkara {
			tokenUsed = tryToken()
		}
	}

	if !usedAkara && !tokenUsed {
		ctx.Logger.Warn("Auto respec: no respec method succeeded")
		return nil
	}

	ctx.RefreshGameData()
	afterStatPoints := getStatValue(stat.StatPoints)
	afterSkillPoints := getStatValue(stat.SkillPoints)
	if afterStatPoints == beforeStatPoints && afterSkillPoints == beforeSkillPoints {
		ctx.Logger.Warn("Auto respec: no point change detected after respec", "statBefore", beforeStatPoints, "statAfter", afterStatPoints, "skillBefore", beforeSkillPoints, "skillAfter", afterSkillPoints)
	}

	ctx.CharacterCfg.Character.AutoStatSkill.Respec.Applied = true
	ctx.CharacterCfg.Character.AutoStatSkill.Respec.Enabled = false
	if err := config.SaveSupervisorConfig(ctx.Name, ctx.CharacterCfg); err != nil {
		ctx.Logger.Error("Auto respec: failed to save config", "error", err)
		return err
	}

	ctx.Logger.Info("Auto respec: completed", "targetLevel", autoCfg.Respec.TargetLevel)
	return nil
}

func tryConsumeRespecToken() (bool, error) {
	ctx := context.Get()
	tokenName := item.Name("TokenofAbsolution")

	token, found := ctx.Data.Inventory.Find(tokenName, item.LocationInventory)
	if !found {
		stashedToken, foundStash := ctx.Data.Inventory.Find(tokenName, item.LocationStash, item.LocationSharedStash)
		if !foundStash {
			return false, nil
		}
		if err := TakeItemsFromStash([]data.Item{stashedToken}); err != nil {
			step.CloseAllMenus()
			return false, fmt.Errorf("take token from stash: %w", err)
		}
		step.CloseAllMenus()
		ctx.RefreshGameData()
		token, found = ctx.Data.Inventory.Find(tokenName, item.LocationInventory)
		if !found {
			return false, fmt.Errorf("token not found in inventory after stash transfer")
		}
	}

	if !ctx.Data.OpenMenus.Inventory {
		ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.Inventory)
		utils.Sleep(300)
		ctx.RefreshGameData()
	}

	screenPos := ui.GetScreenCoordsForItem(token)
	ctx.HID.Click(game.RightButton, screenPos.X, screenPos.Y)
	utils.Sleep(800)
	ctx.RefreshGameData()
	step.CloseAllMenus()

	if _, stillFound := ctx.Data.Inventory.Find(tokenName, item.LocationInventory); stillFound {
		return false, fmt.Errorf("token still present after use")
	}

	return true, nil
}

func respecAtAkara() error {
	ctx := context.Get()
	currentArea := ctx.Data.PlayerUnit.Area
	if currentArea != area.RogueEncampment {
		if err := WayPoint(area.RogueEncampment); err != nil {
			return err
		}
	}

	if err := InteractNPC(npc.Akara); err != nil {
		return err
	}

	ctx.HID.KeySequence(win.VK_HOME, win.VK_DOWN, win.VK_DOWN, win.VK_RETURN)
	utils.Sleep(800)
	ctx.HID.KeySequence(win.VK_HOME, win.VK_RETURN)
	utils.Sleep(800)
	step.CloseAllMenus()

	return nil
}

func getStatValue(id stat.ID) int {
	ctx := context.Get()
	value, _ := ctx.Data.PlayerUnit.FindStat(id, 0)
	return value.Value
}

func shouldDeferAutoSkillsForStats() bool {
	ctx := context.Get()
	statPoints, hasUnusedPoints := ctx.Data.PlayerUnit.FindStat(stat.StatPoints, 0)
	if !hasUnusedPoints || statPoints.Value <= 0 {
		return false
	}

	statKeyToID := map[string]stat.ID{
		"strength":  stat.Strength,
		"dexterity": stat.Dexterity,
		"vitality":  stat.Vitality,
		"energy":    stat.Energy,
	}

	for _, entry := range ctx.CharacterCfg.Character.AutoStatSkill.Stats {
		if entry.Target <= 0 {
			continue
		}
		statKey := strings.ToLower(strings.TrimSpace(entry.Stat))
		statID, ok := statKeyToID[statKey]
		if !ok {
			continue
		}
		currentValue, _ := ctx.Data.PlayerUnit.BaseStats.FindStat(statID, 0)
		if currentValue.Value < entry.Target {
			return true
		}
	}

	return false
}
