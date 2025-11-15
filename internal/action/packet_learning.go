package action

import (
	"fmt"
	"log/slog"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/koolo/internal/context"
)

// LearnSkillPacket uses packet 0x3B to instantly learn a skill
// Faster than UI interaction - useful for leveling automation
func LearnSkillPacket(skillID skill.ID) error {
	ctx := context.Get()
	ctx.SetLastAction("LearnSkillPacket")

	// Verify we have skill points available
	skillPoints, found := ctx.Data.PlayerUnit.FindStat(stat.SkillPoints, 0)
	if !found || skillPoints.Value <= 0 {
		return fmt.Errorf("no skill points available to allocate")
	}

	// Send packet to learn skill
	ctx.Logger.Debug("Learning skill via packet",
		slog.Int("skill_id", int(skillID)),
	)

	if err := ctx.PacketSender.LearnSkill(skillID); err != nil {
		return fmt.Errorf("failed to learn skill %d via packet: %w", skillID, err)
	}

	// Delay for server processing and safety (prevent packet spam)
	time.Sleep(100 * time.Millisecond)

	// Refresh game data to see updated skills
	ctx.RefreshGameData()

	return nil
}

// AllocateStatPointPacket uses packet 0x3A to instantly allocate a stat point
// Faster than UI interaction - useful for leveling automation
func AllocateStatPointPacket(statID stat.ID) error {
	ctx := context.Get()
	ctx.SetLastAction("AllocateStatPointPacket")

	// Verify we have stat points available
	statPoints, found := ctx.Data.PlayerUnit.FindStat(stat.StatPoints, 0)
	if !found || statPoints.Value <= 0 {
		return fmt.Errorf("no stat points available to allocate")
	}

	// Send packet to allocate stat
	ctx.Logger.Debug("Allocating stat point via packet",
		slog.Int("stat_id", int(statID)),
	)

	if err := ctx.PacketSender.AllocateStatPoint(statID); err != nil {
		return fmt.Errorf("failed to allocate stat %d via packet: %w", statID, err)
	}

	// Delay for server processing and safety (prevent packet spam)
	time.Sleep(100 * time.Millisecond)

	// Refresh game data to see updated stats
	ctx.RefreshGameData()

	return nil
}
