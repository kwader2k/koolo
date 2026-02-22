package step

import (
	"errors"
	"fmt"
	"sync"
	"time"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/npc"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	"github.com/hectorgimenez/d2go/pkg/utils"
	"github.com/hectorgimenez/koolo/internal/chicken"
	"github.com/hectorgimenez/koolo/internal/context"
	"github.com/hectorgimenez/koolo/internal/game"
	"github.com/hectorgimenez/koolo/internal/packet"
)

const attackCycleDuration = 120 * time.Millisecond
const repositionCooldown = 2 * time.Second // Constant for repositioning cooldown

var (
	statesMutex           sync.RWMutex
	monsterStates         = make(map[data.UnitID]*attackState)
	ErrMonsterUnreachable = errors.New("monster appears to be unreachable or unkillable")
)

// Contains all configuration for an attack sequence
type attackSettings struct {
	primaryAttack    bool          // Whether this is a primary (left click) attack
	skill            skill.ID      // Skill ID for secondary attacks
	followEnemy      bool          // Whether to follow the enemy while attacking
	minDistance      int           // Minimum attack range
	maxDistance      int           // Maximum attack range
	aura             skill.ID      // Aura to maintain during attack
	target           data.UnitID   // Specific target's unit ID (0 for AOE)
	shouldStandStill bool          // Whether to stand still while attacking
	numOfAttacks     int           // Number of attacks to perform
	timeout          time.Duration // Timeout for the attack sequence
	isBurstCastSkill bool          // Whether this is a channeled/burst skill like Nova
}

// AttackOption defines a function type for configuring attack settings
type AttackOption func(step *attackSettings)

type attackState struct {
	lastHealth             int
	lastHealthCheckTime    time.Time
	failedAttemptStartTime time.Time
	lastRepositionTime     time.Time
	repositionAttempts     int
	position               data.Position
}

// Distance configures attack to follow enemy within specified range
func Distance(minimum, maximum int) AttackOption {
	return func(step *attackSettings) {
		step.followEnemy = true
		step.minDistance = minimum
		step.maxDistance = maximum
	}
}

// RangedDistance configures attack for ranged combat without following
func RangedDistance(minimum, maximum int) AttackOption {
	return func(step *attackSettings) {
		step.followEnemy = false // Don't follow enemies for ranged attacks
		step.minDistance = minimum
		step.maxDistance = maximum
	}
}

// StationaryDistance configures attack to remain stationary (like FoH)
func StationaryDistance(minimum, maximum int) AttackOption {
	return func(step *attackSettings) {
		step.followEnemy = false
		step.minDistance = minimum
		step.maxDistance = maximum
		step.shouldStandStill = true
	}
}

// EnsureAura ensures specified aura is active during attack
func EnsureAura(aura skill.ID) AttackOption {
	return func(step *attackSettings) {
		step.aura = aura
	}
}

// PrimaryAttack initiates a primary (left-click) attack sequence
func PrimaryAttack(target data.UnitID, numOfAttacks int, standStill bool, opts ...AttackOption) error {
	ctx := context.Get()

	// Special handling for Berserker characters
	if berserker, ok := ctx.Char.(interface{ PerformBerserkAttack(data.UnitID) }); ok {
		for i := 0; i < numOfAttacks; i++ {
			berserker.PerformBerserkAttack(target)
		}
		return nil
	}

	settings := attackSettings{
		target:           target,
		numOfAttacks:     numOfAttacks,
		shouldStandStill: standStill,
		primaryAttack:    true,
	}
	for _, o := range opts {
		o(&settings)
	}

	return attack(settings)
}

// SecondaryAttack initiates a secondary (right-click) attack sequence with a specific skill
func SecondaryAttack(skill skill.ID, target data.UnitID, numOfAttacks int, opts ...AttackOption) error {
	settings := attackSettings{
		target:           target,
		numOfAttacks:     numOfAttacks,
		skill:            skill,
		primaryAttack:    false,
		isBurstCastSkill: skill == 48, // nova can define any other burst skill here
	}
	for _, o := range opts {
		o(&settings)
	}

	if settings.isBurstCastSkill {
		settings.timeout = 30 * time.Second
		return burstAttack(settings)
	}

	return attack(settings)
}

// Helper function to validate if a monster should be targetable
func isValidEnemy(monster data.Monster, ctx *context.Status) bool {
	// Special case: Always allow Vizier seal boss even if off grid
	isVizier := monster.Type == data.MonsterTypeSuperUnique && monster.Name == npc.StormCaster
	if isVizier {
		return monster.Stats[stat.Life] > 0
	}

	// Skip monsters in invalid positions
	if !ctx.Data.AreaData.IsWalkable(monster.Position) {
		return false
	}

	// Skip dead monsters
	if monster.Stats[stat.Life] <= 0 {
		return false
	}

	return true
}

// Cleanup function to ensure proper state on exit
func keyCleanup(ctx *context.Status) {
	ctx.HID.KeyUp(ctx.Data.KeyBindings.StandStill)
}

func attack(settings attackSettings) error {
	ctx := context.Get()
	ctx.SetLastStep("Attack")
	defer keyCleanup(ctx) // cleanup possible pressed keys/buttons

	numOfAttacksRemaining := settings.numOfAttacks
	lastRunAt := time.Time{}

	for {
		ctx.PauseIfNotPriority()
		chicken.CheckForScaryAuraAndCurse()

		if numOfAttacksRemaining <= 0 {
			return nil
		}

		monster, found := ctx.Data.Monsters.FindByID(settings.target)
		if !found || !isValidEnemy(monster, ctx) {
			return nil // Target is not valid, we don't have anything to attack
		}

		distance := ctx.PathFinder.DistanceFromMe(monster.Position)
		if !lastRunAt.IsZero() && !settings.followEnemy && distance > settings.maxDistance {
			return nil // Enemy is out of range and followEnemy is disabled, we cannot attack
		}

		// Check if we need to reposition if we aren't doing any damage (prevent attacking through doors etc.)
		_, state := checkMonsterDamage(monster) // Get the state
		needsRepositioning := !state.failedAttemptStartTime.IsZero() &&
			time.Since(state.failedAttemptStartTime) > 3*time.Second

		// Be sure we stay in range of the enemy. ensureEnemyIsInRange will handle reposition attempts.
		err := ensureEnemyIsInRange(monster, state, settings.maxDistance, settings.minDistance, needsRepositioning)
		if err != nil {
			if errors.Is(err, ErrMonsterUnreachable) {
				ctx.Logger.Info(fmt.Sprintf("Giving up on monster [%d] (Area: %s) due to unreachability/unkillability.", monster.Name, ctx.Data.PlayerUnit.Area.Area().Name))
				statesMutex.Lock()
				delete(monsterStates, settings.target) // Clean up state for this monster
				statesMutex.Unlock()
				return nil // Return nil, allowing the higher-level action to find a new monster or finish.
			}
			return err // Propagate other errors from ensureEnemyIsInRange
		}

		// Handle aura activation
		if settings.aura != 0 && lastRunAt.IsZero() {
			ctx.HID.PressKeyBinding(ctx.Data.KeyBindings.MustKBForSkill(settings.aura))
		}

		// Attack timing check
		if time.Since(lastRunAt) <= ctx.Data.PlayerCastDuration()-attackCycleDuration {
			continue
		}

		performAttack(ctx, settings, monster.UnitID, monster.Position.X, monster.Position.Y)

		lastRunAt = time.Now()
		numOfAttacksRemaining--
	}
}

func burstAttack(settings attackSettings) error {
	ctx := context.Get()
	ctx.SetLastStep("BurstAttack")
	defer keyCleanup(ctx) // cleanup possible pressed keys/buttons

	monster, found := ctx.Data.Monsters.FindByID(settings.target)
	if !found || !isValidEnemy(monster, ctx) {
		return nil // Target is not valid, we don't have anything to attack
	}

	// Initially we try to move to the enemy, later we will check for closer enemies to keep attacking
	_, state := checkMonsterDamage(monster)                                                        // Get the state for the initial monster
	err := ensureEnemyIsInRange(monster, state, settings.maxDistance, settings.minDistance, false) // No initial repositioning check for burst
	if err != nil {
		if errors.Is(err, ErrMonsterUnreachable) {
			ctx.Logger.Info(fmt.Sprintf("Giving up on initial monster [%d] (Area: %s) due to unreachability/unkillability during burst.", monster.Name, ctx.Data.PlayerUnit.Area.Area().Name))
			statesMutex.Lock()
			delete(monsterStates, monster.UnitID) // Clean up state for this monster
			statesMutex.Unlock()
			return nil // Exit burst attack, caller will find next target.
		}
		return err // Propagate error from initial range check
	}

	startedAt := time.Now()
	for {
		ctx.PauseIfNotPriority()
		chicken.CheckForScaryAuraAndCurse()

		if !startedAt.IsZero() && time.Since(startedAt) > settings.timeout {
			return nil // Timeout reached, finish attack sequence
		}

		target := data.Monster{}
		for _, m := range ctx.Data.Monsters.Enemies() { // Changed 'monster' to 'm' to avoid shadowing
			distance := ctx.PathFinder.DistanceFromMe(m.Position)
			if isValidEnemy(m, ctx) && distance <= settings.maxDistance {
				target = m
				break
			}
		}

		if target.UnitID == 0 {
			return nil // We have no valid targets in range, finish attack sequence
		}

		// Check if we need to reposition if we aren't doing any damage
		_, state = checkMonsterDamage(target) // Get the state for the current target

		needsRepositioning := !state.failedAttemptStartTime.IsZero() &&
			time.Since(state.failedAttemptStartTime) > 3*time.Second

		// If we don't have LoS we will need to interrupt and move :(
		if !ctx.PathFinder.LineOfSight(ctx.Data.PlayerUnit.Position, target.Position) || needsRepositioning {
			// ensureEnemyIsInRange will handle reposition attempts and return nil if it skips
			err = ensureEnemyIsInRange(target, state, settings.maxDistance, settings.minDistance, needsRepositioning)
			if err != nil {
				if errors.Is(err, ErrMonsterUnreachable) { // HANDLE NEW ERROR
					ctx.Logger.Info(fmt.Sprintf("Giving up on monster [%d] (Area: %s) due to unreachability/unkillability during burst.", target.Name, ctx.Data.PlayerUnit.Area.Area().Name))
					statesMutex.Lock()
					delete(monsterStates, target.UnitID) // Clean up state for this monster
					statesMutex.Unlock()
					return nil // Exit burst attack, caller will find next target.
				}
				return err // Propagate general errors from ensureEnemyIsInRange
			}
			continue // Continue loop to re-evaluate conditions after a potential move
		}

		performAttack(ctx, settings, target.UnitID, target.Position.X, target.Position.Y)
	}
}

func selectSecondarySkillButton(ctx *context.Status, skillID skill.ID) (game.MouseButton, bool) {
	if skillID == 0 {
		return game.RightButton, false
	}
	if ctx.Data.PlayerUnit.LeftSkill == skillID {
		return game.LeftButton, false
	}
	if ctx.Data.PlayerUnit.RightSkill == skillID {
		return game.RightButton, false
	}
	button, ok := SelectSkill(skillID)
	if !ok {
		return game.RightButton, false
	}
	return button, true
}

func performAttack(ctx *context.Status, settings attackSettings, targetID data.UnitID, x, y int) {
	monsterPos := data.Position{X: x, Y: y}
	if !ctx.PathFinder.LineOfSight(ctx.Data.PlayerUnit.Position, monsterPos) && !ctx.ForceAttack {
		return // Skip attack if no line of sight
	}

	// Check if we should use packet casting for Blizzard (location-based)
	useBlizzardPacket := false
	if settings.skill == skill.Blizzard {
		switch ctx.CharacterCfg.Character.Class {
		case "sorceress":
			useBlizzardPacket = ctx.CharacterCfg.Character.BlizzardSorceress.UseBlizzardPackets
		case "sorceress_leveling":
			useBlizzardPacket = ctx.CharacterCfg.Character.SorceressLeveling.UseBlizzardPackets
		}
	}

	// If using packet casting for Blizzard (location-based skill)
	if useBlizzardPacket {
		// Ensure we have Blizzard selected on right-click
		if ctx.Data.PlayerUnit.RightSkill != skill.Blizzard {
			SelectRightSkill(skill.Blizzard)
			time.Sleep(time.Millisecond * 10)
		}

		// Send packet to cast Blizzard at location
		if err := ctx.PacketSender.CastSkillAtLocation(monsterPos); err != nil {
			ctx.Logger.Warn("Failed to cast Blizzard via packet, falling back to mouse", "error", err)
			// Fall back to regular mouse casting
			performMouseAttack(ctx, settings, x, y)
		}
		return
	}

	// Check if we should use entity-targeted packet casting
	if ctx.CharacterCfg.PacketCasting.UseForEntitySkills && ctx.PacketSender != nil && targetID != 0 {
		// Ensure we have the skill selected
		if settings.primaryAttack {
			if settings.skill != 0 && ctx.Data.PlayerUnit.LeftSkill != settings.skill {
				SelectLeftSkill(settings.skill)
				time.Sleep(time.Millisecond * 10)
			}
			// Send left-click entity skill packet
			castPacket := packet.NewCastSkillEntityLeft(targetID)
			if err := ctx.PacketSender.SendPacket(castPacket.GetPayload()); err != nil {
				ctx.Logger.Warn("Failed to cast entity skill via packet (left), falling back to mouse", "error", err)
				performMouseAttack(ctx, settings, x, y)
			} else {
				// Respect cast duration to avoid spamming server
				time.Sleep(ctx.Data.PlayerCastDuration())
			}
		} else {
			selectedButton, selected := selectSecondarySkillButton(ctx, settings.skill)
			if selected {
				time.Sleep(time.Millisecond * 10)
			}
			if selectedButton == game.LeftButton {
				castPacket := packet.NewCastSkillEntityLeft(targetID)
				if err := ctx.PacketSender.SendPacket(castPacket.GetPayload()); err != nil {
					ctx.Logger.Warn("Failed to cast entity skill via packet (left), falling back to mouse", "error", err)
					performMouseAttack(ctx, settings, x, y)
				} else {
					// Respect cast duration to avoid spamming server
					time.Sleep(ctx.Data.PlayerCastDuration())
				}
			} else {
				castPacket := packet.NewCastSkillEntityRight(targetID)
				if err := ctx.PacketSender.SendPacket(castPacket.GetPayload()); err != nil {
					ctx.Logger.Warn("Failed to cast entity skill via packet (right), falling back to mouse", "error", err)
					performMouseAttack(ctx, settings, x, y)
				} else {
					// Respect cast duration to avoid spamming server
					time.Sleep(ctx.Data.PlayerCastDuration())
				}
			}
		}
		return
	}

	// Regular mouse-based attack
	performMouseAttack(ctx, settings, x, y)
}

func performMouseAttack(ctx *context.Status, settings attackSettings, x, y int) {
	selectedButton := game.RightButton
	if settings.primaryAttack {
		selectedButton = game.LeftButton
	} else {
		var selected bool
		selectedButton, selected = selectSecondarySkillButton(ctx, settings.skill)
		if selected {
			time.Sleep(time.Millisecond * 10)
		}
	}

	if settings.shouldStandStill {
		ctx.HID.KeyDown(ctx.Data.KeyBindings.StandStill)
	}

	x, y = ctx.PathFinder.GameCoordsToScreenCords(x, y)
	ctx.HID.Click(selectedButton, x, y)

	if settings.shouldStandStill {
		ctx.HID.KeyUp(ctx.Data.KeyBindings.StandStill)
	}
}

// Modified: Added 'state' parameter to manage lastRepositionTime and repositionAttempts
func ensureEnemyIsInRange(monster data.Monster, state *attackState, maxDistance, minDistance int, needsRepositioning bool) error {
	ctx := context.Get()
	ctx.SetLastStep("ensureEnemyIsInRange")

	currentPos := ctx.Data.PlayerUnit.Position
	distanceToMonster := ctx.PathFinder.DistanceFromMe(monster.Position)
	hasLoS := ctx.PathFinder.LineOfSight(currentPos, monster.Position)

	// If we are already in range, have LoS, and don't need repositioning, we are good.
	// Reset repositionAttempts for future needs.
	if hasLoS && distanceToMonster <= maxDistance && !needsRepositioning {
		state.repositionAttempts = 0 // Reset attempts if we're in a good state
		return nil
	}

	// Handle repositioning if needed (due to no damage, or no LoS for burst attacks)
	if needsRepositioning {
		// If we've already tried repositioning once for this "stuck" phase
		if state.repositionAttempts >= 1 { // This is the problematic part. User wants to allow 1 attempt.
			ctx.Logger.Info(fmt.Sprintf(
				"Already attempted repositioning for monster [%d] in area [%s]. Skipping further attempts and considering monster unkillable.", // Updated log message
				monster.Name, ctx.Data.PlayerUnit.Area.Area().Name,
			))
			return ErrMonsterUnreachable // <-- CHANGE: Return specific error
		}

		// Check if enough time has passed since the last reposition attempt (cooldown)
		if time.Since(state.lastRepositionTime) < repositionCooldown {
			return nil // Still on cooldown, do not reposition yet. Return nil to continue attacking.
		}

		ctx.Logger.Info(fmt.Sprintf(
			"No damage taken by target monster [%d] in area [%s] for more than 3 seconds. Trying to re-position (attempt %d/1)",
			monster.Name, ctx.Data.PlayerUnit.Area.Area().Name, state.repositionAttempts+1,
		))

		dest := ctx.PathFinder.BeyondPosition(currentPos, monster.Position, 4)
		err := MoveTo(dest, WithIgnoreMonsters())
		state.repositionAttempts++ // Increment attempt count after trying to move
		if err != nil {
			ctx.Logger.Error(fmt.Sprintf("MoveTo failed during reposition attempt for monster [%d]: %v", monster.Name, err))
			// Do NOT update lastRepositionTime here if MoveTo completely failed, so it can try again sooner if the path clears.
			// However, since we're only allowing ONE attempt, the increment of repositionAttempts handles the "give up" logic.
			return nil // Continue attacking, but the next loop iteration will hit repositionAttempts >= 1 and return ErrMonsterUnreachable
		}
		state.lastRepositionTime = time.Now() // Update the last reposition time only if MoveTo was initiated without error
		return nil                            // Successfully initiated the move, continue attacking next loop iteration
	}

	// Any close-range combat (mosaic,barb...) should move directly to target
	// This is general movement, not triggered by needsRepositioning (no damage), so don't touch repositionAttempts.
	if maxDistance <= 3 {
		return MoveTo(monster.Position, WithIgnoreMonsters(), WithDistanceToFinish(max(2, maxDistance)))
	}

	// Get path to monster
	path, _, found := ctx.PathFinder.GetPath(monster.Position)
	// We cannot reach the enemy, let's skip the attack sequence by returning an error
	if !found {
		return errors.New("path could not be calculated to reach monster") // This is a fundamental pathing error, propagate it.
	}

	// Look for suitable position along path
	for _, pos := range path {
		monsterDistance := utils.DistanceFromPoint(ctx.Data.AreaData.RelativePosition(monster.Position), pos)
		if monsterDistance > maxDistance || monsterDistance < minDistance {
			continue
		}

		dest := data.Position{
			X: pos.X + ctx.Data.AreaData.OffsetX,
			Y: pos.Y + ctx.Data.AreaData.OffsetY,
		}

		// Handle overshooting for short distances (Nova distances)
		distanceToMove := ctx.PathFinder.DistanceFromMe(dest)
		if distanceToMove <= DistanceToFinishMoving {
			dest = ctx.PathFinder.BeyondPosition(currentPos, dest, 9)
		}

		if ctx.PathFinder.LineOfSight(dest, monster.Position) && !ctx.ForceAttack {
			// This is also general movement to get into attack range, not a "repositioning attempt" for being stuck.
			return MoveTo(dest, WithIgnoreMonsters())
		}
	}

	return nil // No suitable position found along path, continue attacking
}

func checkMonsterDamage(monster data.Monster) (bool, *attackState) {
	statesMutex.Lock()
	defer statesMutex.Unlock()

	state, exists := monsterStates[monster.UnitID]
	if !exists {
		state = &attackState{
			lastHealth:          monster.Stats[stat.Life],
			lastHealthCheckTime: time.Now(),
			position:            monster.Position,
			repositionAttempts:  0, // Initialize counter to 0 for new states
		}
		monsterStates[monster.UnitID] = state
	}

	didDamage := false
	currentHealth := monster.Stats[stat.Life]

	// Only update health check if some time has passed
	if time.Since(state.lastHealthCheckTime) > 100*time.Millisecond {
		if currentHealth < state.lastHealth {
			didDamage = true
			state.failedAttemptStartTime = time.Time{}
			state.repositionAttempts = 0 // Reset attempts when damage is successfully dealt
		} else if state.failedAttemptStartTime.IsZero() &&
			monster.Position == state.position { // only start failing if monster hasn't moved
			state.failedAttemptStartTime = time.Now()
			state.repositionAttempts = 0 // Reset attempts when starting a new failed phase
		}

		state.lastHealth = currentHealth
		state.lastHealthCheckTime = time.Now()
		state.position = monster.Position

		// Clean up old entries periodically
		if len(monsterStates) > 100 {
			now := time.Now()
			for id, s := range monsterStates {
				if now.Sub(s.lastHealthCheckTime) > 5*time.Minute {
					delete(monsterStates, id)
				}
			}
		}
	}

	return didDamage, state
}
