package game

import (
	"fmt"

	"github.com/hectorgimenez/d2go/pkg/data"
	"github.com/hectorgimenez/d2go/pkg/data/skill"
	"github.com/hectorgimenez/d2go/pkg/data/stat"
	packet "github.com/hectorgimenez/koolo/internal/packet"
)

type ProcessSender interface {
	SendPacket([]byte) error
}

type PacketSender struct {
	process ProcessSender
}

func NewPacketSender(process ProcessSender) *PacketSender {
	return &PacketSender{
		process: process,
	}
}

func (ps *PacketSender) SendPacket(packet []byte) error {
	return ps.process.SendPacket(packet)
}

func (ps *PacketSender) PickUpItem(item data.Item) error {
	err := ps.SendPacket(packet.NewPickUpItem(item).GetPayload())
	if err != nil {
		return fmt.Errorf("failed to send pick item packet: %w", err)
	}

	return nil
}

func (ps *PacketSender) InteractWithTp(object data.Object) error {
	if err := ps.SendPacket(packet.NewTpInteraction(object).GetPayload()); err != nil {
		return fmt.Errorf("failed to send tp interaction packet: %w", err)
	}
	return nil
}

func (ps *PacketSender) InteractWithEntrance(entrance data.Entrance) error {
	if err := ps.SendPacket(packet.NewEntranceInteraction(entrance).GetPayload()); err != nil {
		return fmt.Errorf("failed to send entrance interaction packet: %w", err)
	}
	return nil
}

func (ps *PacketSender) Teleport(position data.Position) error {
	payload := packet.NewTeleport(position).GetPayload()

	// Debug: log the packet being sent
	fmt.Printf("Sending teleport packet: X=%d Y=%d Hex=%02X\n", position.X, position.Y, payload)

	if err := ps.SendPacket(payload); err != nil {
		return fmt.Errorf("failed to send teleport packet: %w", err)
	}
	return nil
}

// TelekinesisInteraction sends packet 0x0D for object interaction using telekinesis
// Use cases: waypoints, chests, shrines from distance (Sorceress only)
// Requires character to have Telekinesis skill and be within interaction range
func (ps *PacketSender) TelekinesisInteraction(objectGID data.UnitID) error {
	if err := ps.SendPacket(packet.NewTelekinesisInteraction(objectGID).GetPayload()); err != nil {
		return fmt.Errorf("failed to send telekinesis interaction packet: %w", err)
	}
	return nil
}

// CastSkillAtLocation sends packet 0x0C to cast a skill at a specific location
// Use cases: Blizzard, Meteor, Frozen Orb, or any location-targeted skill
// Useful for faster/more precise casting than HID mouse clicks
func (ps *PacketSender) CastSkillAtLocation(position data.Position) error {
	payload := packet.NewCastSkillLocation(position).GetPayload()

	if err := ps.SendPacket(payload); err != nil {
		return fmt.Errorf("failed to send cast skill at location packet: %w", err)
	}
	return nil
}

// SelectRightSkill sends packet 0x3C to change the active right-click skill
// Use cases: Switch skills via packet instead of clicking UI (F1-F9 functionality)
// Useful for quick skill switching during combat or automation
func (ps *PacketSender) SelectRightSkill(skillID skill.ID) error {
	if err := ps.SendPacket(packet.NewSkillSelection(skillID).GetPayload()); err != nil {
		return fmt.Errorf("failed to send skill selection packet: %w", err)
	}
	return nil
}

// SelectLeftSkill sends packet 0x3C to change the active left-click skill
// Use cases: Switch left-click skills via packet instead of clicking UI
// Useful for automation or quick skill switching
func (ps *PacketSender) SelectLeftSkill(skillID skill.ID) error {
	if err := ps.SendPacket(packet.NewLeftSkillSelection(skillID).GetPayload()); err != nil {
		return fmt.Errorf("failed to send left skill selection packet: %w", err)
	}
	return nil
}

// LearnSkill sends packet 0x3B to allocate a skill point
// Use cases: Faster skill point allocation during leveling (Sorceress leveling)
// Bypasses UI interaction for instant skill learning
func (ps *PacketSender) LearnSkill(skillID skill.ID) error {
	if err := ps.SendPacket(packet.NewLearnSkill(skillID).GetPayload()); err != nil {
		return fmt.Errorf("failed to send learn skill packet: %w", err)
	}
	return nil
}

// AllocateStatPoint sends packet 0x3A to allocate a stat point
// Use cases: Faster stat point allocation during leveling (Sorceress leveling)
// Bypasses UI interaction for instant stat allocation
func (ps *PacketSender) AllocateStatPoint(statID stat.ID) error {
	if err := ps.SendPacket(packet.NewAllocateStat(statID).GetPayload()); err != nil {
		return fmt.Errorf("failed to send allocate stat packet: %w", err)
	}
	return nil
}
