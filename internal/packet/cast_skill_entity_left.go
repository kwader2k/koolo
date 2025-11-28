package packet

import (
	"encoding/binary"

	"github.com/hectorgimenez/d2go/pkg/data"
)

// CastSkillEntityLeft represents packet 0x06 for casting left-click skill on entity
// Format: [0x06][0x01 00 00 00][TargetGID:4bytes]
// Total: 9 bytes
//
// This packet casts the currently selected left-click skill on a specific entity (monster, NPC, object).
// The skill must already be bound to left-click before sending this packet.
// TargetGID is the Global Unit ID of the target entity.
type CastSkillEntityLeft struct {
	PacketID byte
	Action   uint32 // Always 0x00000001
	TargetID uint32
}

// NewCastSkillEntityLeft creates a new left-click entity skill cast packet
func NewCastSkillEntityLeft(targetID data.UnitID) *CastSkillEntityLeft {
	return &CastSkillEntityLeft{
		PacketID: 0x06,
		Action:   0x00000001,
		TargetID: uint32(targetID),
	}
}

// GetPayload serializes the packet to byte array
func (p *CastSkillEntityLeft) GetPayload() []byte {
	payload := make([]byte, 9)
	payload[0] = p.PacketID
	binary.LittleEndian.PutUint32(payload[1:5], p.Action)
	binary.LittleEndian.PutUint32(payload[5:9], p.TargetID)
	return payload
}
