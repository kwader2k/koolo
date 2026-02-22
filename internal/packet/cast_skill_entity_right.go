package packet

import (
	"encoding/binary"

	"github.com/hectorgimenez/d2go/pkg/data"
)

// CastSkillEntityRight represents packet 0x0D for casting right-click skill on entity
// Format: [0x0D][0x01 00 00 00][TargetGID:4bytes]
// Total: 9 bytes
//
// This packet casts the currently selected right-click skill on a specific entity (monster, NPC, object).
// The skill must already be bound to right-click before sending this packet.
// TargetGID is the Global Unit ID of the target entity.
type CastSkillEntityRight struct {
	PacketID byte
	Action   uint32 // Always 0x00000001
	TargetID uint32
}

// NewCastSkillEntityRight creates a new right-click entity skill cast packet
func NewCastSkillEntityRight(targetID data.UnitID) *CastSkillEntityRight {
	return &CastSkillEntityRight{
		PacketID: 0x0D,
		Action:   0x00000001,
		TargetID: uint32(targetID),
	}
}

// GetPayload serializes the packet to byte array
func (p *CastSkillEntityRight) GetPayload() []byte {
	payload := make([]byte, 9)
	payload[0] = p.PacketID
	binary.LittleEndian.PutUint32(payload[1:5], p.Action)
	binary.LittleEndian.PutUint32(payload[5:9], p.TargetID)
	return payload
}
