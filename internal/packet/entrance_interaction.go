package packet

import (
	"encoding/binary"

	"github.com/hectorgimenez/d2go/pkg/data"
)

// EntranceInteraction represents the packet for interacting with entrances
// Packet structure discovered via memory scan (Blood Moor → Den of Evil):
// Size: 5 bytes
// Format: [PacketID:0x40][UnitID:uint32 little-endian]
type EntranceInteraction struct {
	PacketID byte
	UnitID   uint32
}

// NewEntranceInteraction creates an entrance interaction packet
// Example: Blood Moor → Den of Evil with UnitID 1585685452
// Returns bytes: [0x40, 0xCC, 0xA3, 0x83, 0x5E]
func NewEntranceInteraction(entrance data.Entrance) *EntranceInteraction {
	return &EntranceInteraction{
		PacketID: 0x40, // Discovered packet ID for entrance interaction
		UnitID:   uint32(entrance.ID),
	}
}

// GetPayload converts the EntranceInteraction struct to bytes
func (p *EntranceInteraction) GetPayload() []byte {
	buf := make([]byte, 5)
	buf[0] = byte(p.PacketID)
	binary.LittleEndian.PutUint32(buf[1:], p.UnitID)
	return buf
}
