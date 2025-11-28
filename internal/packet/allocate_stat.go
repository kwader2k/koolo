package packet

import (
	"encoding/binary"

	"github.com/hectorgimenez/d2go/pkg/data/stat"
)

// AllocateStat represents packet 0x3A for allocating stat points
// Format: [0x3A][StatID:2bytes][0x0000]
// Total: 5 bytes
//
// Stat IDs from sniffed data:
// 0x00 = Strength
// 0x01 = Energy
// 0x02 = Dexterity
// 0x03 = Vitality
type AllocateStat struct {
	PacketID byte
	StatID   uint16
	Padding  uint16
}

// statToPacketID maps d2go stat.ID to packet stat ID
func statToPacketID(s stat.ID) uint16 {
	switch s {
	case stat.Strength:
		return 0x00
	case stat.Energy:
		return 0x01
	case stat.Dexterity:
		return 0x02
	case stat.Vitality:
		return 0x03
	default:
		return 0x00 // Default to strength if unknown
	}
}

// NewAllocateStat creates a new stat allocation packet
func NewAllocateStat(statID stat.ID) *AllocateStat {
	return &AllocateStat{
		PacketID: 0x3A,
		StatID:   statToPacketID(statID),
		Padding:  0x0000,
	}
}

func (p *AllocateStat) GetPayload() []byte {
	buf := make([]byte, 5)
	buf[0] = p.PacketID
	binary.LittleEndian.PutUint16(buf[1:], p.StatID)
	binary.LittleEndian.PutUint16(buf[3:], p.Padding)
	return buf
}
