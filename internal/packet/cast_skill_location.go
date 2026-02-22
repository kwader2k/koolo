package packet

import (
	"encoding/binary"

	"github.com/hectorgimenez/d2go/pkg/data"
)

type CastSkillLocation struct {
	PacketID byte
	X        uint16
	Y        uint16
}

func NewCastSkillLocation(position data.Position) *CastSkillLocation {
	return &CastSkillLocation{
		PacketID: 0x0C,
		X:        uint16(position.X),
		Y:        uint16(position.Y),
	}
}

func NewTeleport(position data.Position) *CastSkillLocation {
	return NewCastSkillLocation(position)
}

func (p *CastSkillLocation) GetPayload() []byte {
	buf := make([]byte, 5)
	buf[0] = p.PacketID
	binary.LittleEndian.PutUint16(buf[1:], p.X)
	binary.LittleEndian.PutUint16(buf[3:], p.Y)
	return buf
}
