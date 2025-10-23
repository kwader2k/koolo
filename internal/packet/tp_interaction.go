package packet

import (
	"encoding/binary"

	"github.com/hectorgimenez/d2go/pkg/data"
)

type TpInteraction struct {
	PacketID byte
	ID       uint32
	Action   uint32
	Padding  uint32
}

func NewTpInteraction(object data.Object) *TpInteraction {
	return &TpInteraction{
		PacketID: 0x41,
		ID:       uint32(object.ID),
		Action:   2,
		Padding:  ^uint32(0), // 0xFFFFFFFF
	}
}

func (p *TpInteraction) GetPayload() []byte {
	buf := make([]byte, 13)
	buf[0] = byte(p.PacketID)
	binary.LittleEndian.PutUint32(buf[1:], p.ID)
	binary.LittleEndian.PutUint32(buf[5:], p.Action)
	binary.LittleEndian.PutUint32(buf[9:], p.Padding)
	return buf
}
