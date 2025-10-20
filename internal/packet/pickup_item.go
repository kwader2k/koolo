package packet

import (
	"encoding/binary"

	"github.com/hectorgimenez/d2go/pkg/data"
)

type PickUpItem struct {
	PacketID byte
	ItemGUID uint32
	X        uint32
	Y        uint32
	Action   uint32
}

func NewPickUpItem(item data.Item) *PickUpItem {
	return &PickUpItem{
		PacketID: 0x16,
		ItemGUID: uint32(item.UnitID),
		X:        uint32(item.Position.X),
		Y:        uint32(item.Position.Y),
		Action:   1,
	}
}

func (p *PickUpItem) GetPayload() []byte {
	buf := make([]byte, 17)
	buf[0] = byte(p.PacketID)
	binary.LittleEndian.PutUint32(buf[1:], p.ItemGUID)
	binary.LittleEndian.PutUint32(buf[5:], p.X)
	binary.LittleEndian.PutUint32(buf[9:], p.Y)
	binary.LittleEndian.PutUint32(buf[13:], uint32(p.Action))
	return buf
}
