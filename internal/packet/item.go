package packet

import (
	"encoding/binary"

	"github.com/hectorgimenez/d2go/pkg/data"
)

type PickItem struct {
	PacketID PacketID
	ItemGUID uint32
	X        uint32
	Y        uint32
	Action   uint32
}

func NewPickItem(item data.Item) *PickItem {
	return &PickItem{
		PacketID: PickItemPacketId,
		ItemGUID: uint32(item.UnitID),
		X:        uint32(item.Position.X),
		Y:        uint32(item.Position.Y),
		Action:   1,
	}
}

func (p *PickItem) GetPayload() []byte {
	buf := make([]byte, 17)
	buf[0] = byte(p.PacketID)
	binary.LittleEndian.PutUint32(buf[1:], p.ItemGUID)
	binary.LittleEndian.PutUint32(buf[5:], p.X)
	binary.LittleEndian.PutUint32(buf[9:], p.Y)
	binary.LittleEndian.PutUint32(buf[13:], uint32(p.Action))
	return buf
}
