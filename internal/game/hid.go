package game

type HID struct {
	gr *MemoryReader
	gi *MemoryInjector
}

func NewHID(gr *MemoryReader, gi *MemoryInjector) *HID {
	return &HID{
		gr: gr,
		gi: gi,
	}
}

// InjectorActive reports whether the memory injector is loaded and the cursor
// override is operational. Movement helpers use this to bail out early when
// the injector is disabled (e.g. during pause).
func (hid *HID) InjectorActive() bool {
	return hid.gi.CursorOverrideActive()
}
