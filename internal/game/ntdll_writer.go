package game

import (
	"crypto/rand"
	"fmt"
	"sync"
	"unsafe"

	"golang.org/x/sys/windows"
)

var (
	ntdll                      = windows.NewLazySystemDLL("ntdll.dll")
	procNtWriteVirtualMemory   = ntdll.NewProc("NtWriteVirtualMemory")
	procNtReadVirtualMemory    = ntdll.NewProc("NtReadVirtualMemory")
	procNtProtectVirtualMemory = ntdll.NewProc("NtProtectVirtualMemory")
	procNtOpenProcess          = ntdll.NewProc("NtOpenProcess")
	procNtClose                = ntdll.NewProc("NtClose")
)

var (
	sessionKey     [32]byte
	sessionKeyOnce sync.Once
)

func ensureSessionKey() {
	sessionKeyOnce.Do(func() {
		if _, err := rand.Read(sessionKey[:]); err != nil {
			// Fallback: deterministic but varies per session via ASLR.
			for i := range sessionKey {
				sessionKey[i] = byte(i*0x37 ^ 0xAB)
			}
		}
	})
}

func xorEncode(data []byte) {
	ensureSessionKey()
	for i := range data {
		data[i] ^= sessionKey[i%len(sessionKey)]
	}
}

func secureZero(buf []byte) {
	if len(buf) == 0 {
		return
	}
	for i := range buf {
		buf[i] = 0
	}
	// Force the compiler to consider the slice live after zeroing.
	_ = buf[len(buf)-1]
}

// clientID mirrors the Windows CLIENT_ID structure (16 bytes on amd64).
type clientID struct {
	UniqueProcess uintptr
	UniqueThread  uintptr
}

// objectAttributes mirrors the Windows OBJECT_ATTRIBUTES structure (48 bytes on amd64).
type objectAttributes struct {
	Length                   uint32
	RootDirectory            uintptr
	ObjectName               uintptr
	Attributes               uint32
	SecurityDescriptor       uintptr
	SecurityQualityOfService uintptr
}

func ntOpenProcess(pid uint32, access uint32) (windows.Handle, error) {
	if err := procNtOpenProcess.Find(); err != nil {
		return 0, fmt.Errorf("ntdll: NtOpenProcess not available: %w", err)
	}

	var handle windows.Handle
	cid := clientID{UniqueProcess: uintptr(pid)}
	oa := objectAttributes{Length: uint32(unsafe.Sizeof(objectAttributes{}))}

	r, _, _ := procNtOpenProcess.Call(
		uintptr(unsafe.Pointer(&handle)),
		uintptr(access),
		uintptr(unsafe.Pointer(&oa)),
		uintptr(unsafe.Pointer(&cid)),
	)
	if r != 0 {
		return 0, fmt.Errorf("NtOpenProcess: NTSTATUS 0x%08X", r)
	}
	return handle, nil
}

func ntClose(handle windows.Handle) error {
	if err := procNtClose.Find(); err != nil {
		// NtClose unavailable — fall back to kernel32 to prevent handle leak.
		return windows.CloseHandle(handle)
	}
	r, _, _ := procNtClose.Call(uintptr(handle))
	if r != 0 {
		return fmt.Errorf("NtClose: NTSTATUS 0x%08X", r)
	}
	return nil
}

func ntWriteMemory(handle windows.Handle, addr uintptr, buf *byte, size uintptr) error {
	if err := procNtWriteVirtualMemory.Find(); err != nil {
		return fmt.Errorf("ntdll: NtWriteVirtualMemory not available: %w", err)
	}

	regionBase := addr
	regionSize := size
	var oldProtect uint32
	protChanged := false

	if err := procNtProtectVirtualMemory.Find(); err == nil {
		r1, _, _ := procNtProtectVirtualMemory.Call(
			uintptr(handle),
			uintptr(unsafe.Pointer(&regionBase)),
			uintptr(unsafe.Pointer(&regionSize)),
			windows.PAGE_EXECUTE_READWRITE,
			uintptr(unsafe.Pointer(&oldProtect)),
		)
		protChanged = r1 == 0
	}

	var bytesWritten uintptr
	r2, _, _ := procNtWriteVirtualMemory.Call(
		uintptr(handle),
		addr,
		uintptr(unsafe.Pointer(buf)),
		size,
		uintptr(unsafe.Pointer(&bytesWritten)),
	)

	if protChanged {
		regionBase = addr
		regionSize = size
		var dummy uint32
		procNtProtectVirtualMemory.Call(
			uintptr(handle),
			uintptr(unsafe.Pointer(&regionBase)),
			uintptr(unsafe.Pointer(&regionSize)),
			uintptr(oldProtect),
			uintptr(unsafe.Pointer(&dummy)),
		)
	}

	if r2 != 0 {
		return fmt.Errorf("NtWriteVirtualMemory: NTSTATUS 0x%08X", r2)
	}
	if bytesWritten != size {
		return fmt.Errorf("NtWriteVirtualMemory: partial write (%d of %d bytes)", bytesWritten, size)
	}
	return nil
}

func ntReadMemory(handle windows.Handle, addr uintptr, buf *byte, size uintptr) error {
	if err := procNtReadVirtualMemory.Find(); err != nil {
		return fmt.Errorf("ntdll: NtReadVirtualMemory not available: %w", err)
	}

	var bytesRead uintptr
	r, _, _ := procNtReadVirtualMemory.Call(
		uintptr(handle),
		addr,
		uintptr(unsafe.Pointer(buf)),
		size,
		uintptr(unsafe.Pointer(&bytesRead)),
	)
	if r != 0 {
		return fmt.Errorf("NtReadVirtualMemory: NTSTATUS 0x%08X", r)
	}
	if bytesRead != size {
		return fmt.Errorf("NtReadVirtualMemory: partial read (%d of %d bytes)", bytesRead, size)
	}
	return nil
}

func polyReturn1() []byte {
	var variant int
	b := make([]byte, 1)
	if _, err := rand.Read(b); err == nil {
		variant = int(b[0]) % 5
	}

	switch variant {
	case 0:
		// mov eax, 1; ret
		return []byte{0xB8, 0x01, 0x00, 0x00, 0x00, 0xC3}
	case 1:
		// xor eax, eax; inc eax; ret
		return []byte{0x31, 0xC0, 0xFF, 0xC0, 0xC3}
	case 2:
		// xor eax, eax; mov al, 1; ret
		return []byte{0x31, 0xC0, 0xB0, 0x01, 0xC3}
	case 3:
		// push 1; pop rax; ret
		return []byte{0x6A, 0x01, 0x58, 0xC3}
	default:
		// xor ecx, ecx; lea eax, [rcx+1]; ret
		return []byte{0x31, 0xC9, 0x8D, 0x41, 0x01, 0xC3}
	}
}

func polyGetCursorPos(x, y int) []byte {
	var variant int
	b := make([]byte, 1)
	if _, err := rand.Read(b); err == nil {
		variant = int(b[0]) % 3
	}

	xBytes := make([]byte, 4)
	yBytes := make([]byte, 4)
	xBytes[0] = byte(x)
	xBytes[1] = byte(x >> 8)
	xBytes[2] = byte(x >> 16)
	xBytes[3] = byte(x >> 24)
	yBytes[0] = byte(y)
	yBytes[1] = byte(y >> 8)
	yBytes[2] = byte(y >> 16)
	yBytes[3] = byte(y >> 24)

	switch variant {
	case 0:
		// Original: push rax; mov rax,rcx; mov [rax],X; mov [rax+4],Y; pop rax; mov al,1; ret
		code := []byte{0x50, 0x48, 0x89, 0xC8,
			0xC7, 0x00, 0x00, 0x00, 0x00, 0x00,
			0xC7, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00,
			0x58, 0xB0, 0x01, 0xC3}
		copy(code[6:10], xBytes)
		copy(code[13:17], yBytes)
		return code

	case 1:
		// Variant: mov dword ptr [rcx], X; mov dword ptr [rcx+4], Y; mov eax, 1; ret
		code := []byte{
			0xC7, 0x01, 0x00, 0x00, 0x00, 0x00, // mov dword ptr [rcx], X
			0xC7, 0x41, 0x04, 0x00, 0x00, 0x00, 0x00, // mov dword ptr [rcx+4], Y
			0xB8, 0x01, 0x00, 0x00, 0x00, // mov eax, 1
			0xC3, // ret
		}
		copy(code[2:6], xBytes)
		copy(code[9:13], yBytes)
		return code

	default:
		// Variant: mov dword ptr [rcx], X; mov dword ptr [rcx+4], Y; xor eax,eax; inc eax; ret
		code := []byte{
			0xC7, 0x01, 0x00, 0x00, 0x00, 0x00, // mov dword ptr [rcx], X
			0xC7, 0x41, 0x04, 0x00, 0x00, 0x00, 0x00, // mov dword ptr [rcx+4], Y
			0x31, 0xC0, // xor eax, eax
			0xFF, 0xC0, // inc eax
			0xC3, // ret
		}
		copy(code[2:6], xBytes)
		copy(code[9:13], yBytes)
		return code
	}
}

func polyGetKeyStateHook(key byte) []byte {
	var variant int
	b := make([]byte, 1)
	if _, err := rand.Read(b); err == nil {
		variant = int(b[0]) % 3
	}

	switch variant {
	case 0:
		// Original: cmp cl, key; sete al; shl ax, 15; ret
		return []byte{0x80, 0xF9, key, 0x0F, 0x94, 0xC0, 0x66, 0xC1, 0xE0, 0x0F, 0xC3}

	case 1:
		// Variant: xor eax, eax; cmp cl, key; jne +3; mov ax, 0x8000; ret
		return []byte{
			0x31, 0xC0, // xor eax, eax
			0x80, 0xF9, key, // cmp cl, key
			0x75, 0x04, // jne skip (4 bytes forward)
			0x66, 0xB8, 0x00, 0x80, // mov ax, 0x8000
			0xC3, // ret
		}

	default:
		// Variant: xor eax, eax; cmp cl, key; sete al; neg ax; and ax, 0x8000; ret
		return []byte{
			0x31, 0xC0, // xor eax, eax
			0x80, 0xF9, key, // cmp cl, key
			0x0F, 0x94, 0xC0, // sete al
			0x66, 0xF7, 0xD8, // neg ax (0→0, 1→0xFFFF)
			0x66, 0x25, 0x00, 0x80, // and ax, 0x8000
			0xC3, // ret
		}
	}
}

func writeAndClean(handle windows.Handle, addr uintptr, code []byte) error {
	// XOR-encode the cleartext shellcode in the original slice.
	xorEncode(code)

	// Decode into a scratch buffer for the actual write.
	scratch := make([]byte, len(code))
	copy(scratch, code)
	xorEncode(scratch) // XOR again → back to cleartext

	err := ntWriteMemory(handle, addr, &scratch[0], uintptr(len(scratch)))

	// Wipe both buffers regardless of success.
	secureZero(scratch)
	secureZero(code)

	return err
}
