package utils

import (
	"strconv"
	"strings"
)

func ParsePercentOrDefault(raw string, fallback int) int {
	value, err := strconv.Atoi(strings.TrimSpace(raw))
	if err != nil || value < 1 || value > 100 {
		return fallback
	}
	return value
}
