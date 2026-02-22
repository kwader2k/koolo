package utils

import (
	"os"
	"regexp"
)

func GetJsonData(filePath string) ([]byte, error) {
	//Check if file exists
	if _, err := os.Stat(filePath); err != nil {
		return []byte{}, err
	}

	//Read raw file data
	data, err := os.ReadFile(filePath)
	if err != nil {
		return data, err
	}

	//Remove C style comments
	re := regexp.MustCompile("(?s)//.*?\n|/\\*.*?\\*/")
	jsonData := re.ReplaceAll(data, nil)

	return jsonData, nil
}
