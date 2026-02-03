package server

import (
	"encoding/json"
	"fmt"
	"net/http"

	"github.com/hectorgimenez/koolo/internal/bot"
)

// armoryPage serves the armory HTML page
func (s *HttpServer) armoryPage(w http.ResponseWriter, r *http.Request) {
	characterName := r.URL.Query().Get("character")
	if characterName == "" {
		// Show armory selection page
		s.templates.ExecuteTemplate(w, "armory.gohtml", map[string]interface{}{
			"Characters": s.manager.AvailableSupervisors(),
		})
		return
	}

	// Load armory data for specific character
	armory, err := bot.LoadArmoryData(characterName)
	if err != nil {
		s.templates.ExecuteTemplate(w, "armory.gohtml", map[string]interface{}{
			"Characters": s.manager.AvailableSupervisors(),
			"Error":      fmt.Sprintf("No armory data found for %s. Start the character in a game first.", characterName),
			"Character":  characterName,
		})
		return
	}

	s.templates.ExecuteTemplate(w, "armory.gohtml", map[string]interface{}{
		"Characters": s.manager.AvailableSupervisors(),
		"Armory":     armory,
		"Character":  characterName,
	})
}

// armoryAPI returns armory data as JSON
func (s *HttpServer) armoryAPI(w http.ResponseWriter, r *http.Request) {
	characterName := r.URL.Query().Get("character")
	if characterName == "" {
		http.Error(w, "character parameter is required", http.StatusBadRequest)
		return
	}

	armory, err := bot.LoadArmoryData(characterName)
	if err != nil {
		http.Error(w, err.Error(), http.StatusNotFound)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(armory)
}

// armoryCharactersAPI returns list of characters with armory data
func (s *HttpServer) armoryCharactersAPI(w http.ResponseWriter, r *http.Request) {
	supervisors := s.manager.AvailableSupervisors()
	charactersWithArmory := make([]map[string]interface{}, 0)

	for _, name := range supervisors {
		armory, err := bot.LoadArmoryData(name)
		hasArmory := err == nil

		charInfo := map[string]interface{}{
			"name":      name,
			"hasArmory": hasArmory,
		}

		if hasArmory {
			charInfo["level"] = armory.Level
			charInfo["class"] = armory.Class
			charInfo["dumpTime"] = armory.DumpTime
		}

		charactersWithArmory = append(charactersWithArmory, charInfo)
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(charactersWithArmory)
}
