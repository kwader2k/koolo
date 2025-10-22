/**
 * Character Settings
 * Handles character configuration UI interactions
 */

// ========================================
// CONSTANTS
// ========================================
const SETTINGS_CONFIG = {
    LEVELING_PROFILES: ['paladin', 'sorceress_leveling', 'druid_leveling', 'necromancer', 'assassin'],
    LEVELING_CLASSES: ['paladin', 'sorceress_leveling', 'druid_leveling', 'necromancer', 'assassin']
};

// ========================================
// STATE MANAGEMENT
// ========================================
class SettingsState {
    constructor() {
        this.enabledRunsUl = null;
        this.disabledRunsUl = null;
        this.searchInput = null;
        this.runewordSearchInput = null;
    }

    init() {
        this.enabledRunsUl = document.getElementById('enabled_runs');
        this.disabledRunsUl = document.getElementById('disabled_runs');
        this.searchInput = document.getElementById('search-disabled-runs');
        this.runewordSearchInput = document.getElementById('search-runewords');
    }
}

// ========================================
// SORTABLE LISTS MANAGER
// ========================================
class SortableManager {
    static init(state) {
        this.state = state;
        this.setupSortables();
    }

    static setupSortables() {
        if (!this.state.enabledRunsUl || !this.state.disabledRunsUl) return;

        new Sortable(this.state.enabledRunsUl, {
            group: 'runs',
            animation: 150,
            onSort: () => RunsManager.updateEnabledRunsHiddenField(),
            onAdd: (evt) => RunsManager.updateButtonForEnabledRun(evt.item)
        });

        new Sortable(this.state.disabledRunsUl, {
            group: 'runs',
            animation: 150,
            onAdd: (evt) => RunsManager.updateButtonForDisabledRun(evt.item)
        });
    }
}

// ========================================
// RUNS MANAGER
// ========================================
class RunsManager {
    static updateEnabledRunsHiddenField() {
        const listItems = document.querySelectorAll('#enabled_runs li');
        const values = Array.from(listItems).map(item => item.getAttribute('value'));
        const hiddenField = document.getElementById('gameRuns');
        
        if (hiddenField) {
            hiddenField.value = JSON.stringify(values);
        }
    }

    static updateButtonForEnabledRun(runElement) {
        const button = runElement.querySelector('button');
        if (!button) return;

        button.classList.remove('add-run');
        button.classList.add('remove-run');
        button.title = 'Remove run';
        button.innerHTML = '<i class="bi bi-dash"></i>';
    }

    static updateButtonForDisabledRun(runElement) {
        const button = runElement.querySelector('button');
        if (!button) return;

        button.classList.remove('remove-run');
        button.classList.add('add-run');
        button.title = 'Add run';
        button.innerHTML = '<i class="bi bi-plus"></i>';
    }

    static moveRunToDisabled(runElement) {
        const disabledRunsUl = document.getElementById('disabled_runs');
        if (!disabledRunsUl) return;

        this.updateButtonForDisabledRun(runElement);
        disabledRunsUl.appendChild(runElement);
        this.updateEnabledRunsHiddenField();
    }

    static moveRunToEnabled(runElement) {
        const enabledRunsUl = document.getElementById('enabled_runs');
        if (!enabledRunsUl) return;

        this.updateButtonForEnabledRun(runElement);
        enabledRunsUl.appendChild(runElement);
        this.updateEnabledRunsHiddenField();
    }

    static clearEnabledRuns() {
        const enabledRunsUl = document.getElementById('enabled_runs');
        const disabledRunsUl = document.getElementById('disabled_runs');
        
        if (!enabledRunsUl || !disabledRunsUl) return;

        Array.from(enabledRunsUl.children).forEach(item => {
            this.updateButtonForDisabledRun(item);
            disabledRunsUl.appendChild(item);
        });

        this.updateEnabledRunsHiddenField();
    }

    static selectLevelingProfile() {
        const disabledRunsUl = document.getElementById('disabled_runs');
        const enabledRunsUl = document.getElementById('enabled_runs');
        
        if (!disabledRunsUl || !enabledRunsUl) return;

        const levelingRun = Array.from(disabledRunsUl.children).find(
            item => item.getAttribute('value') === 'leveling'
        );

        if (levelingRun) {
            this.updateButtonForEnabledRun(levelingRun);
            enabledRunsUl.appendChild(levelingRun);
            this.updateEnabledRunsHiddenField();
        }
    }
}

// ========================================
// SEARCH MANAGER
// ========================================
class SearchManager {
    static filterDisabledRuns(searchTerm) {
        const listItems = document.querySelectorAll('#disabled_runs li');
        const normalizedTerm = searchTerm.toLowerCase();

        listItems.forEach(item => {
            const runName = item.getAttribute('value')?.toLowerCase() || '';
            item.style.display = runName.includes(normalizedTerm) ? '' : 'none';
        });
    }

    static filterRunewords(searchTerm = '') {
        const listItems = document.querySelectorAll('.runeword-item');
        const normalizedTerm = searchTerm.toLowerCase();

        listItems.forEach(item => {
            const checkbox = item.querySelector('input[type="checkbox"]');
            const isChecked = checkbox?.checked || false;
            const nameElement = item.querySelector('.runeword-name');
            const rwName = nameElement?.textContent.toLowerCase() || '';

            // Show if checked OR if matches search term (or if no search term and checked)
            const shouldShow = isChecked || (normalizedTerm && rwName.includes(normalizedTerm));
            item.style.display = shouldShow ? '' : 'none';
        });
    }
}

// ========================================
// CHARACTER CLASS MANAGER
// ========================================
class CharacterClassManager {
    static init() {
        this.setupClassSpecificSettings();
        this.setupBuildValidation();
    }

    static setupClassSpecificSettings() {
        const characterClassSelect = document.querySelector('select[name="characterClass"]');
        const gameDifficultySelect = document.getElementById('gameDifficulty');

        if (characterClassSelect) {
            characterClassSelect.addEventListener('change', () => {
                this.updateCharacterOptions();
            });
        }

        if (gameDifficultySelect) {
            gameDifficultySelect.addEventListener('change', () => {
                const selectedClass = characterClassSelect?.value;
                if (selectedClass === 'nova' || selectedClass === 'lightsorc') {
                    this.updateNovaSorceressOptions();
                }
            });
        }

        // Initialize on load
        this.updateCharacterOptions();
    }

    static updateCharacterOptions() {
        const characterClassSelect = document.querySelector('select[name="characterClass"]');
        const selectedClass = characterClassSelect?.value;

        const noSettingsMessage = document.getElementById('no-settings-message');
        const berserkerBarbOptions = document.querySelector('.berserker-barb-options');
        const novaSorceressOptions = document.querySelector('.nova-sorceress-options');
        const mosaicAssassinOptions = document.querySelector('.mosaic-assassin-options');

        // Hide all options first
        [berserkerBarbOptions, novaSorceressOptions, mosaicAssassinOptions, noSettingsMessage]
            .forEach(el => {
                if (el) el.style.display = 'none';
            });

        // Show relevant options based on class
        if (selectedClass === 'berserker' && berserkerBarbOptions) {
            berserkerBarbOptions.style.display = 'block';
        } else if ((selectedClass === 'nova' || selectedClass === 'lightsorc') && novaSorceressOptions) {
            novaSorceressOptions.style.display = 'block';
            this.updateNovaSorceressOptions();
        } else if (selectedClass === 'mosaic' && mosaicAssassinOptions) {
            mosaicAssassinOptions.style.display = 'block';
        } else if (noSettingsMessage) {
            noSettingsMessage.style.display = 'block';
        }
    }

    static updateNovaSorceressOptions() {
        const gameDifficultySelect = document.getElementById('gameDifficulty');
        const selectedDifficulty = gameDifficultySelect?.value;
        
        if (selectedDifficulty) {
            this.updateBossStaticThresholdMin(selectedDifficulty);
            this.handleBossStaticThresholdChange();
        }
    }

    static updateBossStaticThresholdMin(difficulty) {
        const input = document.getElementById('novaBossStaticThreshold');
        if (!input) return;

        const minValues = {
            'normal': 1,
            'nightmare': 33,
            'hell': 50
        };

        const minValue = minValues[difficulty] || 65;
        input.min = minValue;

        // Ensure current value is not less than minimum
        if (parseInt(input.value) < minValue) {
            input.value = minValue;
        }
    }

    static handleBossStaticThresholdChange() {
        const input = document.getElementById('novaBossStaticThreshold');
        const gameDifficultySelect = document.getElementById('gameDifficulty');
        
        if (!input || !gameDifficultySelect) return;

        const selectedDifficulty = gameDifficultySelect.value;
        const minValues = {
            'normal': 1,
            'nightmare': 33,
            'hell': 50
        };

        const minValue = minValues[selectedDifficulty] || 65;
        let value = parseInt(input.value);

        if (isNaN(value) || value < minValue) {
            value = minValue;
        } else if (value > 100) {
            value = 100;
        }

        input.value = value;
    }

    static setupBuildValidation() {
        const buildSelectElement = document.querySelector('select[name="characterClass"]');
        if (!buildSelectElement) return;

        buildSelectElement.addEventListener('change', () => {
            const selectedBuild = buildSelectElement.value;
            const enabledRunListElement = document.getElementById('enabled_runs');
            
            if (!enabledRunListElement) return;

            const enabledRuns = Array.from(enabledRunListElement.querySelectorAll('li'))
                .map(li => li.getAttribute('value'));

            const isLevelingRunEnabled = enabledRuns.includes('leveling');
            const hasOtherRunsEnabled = enabledRuns.length > 1;

            if (SETTINGS_CONFIG.LEVELING_CLASSES.includes(selectedBuild) && 
                (!isLevelingRunEnabled || hasOtherRunsEnabled)) {
                alert("This profile requires enabling the leveling run. Please add only the 'leveling' run to the enabled run list and remove the others.");
            }
        });
    }
}

// ========================================
// TELEPORT/CLEAR PATH MANAGER
// ========================================
class TeleportManager {
    static init() {
        this.setupTeleportToggle();
        this.setupClearPathSlider();
    }

    static setupTeleportToggle() {
        const useTeleportCheckbox = document.getElementById('characterUseTeleport');
        const clearPathDistContainer = document.getElementById('clearPathDistContainer');

        if (!useTeleportCheckbox || !clearPathDistContainer) return;

        const toggleVisibility = () => {
            clearPathDistContainer.style.display = useTeleportCheckbox.checked ? 'none' : 'block';
        };

        useTeleportCheckbox.addEventListener('change', toggleVisibility);
        toggleVisibility(); // Initialize
    }

    static setupClearPathSlider() {
        const clearPathDistInput = document.getElementById('clearPathDist');
        const clearPathDistValue = document.getElementById('clearPathDistValue');

        if (!clearPathDistInput || !clearPathDistValue) return;

        const updateValue = () => {
            clearPathDistValue.textContent = clearPathDistInput.value;
        };

        clearPathDistInput.addEventListener('input', updateValue);
        updateValue(); // Initialize
    }
}

// ========================================
// SCHEDULER MANAGER
// ========================================
class SchedulerManager {
    static init() {
        this.setupSchedulerToggle();
        this.setupTimeRanges();
    }

    static setupSchedulerToggle() {
        const schedulerEnabled = document.querySelector('input[name="schedulerEnabled"]');
        const schedulerSettings = document.getElementById('scheduler-settings');

        if (!schedulerEnabled || !schedulerSettings) return;

        const toggleVisibility = () => {
            schedulerSettings.style.display = schedulerEnabled.checked ? 'grid' : 'none';
        };

        schedulerEnabled.addEventListener('change', toggleVisibility);
        toggleVisibility(); // Initialize
    }

    static setupTimeRanges() {
        // Add time range buttons
        document.querySelectorAll('.add-time-range').forEach(button => {
            button.addEventListener('click', (e) => {
                const day = e.target.dataset.day;
                this.addTimeRange(day, e.target);
            });
        });

        // Remove time range buttons (delegated)
        document.addEventListener('click', (e) => {
            if (e.target.closest('.remove-time-range')) {
                e.target.closest('.time-range').remove();
            }
        });
    }

    static addTimeRange(day, button) {
        const timeRangesDiv = button.previousElementSibling;
        if (!timeRangesDiv) return;

        const newTimeRange = document.createElement('div');
        newTimeRange.className = 'time-range';
        newTimeRange.innerHTML = `
            <input type="time" name="scheduler[${day}][start][]" required>
            <span>to</span>
            <input type="time" name="scheduler[${day}][end][]" required>
            <button type="button" class="remove-time-range"><i class="bi bi-trash"></i></button>
        `;
        timeRangesDiv.appendChild(newTimeRange);
    }
}

// ========================================
// TERROR ZONE MANAGER
// ========================================
class TerrorZoneManager {
    static init() {
        const tzTrackAll = document.getElementById('tzTrackAll');
        if (!tzTrackAll) return;

        tzTrackAll.addEventListener('change', (e) => {
            document.querySelectorAll('.tzTrackCheckbox').forEach(checkbox => {
                checkbox.checked = e.target.checked;
            });
        });
    }
}

// ========================================
// EVENT HANDLERS
// ========================================
class EventHandlers {
    static init(state) {
        this.state = state;
        this.setupAllHandlers();
    }

    static setupAllHandlers() {
        // Run list add/remove buttons
        document.addEventListener('click', (e) => {
            if (e.target.closest('.remove-run')) {
                e.preventDefault();
                const runElement = e.target.closest('li');
                RunsManager.moveRunToDisabled(runElement);
            } else if (e.target.closest('.add-run')) {
                e.preventDefault();
                const runElement = e.target.closest('li');
                RunsManager.moveRunToEnabled(runElement);
            }
        });

        // Search inputs
        if (this.state.searchInput) {
            this.state.searchInput.addEventListener('input', (e) => {
                SearchManager.filterDisabledRuns(e.target.value);
            });
        }

        if (this.state.runewordSearchInput) {
            this.state.runewordSearchInput.addEventListener('input', (e) => {
                SearchManager.filterRunewords(e.target.value);
            });

            // Also filter when checkboxes change
            document.addEventListener('change', (e) => {
                if (e.target.matches('.runeword-item input[type="checkbox"]')) {
                    SearchManager.filterRunewords(this.state.runewordSearchInput.value);
                }
            });

            // Initialize filter
            SearchManager.filterRunewords();
        }

        // Boss static threshold input
        const bossStaticThresholdInput = document.getElementById('novaBossStaticThreshold');
        if (bossStaticThresholdInput) {
            bossStaticThresholdInput.addEventListener('input', () => {
                CharacterClassManager.handleBossStaticThresholdChange();
            });
        }
    }
}

// ========================================
// FORM VALIDATION
// ========================================
class FormValidation {
    static checkLevelingProfile() {
        const characterClass = document.getElementById('characterClass')?.value;
        
        if (!characterClass) return;

        if (SETTINGS_CONFIG.LEVELING_PROFILES.includes(characterClass)) {
            const confirmation = confirm(
                "This profile requires the leveling run profile. Would you like to clear enabled run profiles and select the leveling profile?"
            );
            
            if (confirmation) {
                RunsManager.clearEnabledRuns();
                RunsManager.selectLevelingProfile();
            }
        }
    }
}

// ========================================
// INITIALIZATION
// ========================================
const settingsState = new SettingsState();

window.addEventListener('load', () => {
    // Initialize state
    settingsState.init();

    // Initialize all managers
    SortableManager.init(settingsState);
    CharacterClassManager.init();
    TeleportManager.init();
    SchedulerManager.init();
    TerrorZoneManager.init();
    EventHandlers.init(settingsState);

    // Update initial hidden field
    RunsManager.updateEnabledRunsHiddenField();
});

// Expose global functions for potential inline usage
window.checkLevelingProfile = () => FormValidation.checkLevelingProfile();
window.handleBossStaticThresholdChange = () => CharacterClassManager.handleBossStaticThresholdChange();
