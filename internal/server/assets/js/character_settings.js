window.onload = function () {
    let enabled_runs_ul = document.getElementById('enabled_runs');
    let disabled_runs_ul = document.getElementById('disabled_runs');
    let searchInput = document.getElementById('search-disabled-runs');

    new Sortable(enabled_runs_ul, {
        group: 'runs',
        animation: 150,
        onSort: function (evt) {
            updateEnabledRunsHiddenField();
        },
        onAdd: function (evt) {
            updateButtonForEnabledRun(evt.item);
        }
    });

    new Sortable(disabled_runs_ul, {
        group: 'runs',
        animation: 150,
        onAdd: function (evt) {
            updateButtonForDisabledRun(evt.item);
        }
    });

    searchInput.addEventListener('input', function () {
        filterDisabledRuns(searchInput.value);
    });

    // Add event listeners for add and remove buttons
    document.addEventListener('click', function (e) {
        if (e.target.closest('.remove-run')) {
            e.preventDefault();
            const runElement = e.target.closest('li');
            moveRunToDisabled(runElement);
        } else if (e.target.closest('.add-run')) {
            e.preventDefault();
            const runElement = e.target.closest('li');
            moveRunToEnabled(runElement);
        }
    });

    updateEnabledRunsHiddenField();

    const buildSelectElement = document.querySelector('select[name="characterClass"]');
    buildSelectElement.addEventListener('change', function () {
        const selectedBuild = buildSelectElement.value;
        const levelingBuilds = ['paladin', 'sorceress_leveling', 'druid_leveling', 'amazon_leveling', 'necromancer', 'assassin'];

        const enabledRunListElement = document.getElementById('enabled_runs');
        if (!enabledRunListElement) return;

        const enabledRuns = Array.from(enabledRunListElement.querySelectorAll('li')).map(li => li.getAttribute('value'));
        const isLevelingRunEnabled = enabledRuns.includes('leveling') || enabledRuns.includes('leveling_sequence');
        const hasOtherRunsEnabled = enabledRuns.length > 1;

        if (levelingBuilds.includes(selectedBuild) && (!isLevelingRunEnabled || hasOtherRunsEnabled)) {
            alert("This profile requires enabling the leveling run. Please add only the 'leveling' run to the enabled run list and remove the others.");
        }
    });
}

function updateEnabledRunsHiddenField() {
    let listItems = document.querySelectorAll('#enabled_runs li');
    let values = Array.from(listItems).map(function (item) {
        return item.getAttribute("value");
    });
    document.getElementById('gameRuns').value = JSON.stringify(values);
}

function filterDisabledRuns(searchTerm) {
    let listItems = document.querySelectorAll('#disabled_runs li');
    searchTerm = searchTerm.toLowerCase();
    listItems.forEach(function (item) {
        let runName = item.getAttribute("value").toLowerCase();
        if (runName.includes(searchTerm)) {
            item.style.display = '';
        } else {
            item.style.display = 'none';
        }
    });
}

function checkLevelingProfile() {
    const levelingProfiles = [
        "sorceress_leveling",
        "paladin",
        "druid_leveling",
        "amazon_leveling",
        "necromancer",
        "assassin"
    ];

    const characterClass = document.getElementById('characterClass').value;

    if (levelingProfiles.includes(characterClass)) {
        const confirmation = confirm("This profile requires the leveling run profile, would you like to clear enabled run profiles and select the leveling profile?");
        if (confirmation) {
            clearEnabledRuns();
            selectLevelingProfile();
        }
    }
}

function moveRunToDisabled(runElement) {
    const disabledRunsUl = document.getElementById('disabled_runs');
    updateButtonForDisabledRun(runElement);
    disabledRunsUl.appendChild(runElement);
    updateEnabledRunsHiddenField();
}

function moveRunToEnabled(runElement) {
    const enabledRunsUl = document.getElementById('enabled_runs');
    updateButtonForEnabledRun(runElement);
    enabledRunsUl.appendChild(runElement);
    updateEnabledRunsHiddenField();
}

function updateButtonForEnabledRun(runElement) {
    const button = runElement.querySelector('button');
    button.classList.remove('add-run');
    button.classList.add('remove-run');
    button.title = "Remove run";
    button.innerHTML = '<i class="bi bi-dash"></i>';
}

function updateButtonForDisabledRun(runElement) {
    const button = runElement.querySelector('button');
    button.classList.remove('remove-run');
    button.classList.add('add-run');
    button.title = "Add run";
    button.innerHTML = '<i class="bi bi-plus"></i>';
}

document.addEventListener('DOMContentLoaded', function () {
    const schedulerEnabled = document.querySelector('input[name="schedulerEnabled"]');
    const schedulerSettings = document.getElementById('scheduler-settings');
    const characterClassSelect = document.querySelector('select[name="characterClass"]');
    const berserkerBarbOptions = document.querySelector('.berserker-barb-options');
    const novaSorceressOptions = document.querySelector('.nova-sorceress-options');
    const bossStaticThresholdInput = document.getElementById('novaBossStaticThreshold');
    const mosaicAssassinOptions = document.querySelector('.mosaic-assassin-options');
    const blizzardSorceressOptions = document.querySelector('.blizzard-sorceress-options');
    const sorceressLevelingOptions = document.querySelector('.sorceress_leveling-options');
    const runewordSearchInput = document.getElementById('search-runewords');
    const useTeleportCheckbox = document.getElementById('characterUseTeleport');
    const useExtraBuffsCheckbox = document.getElementById('characterUseExtraBuffs');
    const clearPathDistContainer = document.getElementById('clearPathDistContainer');
    const useExtraBuffsDistContainer = document.getElementById('useExtraBuffsDistContainer');
    const clearPathDistInput = document.getElementById('clearPathDist');
    const clearPathDistValue = document.getElementById('clearPathDistValue');

    if (bossStaticThresholdInput) {
        bossStaticThresholdInput.addEventListener('input', handleBossStaticThresholdChange);
    }

    function toggleSchedulerVisibility() {
        schedulerSettings.style.display = schedulerEnabled.checked ? 'grid' : 'none';
    }

    function updateCharacterOptions() {
        const selectedClass = characterClassSelect.value;
        const noSettingsMessage = document.getElementById('no-settings-message');
        const berserkerBarbOptions = document.querySelector('.berserker-barb-options');
        const novaSorceressOptions = document.querySelector('.nova-sorceress-options');
        const mosaicAssassinOptions = document.querySelector('.mosaic-assassin-options');
        const blizzardSorceressOptions = document.querySelector('.blizzard-sorceress-options');
        const sorceressLevelingOptions = document.querySelector('.sorceress_leveling-options');
        // Hide all options first
        berserkerBarbOptions.style.display = 'none';
        novaSorceressOptions.style.display = 'none';
        mosaicAssassinOptions.style.display = 'none';
        blizzardSorceressOptions.style.display = 'none';
        sorceressLevelingOptions.style.display = 'none';
        noSettingsMessage.style.display = 'none';

        // Show relevant options based on class
        if (selectedClass === 'berserker') {
            berserkerBarbOptions.style.display = 'block';
        } else if (selectedClass === 'nova' || selectedClass === 'lightsorc') {
            novaSorceressOptions.style.display = 'block';
            updateNovaSorceressOptions();
        } else if (selectedClass === 'mosaic') {
            mosaicAssassinOptions.style.display = 'block';
        } else if (selectedClass === 'sorceress') {
            blizzardSorceressOptions.style.display = 'block';
        } else if (selectedClass === 'sorceress_leveling') {
            sorceressLevelingOptions.style.display = 'block';
        } else {
            noSettingsMessage.style.display = 'block';
        }
    }
    function toggleClearPathVisibility() {
        if (useTeleportCheckbox && clearPathDistContainer) {
            if (useTeleportCheckbox.checked) {
                clearPathDistContainer.style.display = 'none';
            } else {
                clearPathDistContainer.style.display = 'block';
            }
        }
    }
    function toggleUseExtraBuffsVisibility() {
        if (useExtraBuffsCheckbox && useExtraBuffsDistContainer) {
            if (useExtraBuffsCheckbox.checked) {
                useExtraBuffsDistContainer.style.display = 'block';
            } else {
                useExtraBuffsDistContainer.style.display = 'none';
            }
        }
    }

    // Update the displayed value when the slider changes
    function updateClearPathValue() {
        if (clearPathDistInput && clearPathDistValue) {
            clearPathDistValue.textContent = clearPathDistInput.value;

            // Calculate tooltip position based on slider value
            const min = parseFloat(clearPathDistInput.min);
            const max = parseFloat(clearPathDistInput.max);
            const value = parseFloat(clearPathDistInput.value);
            const percentage = ((value - min) / (max - min)) * 100;

            // Position the tooltip above the thumb
            clearPathDistValue.style.left = `calc(${percentage}% + (${8 - percentage * 0.15}px))`;
        }
    }

    // Show/hide tooltip on mouse interaction
    function showClearPathTooltip() {
        if (clearPathDistValue) {
            clearPathDistValue.style.opacity = '1';
            clearPathDistValue.style.pointerEvents = 'none';
        }
    }

    function hideClearPathTooltip() {
        if (clearPathDistValue) {
            clearPathDistValue.style.opacity = '0';
        }
    }

    // Set up event listeners
    if (useTeleportCheckbox) {
        useTeleportCheckbox.addEventListener('change', toggleClearPathVisibility);
        // Initialize visibility
        toggleClearPathVisibility();
    }

    // Set up event listeners
    if (useExtraBuffsCheckbox) {
        useExtraBuffsCheckbox.addEventListener('change', toggleUseExtraBuffsVisibility);
        // Initialize visibility
        toggleUseExtraBuffsVisibility();
    }

    if (clearPathDistInput) {
        clearPathDistInput.addEventListener('input', updateClearPathValue);
        clearPathDistInput.addEventListener('mousedown', showClearPathTooltip);
        clearPathDistInput.addEventListener('mouseup', hideClearPathTooltip);
        clearPathDistInput.addEventListener('mouseleave', hideClearPathTooltip);
        // Initialize value display and hide tooltip
        updateClearPathValue();
        hideClearPathTooltip();
    }

    function updateNovaSorceressOptions() {
        const selectedDifficulty = document.getElementById('gameDifficulty').value;
        updateBossStaticThresholdMin(selectedDifficulty);
        handleBossStaticThresholdChange();
    }

    function updateBossStaticThresholdMin(difficulty) {
        const input = document.getElementById('novaBossStaticThreshold');
        let minValue;
        switch (difficulty) {
            case 'normal':
                minValue = 1;
                break;
            case 'nightmare':
                minValue = 33;
                break;
            case 'hell':
                minValue = 50;
                break;
            default:
                minValue = 65;
        }
        input.min = minValue;

        // Ensure the current value is not less than the new minimum
        if (parseInt(input.value) < minValue) {
            input.value = minValue;
        }
    }

    characterClassSelect.addEventListener('change', updateCharacterOptions);
    document.getElementById('gameDifficulty').addEventListener('change', function () {
        if (characterClassSelect.value === 'nova' || characterClassSelect.value === 'lightsorc') {
            updateNovaSorceressOptions();
        }
    });

    characterClassSelect.addEventListener('change', updateCharacterOptions);
    updateCharacterOptions(); // Call this initially to set the correct state

    // Set initial state
    toggleSchedulerVisibility();
    updateNovaSorceressOptions();

    schedulerEnabled.addEventListener('change', toggleSchedulerVisibility);

    document.querySelectorAll('.add-time-range').forEach(button => {
        button.addEventListener('click', function () {
            const day = this.dataset.day;
            const timeRangesDiv = this.previousElementSibling;
            if (timeRangesDiv) {
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
        });
    });

    document.addEventListener('click', function (e) {
        if (e.target.closest('.remove-time-range')) {
            e.target.closest('.time-range').remove();
        }
    });

    function filterRunewords(searchTerm = '') { // Default parameter to ensure previously checked runewords show before searching
        let listItems = document.querySelectorAll('.runeword-item');
        searchTerm = searchTerm.toLowerCase();

        listItems.forEach(function (item) {
            const isChecked = item.querySelector('input[type="checkbox"]').checked;
            const rwName = item.querySelector('.runeword-name').textContent.toLowerCase();

            if (isChecked || (searchTerm && rwName.includes(searchTerm))) {
                item.style.display = '';
            } else {
                item.style.display = 'none';
            }
        });
    }

    if (runewordSearchInput) {
        runewordSearchInput.addEventListener('input', function () {
            filterRunewords(runewordSearchInput.value);
        });

        document.addEventListener('change', function (e) {
            if (e.target.matches('.runeword-item input[type="checkbox"]')) {
                filterRunewords(runewordSearchInput.value);
            }
        });

        filterRunewords();
    }

    const levelingSequenceSelect = document.getElementById('gameLevelingSequenceSelect');
    const levelingSequenceAddBtn = document.getElementById('levelingSequenceAddBtn');
    const levelingSequenceEditBtn = document.getElementById('levelingSequenceEditBtn');
    const levelingSequenceDeleteBtn = document.getElementById('levelingSequenceDeleteBtn');
    const LAST_SEQUENCE_KEY = 'koolo:lastSequenceName';
    const REFRESH_FLAG_KEY = 'koolo:sequenceRefreshRequired';
    const sequenceFilesEndpoint = '/api/sequence-editor/files';
    const sequenceDeleteEndpoint = '/api/sequence-editor/delete';

    const updateLevelingSequenceActionState = () => {
        const hasSelection = Boolean(levelingSequenceSelect && levelingSequenceSelect.value);
        if (levelingSequenceEditBtn) {
            levelingSequenceEditBtn.disabled = !hasSelection;
        }
        if (levelingSequenceDeleteBtn) {
            levelingSequenceDeleteBtn.disabled = !hasSelection;
        }
    };

    const rebuildLevelingSequenceOptions = (files, desiredSelection) => {
        if (!levelingSequenceSelect) {
            return;
        }

        const fragment = document.createDocumentFragment();
        const placeholder = document.createElement('option');
        placeholder.value = '';
        placeholder.disabled = true;
        placeholder.textContent = 'Select a sequence file';
        if (!desiredSelection) {
            placeholder.selected = true;
        }
        fragment.appendChild(placeholder);

        const hasDesired = desiredSelection && files.includes(desiredSelection);

        if (desiredSelection && !hasDesired) {
            const missingOption = document.createElement('option');
            missingOption.value = desiredSelection;
            missingOption.textContent = `${desiredSelection} (missing)`;
            missingOption.selected = true;
            fragment.appendChild(missingOption);
        }

        files.forEach((fileName) => {
            const option = document.createElement('option');
            option.value = fileName;
            option.textContent = fileName;
            if (fileName === desiredSelection) {
                option.selected = true;
            }
            fragment.appendChild(option);
        });

        levelingSequenceSelect.innerHTML = '';
        levelingSequenceSelect.appendChild(fragment);

        if (desiredSelection && hasDesired) {
            levelingSequenceSelect.value = desiredSelection;
        }
    };

    const refreshLevelingSequenceOptions = async (preferredSelection) => {
        if (!levelingSequenceSelect) {
            return false;
        }

        const targetSelection = typeof preferredSelection === 'string' ? preferredSelection : levelingSequenceSelect.value;

        try {
            const response = await fetch(sequenceFilesEndpoint, {
                headers: { 'Accept': 'application/json' },
            });
            if (!response.ok) {
                throw new Error(`Failed to fetch sequence files (${response.status})`);
            }
            const payload = await response.json();
            const files = Array.isArray(payload.files) ? payload.files : [];
            rebuildLevelingSequenceOptions(files, targetSelection);
            updateLevelingSequenceActionState();
            return true;
        } catch (error) {
            console.error('Unable to refresh leveling sequence list', error);
            return false;
        }
    };

    const maybeRefreshSequencesFromStorage = async () => {
        if (!levelingSequenceSelect || !window.localStorage) {
            return;
        }

        let refreshFlag;
        try {
            refreshFlag = window.localStorage.getItem(REFRESH_FLAG_KEY);
        } catch (error) {
            console.warn('Unable to read sequence refresh flag', error);
            return;
        }

        if (!refreshFlag) {
            return;
        }

        let desiredSelection = '';
        try {
            desiredSelection = window.localStorage.getItem(LAST_SEQUENCE_KEY) || '';
        } catch (error) {
            console.warn('Unable to read last sequence name', error);
        }

        const refreshed = await refreshLevelingSequenceOptions(desiredSelection);
        if (refreshed) {
            try {
                window.localStorage.removeItem(REFRESH_FLAG_KEY);
                if (desiredSelection) {
                    window.localStorage.removeItem(LAST_SEQUENCE_KEY);
                }
            } catch (error) {
                console.warn('Unable to clear sequence refresh flags', error);
            }
        }
    };

    if (levelingSequenceSelect) {
        levelingSequenceSelect.addEventListener('change', updateLevelingSequenceActionState);
    }
    if (levelingSequenceDeleteBtn) {
        levelingSequenceDeleteBtn.addEventListener('click', async () => {
            if (!levelingSequenceSelect || !levelingSequenceSelect.value) {
                return;
            }

            const targetName = levelingSequenceSelect.value;
            const confirmed = window.confirm(`Delete "${targetName}"? This cannot be undone.`);
            if (!confirmed) {
                return;
            }

            try {
                const response = await fetch(sequenceDeleteEndpoint, {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                        'Accept': 'application/json',
                    },
                    body: JSON.stringify({ name: targetName }),
                });

                if (!response.ok) {
                    const message = await response.text();
                    throw new Error(message || `Failed to delete sequence (${response.status})`);
                }

                await refreshLevelingSequenceOptions('');
                updateLevelingSequenceActionState();
            } catch (error) {
                console.error('Failed to delete leveling sequence', error);
                alert('Unable to delete the selected sequence. Please check the logs for more information.');
            }
        });
    }


    if (levelingSequenceAddBtn) {
        levelingSequenceAddBtn.addEventListener('click', () => {
            window.open('/sequence-editor', '_blank');
        });
    }

    if (levelingSequenceEditBtn) {
        levelingSequenceEditBtn.addEventListener('click', () => {
            if (!levelingSequenceSelect || !levelingSequenceSelect.value) {
                return;
            }
            const encoded = encodeURIComponent(levelingSequenceSelect.value);
            window.open(`/sequence-editor?sequence=${encoded}`, '_blank');
        });
    }

    window.addEventListener('focus', () => {
        void maybeRefreshSequencesFromStorage();
    });

    document.addEventListener('visibilitychange', () => {
        if (!document.hidden) {
            void maybeRefreshSequencesFromStorage();
        }
    });

    updateLevelingSequenceActionState();
});

function handleBossStaticThresholdChange() {
    const input = document.getElementById('novaBossStaticThreshold');
    const selectedDifficulty = document.getElementById('gameDifficulty').value;
    let minValue;
    switch (selectedDifficulty) {
        case 'normal':
            minValue = 1;
            break;
        case 'nightmare':
            minValue = 33;
            break;
        case 'hell':
            minValue = 50;
            break;
        default:
            minValue = 65;
    }

    let value = parseInt(input.value);
    if (isNaN(value) || value < minValue) {
        value = minValue;
    } else if (value > 100) {
        value = 100;
    }
    input.value = value;
}
