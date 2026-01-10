let activeRunFilter = 'all';
let currentSearchTerm = '';
let runFilterTabs = [];

window.onload = function () {
    let enabled_runs_ul = document.getElementById('enabled_runs');
    let disabled_runs_ul = document.getElementById('disabled_runs');
    let searchInput = document.getElementById('search-disabled-runs');
    runFilterTabs = document.querySelectorAll('.run-filter-tab');

    new Sortable(enabled_runs_ul, {
        group: 'runs',
        animation: 150,
        delay: 180,
        delayOnTouchOnly: true,
        touchStartThreshold: 5,
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
        delay: 180,
        delayOnTouchOnly: true,
        touchStartThreshold: 5,
        onAdd: function (evt) {
            updateButtonForDisabledRun(evt.item);
        }
    });

    searchInput.addEventListener('input', function () {
        currentSearchTerm = searchInput.value;
        filterDisabledRuns(currentSearchTerm);
    });

    // Add event listeners for add and remove buttons
    document.addEventListener('click', function (e) {
        const favButton = e.target.closest('.run-fav-btn');
        if (favButton) {
            e.preventDefault();
            e.stopPropagation();
            const runItem = favButton.closest('li');
            if (runItem) {
                toggleRunFavorite(runItem);
            }
            return;
        }
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

    initializeRunFilters();
    initializeRunFavorites();
    updateEnabledRunsHiddenField();

    const buildSelectElement = document.querySelector('select[name="characterClass"]');
    buildSelectElement.addEventListener('change', function () {
        const selectedBuild = buildSelectElement.value;
        const levelingBuilds = ['paladin', 'sorceress_leveling', 'druid_leveling', 'amazon_leveling', 'necromancer', 'assassin', 'barb_leveling'];

        const enabledRunListElement = document.getElementById('enabled_runs');
        if (!enabledRunListElement) return;

        const enabledRuns = Array.from(enabledRunListElement.querySelectorAll('li')).map(li => li.getAttribute('value'));
        const isLevelingRunEnabled = enabledRuns.includes('leveling') || enabledRuns.includes('leveling_sequence');
        const hasOtherRunsEnabled = enabledRuns.length > 1;

        if (levelingBuilds.includes(selectedBuild) && (!isLevelingRunEnabled || hasOtherRunsEnabled)) {
            alert("This profile requires enabling the leveling run. Please add only the 'leveling' run to the enabled run list and remove the others.");
        }
    });

    // --- Chest toggles ---
    // Keep "All chests" and "Super chests only" mutually exclusive.
    const interactWithChests = document.getElementById('interactWithChests');
    const interactWithSuperChests = document.getElementById('interactWithSuperChests');
    if (interactWithChests && interactWithSuperChests) {
        interactWithChests.addEventListener('change', function () {
            if (interactWithChests.checked) {
                interactWithSuperChests.checked = false;
            }
        });
        interactWithSuperChests.addEventListener('change', function () {
            if (interactWithSuperChests.checked) {
                interactWithChests.checked = false;
            }
        });
    }
}

function updateEnabledRunsHiddenField() {
    let listItems = document.querySelectorAll('#enabled_runs li');
    let values = Array.from(listItems).map(function (item) {
        return item.getAttribute("value");
    });
    document.getElementById('gameRuns').value = JSON.stringify(values);

    if (window.onGameRunsUpdated) {
        try {
            window.onGameRunsUpdated();
        } catch (e) {
            console.error('onGameRunsUpdated handler failed', e);
        }
    }
}

function getRunCategory(runName) {
    const name = runName.toLowerCase();

    if (name.includes('leveling') || name.includes('quests')) {
        return 'leveling';
    }

    if (name.includes('key') || name.includes('countess') || name.includes('summoner') || name.includes('nihl')) {
        return 'key';
    }

    if (
        name.includes('andariel') ||
        name.includes('duriel') ||
        name.includes('mephisto') ||
        name.includes('diablo') ||
        name.includes('baal')
    ) {
        return 'act-boss';
    }

    if (
        name.includes('pit') ||
        name.includes('tunnels') ||
        name.includes('ancient_tunnels') ||
        name.includes('chaos') ||
        name.includes('river') ||
        name.includes('worldstone') ||
        name.includes('wsk') ||
        name.includes('stony_tomb') ||
        name.includes('mausoleum') ||
        name.includes('arachnid_lair') ||
        name.includes('drifter_cavern')
    ) {
        return 'a85';
    }

    if (
        name.includes('pindle') ||
        name.includes('eldritch') ||
        name.includes('shenk') ||
        name.includes('thresh') ||
        name.includes('bishibosh') ||
        name.includes('rakanishu') ||
        name.includes('endugu') ||
        name.includes('fire_eye') ||
        name.includes('travincal')
    ) {
        return 'super-unique';
    }

    if (
        name.includes('cows') ||
        name.includes('lower_kurast_chest') ||
        name.includes('terror_zone') ||
        name.includes('tristram') ||
        name.includes('council') ||
        name.includes('dclone') ||
        name.includes('uber')
    ) {
        return 'special';
    }

    if (
        name.includes('spider_cavern') ||
        name.includes('tal_rasha_tombs') ||
        name.includes('lower_kurast')
    ) {
        return 'all-only';
    }

    return 'other';
}

function assignRunCategories() {
    const allRuns = document.querySelectorAll('#enabled_runs li, #disabled_runs li');
    allRuns.forEach((item) => {
        const runName = (item.getAttribute('value') || '').toLowerCase();
        if (!runName) {
            return;
        }
        item.dataset.runName = runName;
        item.dataset.runCategory = getRunCategory(runName);
    });
}

function applyRunFilter() {
    const element = document.getElementById('disabled_runs');
    if (!element) {
        return;
    }
    const listItems = element.querySelectorAll('li');
    listItems.forEach((item) => {
        const runName = item.dataset.runName || (item.getAttribute('value') || '').toLowerCase();
        const category = item.dataset.runCategory || getRunCategory(runName);
        const isFavorite = item.dataset.favorite === '1';
        const matchesCategory =
            activeRunFilter === 'all' ||
            (activeRunFilter === 'favorite' && isFavorite) ||
            (category !== 'all-only' && category === activeRunFilter);
        const matchesSearch = !currentSearchTerm || runName.includes(currentSearchTerm.toLowerCase());
        item.style.display = matchesCategory && matchesSearch ? '' : 'none';
    });
}

function updateRunFavoriteUI(runItem, isFavorite) {
    const favButton = runItem.querySelector('.run-fav-btn');
    if (favButton) {
        favButton.classList.toggle('active', isFavorite);
        const icon = favButton.querySelector('i');
        if (icon) {
            icon.classList.toggle('bi-star-fill', isFavorite);
            icon.classList.toggle('bi-star', !isFavorite);
        }
    }
    runItem.dataset.favorite = isFavorite ? '1' : '0';

    const existingInput = runItem.querySelector('.run-fav-input');
    if (isFavorite && !existingInput) {
        const input = document.createElement('input');
        input.type = 'hidden';
        input.className = 'run-fav-input';
        input.name = 'runFavoriteRuns';
        input.value = runItem.getAttribute('value') || '';
        runItem.prepend(input);
    } else if (!isFavorite && existingInput) {
        existingInput.remove();
    }
    applyRunFilter();
}

function toggleRunFavorite(runItem) {
    const isFavorite = runItem.dataset.favorite === '1';
    updateRunFavoriteUI(runItem, !isFavorite);
}

function initializeRunFavorites() {
    const runItems = document.querySelectorAll('#enabled_runs li, #disabled_runs li');
    runItems.forEach((item) => {
        const isFavorite = item.dataset.favorite === '1';
        updateRunFavoriteUI(item, isFavorite);
    });
}

function initializeRunFilters() {
    assignRunCategories();
    if (!runFilterTabs || !runFilterTabs.length) {
        return;
    }
    runFilterTabs.forEach((tab) => {
        tab.addEventListener('click', () => {
            activeRunFilter = tab.dataset.runFilter || 'all';
            runFilterTabs.forEach((btn) => btn.classList.remove('active'));
            tab.classList.add('active');
            applyRunFilter();
        });
    });
    applyRunFilter();
}

function filterDisabledRuns(searchTerm) {
    currentSearchTerm = searchTerm || '';
    applyRunFilter();
}

function checkLevelingProfile() {
    const levelingProfiles = [
        "sorceress_leveling",
        "paladin",
        "druid_leveling",
        "amazon_leveling",
        "necromancer",
        "assassin",
        "barb_leveling"
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
    applyRunFilter();
}

function moveRunToEnabled(runElement) {
    const enabledRunsUl = document.getElementById('enabled_runs');
    updateButtonForEnabledRun(runElement);
    enabledRunsUl.appendChild(runElement);
    updateEnabledRunsHiddenField();
    applyRunFilter();
}

function updateButtonForEnabledRun(runElement) {
    const button = runElement.querySelector('button.add-run, button.remove-run');
    if (!button) {
        return;
    }
    button.classList.remove('add-run');
    button.classList.add('remove-run');
    button.title = "Remove run";
    button.innerHTML = '<i class="bi bi-dash"></i>';
}

function updateButtonForDisabledRun(runElement) {
    const button = runElement.querySelector('button.add-run, button.remove-run');
    if (!button) {
        return;
    }
    button.classList.remove('remove-run');
    button.classList.add('add-run');
    button.title = "Add run";
    button.innerHTML = '<i class="bi bi-plus"></i>';
}

document.addEventListener('DOMContentLoaded', function () {
    const schedulerEnabled = document.querySelector('input[name="schedulerEnabled"]');
    const schedulerSettings = document.getElementById('scheduler-settings');
    const cloneSelect = document.getElementById('cloneSupervisorSelect');

    if (cloneSelect) {
        cloneSelect.addEventListener('change', function () {
            const url = new URL(window.location.href);
            url.searchParams.delete('supervisor');
            url.searchParams.delete('clone');
            const newValue = cloneSelect.value;
            if (newValue) {
                url.searchParams.set('clone', newValue);
            }
            const search = url.searchParams.toString();
            const target = url.pathname + (search ? `?${search}` : '');
            window.location.href = target;
        });
    }
    const characterClassSelect = document.querySelector('select[name="characterClass"]');
    const mainCharacterClassSelect = document.getElementById('mainCharacterClass');
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

    const classBuildMapping = {
        amazon: [
            { value: 'javazon', label: 'Javazon' },
            { value: 'amazon_leveling', label: 'Amazon (Leveling)' },
        ],
        assassin: [
            { value: 'assassin', label: 'Assassin (Leveling)' },
            { value: 'trapsin', label: 'Lightning Trapsin' },
            { value: 'mosaic', label: 'Mosaic Assassin' },
        ],
        barbarian: [
            { value: 'barb_leveling', label: 'Barbarian (Leveling)' },
            { value: 'berserker', label: 'Berserk Barbarian' },
            { value: 'warcry_barb', label: 'Warcry Barbarian' },
        ],
        druid: [
            { value: 'druid_leveling', label: 'Druid (Leveling)' },
            { value: 'winddruid', label: 'Tornado Druid' },
        ],
        necromancer: [
            { value: 'necromancer', label: 'Necromancer (Leveling)' },
        ],
        paladin: [
            { value: 'paladin', label: 'Paladin (Leveling)' },
            { value: 'hammerdin', label: 'Hammer Paladin' },
            { value: 'foh', label: 'FOH Paladin' },
            { value: 'dragondin', label: 'Dragondin' },
            { value: 'smiter', label: 'Smiter (Ubers)' },
        ],
        sorceress: [
            { value: 'sorceress', label: 'Blizzard Sorceress' },
            { value: 'nova', label: 'Nova Sorceress' },
            { value: 'hydraorb', label: 'Hydra Orb Sorceress' },
            { value: 'lightsorc', label: 'Lightning Sorceress' },
            { value: 'fireballsorc', label: 'Fireball Sorceress' },
            { value: 'sorceress_leveling', label: 'Sorceress (Leveling)' },
        ],
        other: [
            { value: 'mule', label: 'Mule' },
            { value: 'development', label: 'Development' },
        ],
    };

    function findMainClassForBuild(buildValue) {
        if (!buildValue) return '';
        for (const [mainClass, builds] of Object.entries(classBuildMapping)) {
            if (builds.some(b => b.value === buildValue)) {
                return mainClass;
            }
        }
        return '';
    }

    function populateBuildSelect(mainClass, currentBuild) {
        if (!characterClassSelect) return;
        const builds = classBuildMapping[mainClass] || [];

        characterClassSelect.innerHTML = '';

        const placeholder = document.createElement('option');
        placeholder.value = '';
        placeholder.textContent = builds.length ? '-- Select build --' : '-- No build available --';
        if (!currentBuild) {
            placeholder.selected = true;
        }
        characterClassSelect.appendChild(placeholder);

        if (!builds.length) {
            return;
        }

        builds.forEach(build => {
            const opt = document.createElement('option');
            opt.value = build.value;
            opt.textContent = build.label;
            if (build.value === currentBuild) {
                opt.selected = true;
            }
            characterClassSelect.appendChild(opt);
        });
    }

    function initializeClassSelectors() {
        if (!characterClassSelect || !mainCharacterClassSelect) return;

        const initialBuildValue = characterClassSelect.dataset.currentBuild || '';
        const detectedMainClass = findMainClassForBuild(initialBuildValue) || 'sorceress';

        mainCharacterClassSelect.value = detectedMainClass;
        populateBuildSelect(detectedMainClass, initialBuildValue || undefined);
    }

    if (bossStaticThresholdInput) {
        bossStaticThresholdInput.addEventListener('input', handleBossStaticThresholdChange);
    }

    function toggleSchedulerVisibility() {
        schedulerSettings.style.display = schedulerEnabled.checked ? 'block' : 'none';
    }

    function toggleSchedulerMode() {
        const mode = document.getElementById('schedulerMode').value;
        const timeSlotsMode = document.getElementById('timeSlotsMode');
        const durationMode = document.getElementById('durationMode');

        if (mode === 'duration') {
            if (timeSlotsMode) timeSlotsMode.style.display = 'none';
            if (durationMode) durationMode.style.display = 'block';
        } else {
            if (timeSlotsMode) timeSlotsMode.style.display = 'block';
            if (durationMode) durationMode.style.display = 'none';
        }
    }

    // Load scheduler history from API
    function loadSchedulerHistory() {
        const historyPanel = document.getElementById('schedulerHistoryPanel');
        const historyContent = document.getElementById('schedulerHistoryContent');
        if (!historyPanel || !historyContent) return;

        // Get supervisor name from URL
        const urlParams = new URLSearchParams(window.location.search);
        const supervisor = urlParams.get('supervisor');
        if (!supervisor) {
            historyContent.innerHTML = '<p class="history-empty">No supervisor selected</p>';
            return;
        }

        fetch(`/api/scheduler-history?supervisor=${encodeURIComponent(supervisor)}`)
            .then(response => response.json())
            .then(data => {
                if (!data.history || data.history.length === 0) {
                    historyContent.innerHTML = '<p class="history-empty">No play history yet. History is recorded when using Duration mode.</p>';
                    return;
                }

                // Calculate stats
                let totalPlayMinutes = 0;
                let totalWakeMinutes = 0;
                let totalSleepMinutes = 0;
                let count = 0;

                data.history.forEach(entry => {
                    totalPlayMinutes += entry.totalPlayMinutes || 0;
                    if (entry.wakeTime) {
                        const [h, m] = entry.wakeTime.split(':').map(Number);
                        totalWakeMinutes += h * 60 + m;
                    }
                    if (entry.sleepTime) {
                        const [h, m] = entry.sleepTime.split(':').map(Number);
                        totalSleepMinutes += h * 60 + m;
                    }
                    count++;
                });

                const avgPlayHours = count > 0 ? (totalPlayMinutes / count / 60).toFixed(1) : 0;
                const avgWakeMinutes = count > 0 ? Math.round(totalWakeMinutes / count) : 0;
                const avgSleepMinutes = count > 0 ? Math.round(totalSleepMinutes / count) : 0;
                const avgWakeTime = `${String(Math.floor(avgWakeMinutes / 60)).padStart(2, '0')}:${String(avgWakeMinutes % 60).padStart(2, '0')}`;
                const avgSleepTime = `${String(Math.floor(avgSleepMinutes / 60)).padStart(2, '0')}:${String(avgSleepMinutes % 60).padStart(2, '0')}`;
                const totalHours = (totalPlayMinutes / 60).toFixed(1);

                // Build HTML
                let html = `
                    <div class="history-stats">
                        <div class="stat-item"><strong>Avg Play:</strong> ${avgPlayHours}h/day</div>
                        <div class="stat-item"><strong>Avg Wake:</strong> ${avgWakeTime}</div>
                        <div class="stat-item"><strong>Avg Sleep:</strong> ${avgSleepTime}</div>
                        <div class="stat-item"><strong>Total:</strong> ${totalHours}h over ${count} days</div>
                    </div>
                    <table class="history-table">
                        <thead>
                            <tr>
                                <th>Date</th>
                                <th>Play</th>
                                <th>Wake</th>
                                <th>Sleep</th>
                                <th>Breaks</th>
                            </tr>
                        </thead>
                        <tbody>
                `;

                data.history.slice(0, 10).forEach(entry => {
                    const playHours = ((entry.totalPlayMinutes || 0) / 60).toFixed(1);
                    const breakCount = entry.breaks ? entry.breaks.length : 0;
                    html += `
                        <tr>
                            <td>${entry.date}</td>
                            <td>${playHours}h</td>
                            <td>${entry.wakeTime || '-'}</td>
                            <td>${entry.sleepTime || '-'}</td>
                            <td>${breakCount}</td>
                        </tr>
                    `;
                });

                html += '</tbody></table>';

                if (data.history.length > 10) {
                    html += `<p class="history-more">Showing 10 of ${data.history.length} days</p>`;
                }

                historyContent.innerHTML = html;
            })
            .catch(error => {
                console.error('Failed to load scheduler history:', error);
                historyContent.innerHTML = '<p class="history-error">Failed to load history</p>';
            });
    }

    function updateCharacterOptions() {
        const selectedClass = characterClassSelect.value;
        const noSettingsMessage = document.getElementById('no-settings-message');
        const berserkerBarbOptions = document.querySelector('.berserker-barb-options');
        const warcryBarbOptions = document.querySelector('.warcry-barb-options');
        const barbLevelingOptions = document.querySelector('.barb-leveling-options');
        const novaSorceressOptions = document.querySelector('.nova-sorceress-options');
        const mosaicAssassinOptions = document.querySelector('.mosaic-assassin-options');
        const blizzardSorceressOptions = document.querySelector('.blizzard-sorceress-options');
        const sorceressLevelingOptions = document.querySelector('.sorceress_leveling-options');
        const lightningSorceressOptions = document.querySelector('.lightsorc-options');
        const hydraOrbSorceressOptions = document.querySelector('.hydraorb-options');
        const fireballSorceressOptions = document.querySelector('.fireballsorc-options');
        const assassinLevelingOptions = document.querySelector('.assassin-options');
        const amazonLevelingOptions = document.querySelector('.amazon_leveling-options');
        const druidLevelingOptions = document.querySelector('.druid_leveling-options');
        const necromancerLevelingOptions = document.querySelector('.necromancer-options');
        const paladinLevelingOptions = document.querySelector('.paladin-options');
        const smiterOptions = document.querySelector('.smiter-options');
        const javazonOptions = document.querySelector('.javazon-options');

        // Hide all options first
        if (berserkerBarbOptions) berserkerBarbOptions.style.display = 'none';
        if (warcryBarbOptions) warcryBarbOptions.style.display = 'none';
        if (barbLevelingOptions) barbLevelingOptions.style.display = 'none';

        // Hide all options first
        if (berserkerBarbOptions) berserkerBarbOptions.style.display = 'none';
        if (novaSorceressOptions) novaSorceressOptions.style.display = 'none';
        if (mosaicAssassinOptions) mosaicAssassinOptions.style.display = 'none';
        if (blizzardSorceressOptions) blizzardSorceressOptions.style.display = 'none';
        if (sorceressLevelingOptions) sorceressLevelingOptions.style.display = 'none';
        if (lightningSorceressOptions) lightningSorceressOptions.style.display = 'none';
        if (hydraOrbSorceressOptions) hydraOrbSorceressOptions.style.display = 'none';
        if (fireballSorceressOptions) fireballSorceressOptions.style.display = 'none';
        if (assassinLevelingOptions) assassinLevelingOptions.style.display = 'none';
        if (amazonLevelingOptions) amazonLevelingOptions.style.display = 'none';
        if (druidLevelingOptions) druidLevelingOptions.style.display = 'none';
        if (necromancerLevelingOptions) necromancerLevelingOptions.style.display = 'none';
        if (paladinLevelingOptions) paladinLevelingOptions.style.display = 'none';
        if (smiterOptions) smiterOptions.style.display = 'none';
        if (javazonOptions) javazonOptions.style.display = 'none';
        if (noSettingsMessage) noSettingsMessage.style.display = 'none';

        // Show relevant options based on class
        if (selectedClass === 'berserker') {
            berserkerBarbOptions.style.display = 'block';
        } else if (selectedClass === 'warcry_barb') {
            warcryBarbOptions.style.display = 'block';
        } else if (selectedClass === 'barb_leveling') {
            barbLevelingOptions.style.display = 'block';
        } else if (selectedClass === 'nova' || selectedClass === 'lightsorc') {
            novaSorceressOptions.style.display = 'block';
            updateNovaSorceressOptions();
        } else if (selectedClass === 'lightsorc') {
            if (lightningSorceressOptions) lightningSorceressOptions.style.display = 'block';
        } else if (selectedClass === 'hydraorb') {
            if (hydraOrbSorceressOptions) hydraOrbSorceressOptions.style.display = 'block';
        } else if (selectedClass === 'fireballsorc') {
            if (fireballSorceressOptions) fireballSorceressOptions.style.display = 'block';
        } else if (selectedClass === 'mosaic') {
            if (mosaicAssassinOptions) mosaicAssassinOptions.style.display = 'block';
        } else if (selectedClass === 'sorceress') {
            if (blizzardSorceressOptions) blizzardSorceressOptions.style.display = 'block';
        } else if (selectedClass === 'sorceress_leveling') {
            if (sorceressLevelingOptions) sorceressLevelingOptions.style.display = 'block';
        } else if (selectedClass === 'assassin') {
            if (assassinLevelingOptions) assassinLevelingOptions.style.display = 'block';
        } else if (selectedClass === 'amazon_leveling') {
            if (amazonLevelingOptions) amazonLevelingOptions.style.display = 'block';
        } else if (selectedClass === 'druid_leveling') {
            if (druidLevelingOptions) druidLevelingOptions.style.display = 'block';
        } else if (selectedClass === 'necromancer') {
            if (necromancerLevelingOptions) necromancerLevelingOptions.style.display = 'block';
        } else if (selectedClass === 'paladin') {
            if (paladinLevelingOptions) paladinLevelingOptions.style.display = 'block';
        } else if (selectedClass === 'smiter') {
            if (smiterOptions) smiterOptions.style.display = 'block';
        } else if (selectedClass === 'javazon') {
            if (javazonOptions) javazonOptions.style.display = 'block';
        } else {
            if (noSettingsMessage) noSettingsMessage.style.display = 'block';
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
            useExtraBuffsDistContainer.classList.toggle('is-hidden', !useExtraBuffsCheckbox.checked);
        }
    }

    // Javazon: force quantity refill hint
    const javazonForceRefillInput = document.getElementById('javazonDensityKillerForceRefillBelowPercent');
    const javazonForceRefillHint = document.getElementById('javazonForceRefillHint');

    function updateJavazonForceRefillHint() {
        if (!javazonForceRefillInput || !javazonForceRefillHint) return;
        let v = parseInt(javazonForceRefillInput.value, 10);
        if (isNaN(v)) v = 50;
        if (v < 1) v = 1;
        if (v > 100) v = 100;
        javazonForceRefillInput.value = v;
        javazonForceRefillHint.textContent = `Quantity refill < ${v}%`;
    }

    if (javazonForceRefillInput) {
        javazonForceRefillInput.addEventListener('input', updateJavazonForceRefillHint);
        javazonForceRefillInput.addEventListener('change', updateJavazonForceRefillHint);
        updateJavazonForceRefillHint();
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

    if (mainCharacterClassSelect && characterClassSelect) {
        initializeClassSelectors();

        mainCharacterClassSelect.addEventListener('change', function () {
            const mainClass = mainCharacterClassSelect.value;
            populateBuildSelect(mainClass, '');
            updateCharacterOptions();
        });
    }

    if (characterClassSelect) {
        characterClassSelect.addEventListener('change', updateCharacterOptions);
    }
    document.getElementById('gameDifficulty').addEventListener('change', function () {
        if (characterClassSelect.value === 'nova' || characterClassSelect.value === 'lightsorc') {
            updateNovaSorceressOptions();
        }
    });

    characterClassSelect.addEventListener('change', updateCharacterOptions);
    updateCharacterOptions(); // Call this initially to set the correct state

    // Set initial state
    toggleSchedulerVisibility();
    toggleSchedulerMode();
    loadSchedulerHistory();
    updateNovaSorceressOptions();

    schedulerEnabled.addEventListener('change', toggleSchedulerVisibility);

    // Mode toggle event listener
    const schedulerModeSelect = document.getElementById('schedulerMode');
    if (schedulerModeSelect) {
        schedulerModeSelect.addEventListener('change', toggleSchedulerMode);
    }

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
                    <span>Var:</span>
                    <input type="number" name="scheduler[${day}][startVar][]" min="0" max="60" step="5" placeholder="0" title="Start variance (+/- min)" style="width:60px;">
                    <span>/</span>
                    <input type="number" name="scheduler[${day}][endVar][]" min="0" max="60" step="5" placeholder="0" title="End variance (+/- min)" style="width:60px;">
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

    document.getElementById('tzTrackAll').addEventListener('change', function (e) {
        document.querySelectorAll('.tz-child-checkbox').forEach(checkbox => {
            checkbox.checked = e.target.checked;
        });
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

    const navLinks = Array.from(document.querySelectorAll('.settings-nav a'));
    const navContainer = document.querySelector('.settings-nav');
    const hoverToggle = document.getElementById('navHoverToggle');
    const hoverToggleIcon = document.getElementById('navHoverToggleIcon');
    const hoverToggleText = document.getElementById('navHoverToggleText');
    const HOVER_EXPAND_KEY = 'koolo:navHoverExpand';

    if (navContainer && hoverToggle) {
        const updateHoverToggleUI = (enabled) => {
            if (hoverToggleIcon) {
                hoverToggleIcon.classList.toggle('bi-arrows-angle-expand', !enabled);
                hoverToggleIcon.classList.toggle('bi-arrows-angle-contract', enabled);
            }
            if (hoverToggleText) {
                hoverToggleText.textContent = enabled ? 'Hover expand on' : 'Hover expand off';
            }
        };

        let hoverEnabled = true;
        try {
            hoverEnabled = window.localStorage.getItem(HOVER_EXPAND_KEY) !== '0';
        } catch (error) {
            hoverEnabled = true;
        }
        navContainer.classList.toggle('hover-expand', hoverEnabled);
        hoverToggle.checked = hoverEnabled;
        updateHoverToggleUI(hoverEnabled);
        hoverToggle.addEventListener('change', () => {
            const enabled = hoverToggle.checked;
            navContainer.classList.toggle('hover-expand', enabled);
            updateHoverToggleUI(enabled);
            try {
                window.localStorage.setItem(HOVER_EXPAND_KEY, enabled ? '1' : '0');
            } catch (error) {
                // Ignore storage errors; toggle still works for the session.
            }
        });
    }
    const sectionLinks = new Map();

    navLinks.forEach((link) => {
        const href = link.getAttribute('href') || '';
        if (!href.startsWith('#')) {
            return;
        }
        const targetId = href.slice(1);
        const section = document.getElementById(targetId);
        if (!section) {
            return;
        }
        sectionLinks.set(section, link);
        link.addEventListener('click', (event) => {
            event.preventDefault();
            section.scrollIntoView({ behavior: 'smooth', block: 'start' });
            history.replaceState(null, '', href);
        });
    });

    const setActiveLink = (active) => {
        navLinks.forEach((link) => {
            link.classList.toggle('active', link === active);
        });
    };

    if (sectionLinks.size) {
        const observer = new IntersectionObserver(
            (entries) => {
                entries.forEach((entry) => {
                    if (!entry.isIntersecting) {
                        return;
                    }
                    const activeLink = sectionLinks.get(entry.target);
                    if (activeLink) {
                        setActiveLink(activeLink);
                    }
                });
            },
            {
                rootMargin: '-20% 0px -70% 0px',
                threshold: 0,
            }
        );

        sectionLinks.forEach((_, section) => observer.observe(section));

        const hashLink = window.location.hash
            ? navLinks.find((link) => link.getAttribute('href') === window.location.hash)
            : null;
        setActiveLink(hashLink || navLinks[0]);
    }
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
