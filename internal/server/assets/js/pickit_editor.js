// Pickit Editor JavaScript
let allItems = [];
let selectedItem = null;
let availableStats = [];
let currentCategory = 'All';
let selectedRowIndex = null;
let currentPickitPath = '';
let availableFiles = [];

// Check if file config exists on page load
window.addEventListener('DOMContentLoaded', function () {
    const savedPath = localStorage.getItem('pickitPath');

    if (savedPath) {
        currentPickitPath = savedPath;
        loadCharacterFiles();
    }

    // Always load data
    loadData();
});

// File Configuration Modal Functions
function showFileConfigModal() {
    const modal = document.getElementById('fileConfigModal');
    modal.classList.add('active');

    // Pre-fill if saved
    const savedPath = localStorage.getItem('pickitPath');
    if (savedPath) {
        document.getElementById('pickitPath').value = savedPath;
    }
}

async function browseFolder() {
    try {
        const response = await fetch('/api/pickit/browse-folder', {
            method: 'POST'
        });

        if (!response.ok) {
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }

        const data = await response.json();

        if (data.error) {
            alert('Error: ' + data.error);
            return;
        }

        if (data.cancelled) {
            // User cancelled the dialog, do nothing
            return;
        }

        if (data.path && data.path !== '') {
            document.getElementById('pickitPath').value = data.path;
            // Automatically scan after selecting folder
            await scanForFiles();
        } else {
            alert('No folder was selected. Please try again or enter the path manually.\n\nExample: G:\\koolo\\config\\MyChar\\pickit');
        }
    } catch (error) {
        console.error('Error browsing folder:', error);
        alert('Failed to open folder browser: ' + error.message + '\n\nPlease enter the full path to your pickit folder manually.\n\nExample: G:\\koolo\\config\\MyChar\\pickit');
    }
} function closeFileConfigModal() {
    const modal = document.getElementById('fileConfigModal');
    modal.classList.remove('active');
}

async function scanForFiles() {
    const pickitPath = document.getElementById('pickitPath').value.trim();
    if (!pickitPath) {
        alert('Please enter or browse to a pickit folder path');
        return;
    }

    try {
        const response = await fetch(`/api/pickit/files?path=${encodeURIComponent(pickitPath)}`);
        if (!response.ok) {
            throw new Error('Failed to load files');
        }

        const files = await response.json();
        availableFiles = files;

        const filesList = document.getElementById('foundFiles');
        if (files && files.length > 0) {
            filesList.innerHTML = '<p style="color: #4CAF50;">Found ' + files.length + ' .nip files:</p>' +
                files.map(f => `<div class="file-item">${f.name || f}</div>`).join('');
        } else {
            filesList.innerHTML = '<p style="color: #ff9800;">No .nip files found in this directory.</p>';
        }
    } catch (error) {
        document.getElementById('foundFiles').innerHTML = '<p style="color: #f44336;">Error: ' + error.message + '</p>';
    }
}

function saveFileConfig() {
    const pickitPath = document.getElementById('pickitPath').value.trim();
    if (!pickitPath) {
        alert('Please enter or browse to a pickit folder path');
        return;
    }

    currentPickitPath = pickitPath;
    localStorage.setItem('pickitPath', pickitPath);

    closeFileConfigModal();
    loadData();
    loadCharacterFiles();
}

async function loadCharacterFiles() {
    if (!currentPickitPath) return;

    try {
        const response = await fetch(`/api/pickit/files?path=${encodeURIComponent(currentPickitPath)}`);
        if (response.ok) {
            const files = await response.json();
            const select = document.getElementById('pickitFile');
            select.innerHTML = files.map(f => {
                const fileName = f.name || f;
                return `<option value="${fileName}">${fileName}</option>`;
            }).join('');

            // Add option to create new file
            if (files.length === 0) {
                select.innerHTML = '<option value="">No files found - create new</option>';
            }
        }
    } catch (error) {
        console.error('Failed to load character files:', error);
    }
}

// Load initial data
async function loadData() {
    try {
        // Load items from V2 database
        const itemsResponse = await fetch('/api/pickit/items');
        allItems = await itemsResponse.json();

        console.log(`Loaded ${allItems.length} items from d2go database`);

        // Load stats
        const statsResponse = await fetch('/api/pickit/stats');
        const statsData = await statsResponse.json();
        availableStats = statsData.all;

        // Templates feature removed
        // const templatesResponse = await fetch('/api/pickit/templates');
        // const templates = await templatesResponse.json();
        // renderTemplates(templates);

        renderItems(allItems);
    } catch (error) {
        console.error('Failed to load data:', error);
    }
}

function renderItems(items) {
    const tbody = document.getElementById('itemTableBody');
    if (!tbody) return;

    tbody.innerHTML = items.map((item, index) => {
        const categoryClass = `category-${item.category.toLowerCase()}`;
        return `
            <tr onclick="selectItemRow(${index}, '${item.name.replace(/'/g, "\\'")}')">
                <td>${item.name}</td>
                <td>${item.type || '-'}</td>
                <td><span class="category-badge ${categoryClass}">${item.category}</span></td>
                <td>${item.itemLevel || '-'}</td>
            </tr>
        `;
    }).join('');
}

function showCategory(category) {
    currentCategory = category;

    // Update tab highlighting
    document.querySelectorAll('.tab-button').forEach(btn => {
        btn.classList.remove('active');
    });
    event.target.classList.add('active');

    // Filter items
    filterItems();
}

function filterItems() {
    const searchTerm = document.getElementById('searchInput').value.toLowerCase();

    let filtered = allItems;

    // Filter by category
    if (currentCategory !== 'All') {
        filtered = filtered.filter(item => item.category === currentCategory);
    }

    // Filter by search term
    if (searchTerm) {
        filtered = filtered.filter(item =>
            item.name.toLowerCase().includes(searchTerm) ||
            item.nipName.toLowerCase().includes(searchTerm) ||
            (item.type && item.type.toLowerCase().includes(searchTerm))
        );
    }

    renderItems(filtered);
}

function selectItemRow(index, itemName) {
    // Remove previous selection
    document.querySelectorAll('.item-table tr').forEach(tr => tr.classList.remove('selected'));

    // Highlight selected row
    event.currentTarget.classList.add('selected');
    selectedRowIndex = index;

    // Select item for editing
    selectItem(itemName);
}

// Templates feature removed
/*
function renderTemplates(templates) {
    const grid = document.getElementById('templateGrid');
    grid.innerHTML = templates.map((template, index) => `
        <div class="template-card" onclick="applyTemplate(${index})">
            <div class="template-name">${template.name}</div>
            <div class="template-description">${template.description || ''}</div>
        </div>
    `).join('');

    // Store templates globally
    window.pickitTemplates = templates;
}
*/

function selectItem(itemName) {
    selectedItem = allItems.find(i => i.name === itemName);
    if (!selectedItem) return;

    // Reset form
    document.getElementById('itemName').value = selectedItem.name;
    document.getElementById('maxQuantity').value = '0';
    document.getElementById('comments').value = '';
    document.getElementById('enableStats').checked = false;
    document.getElementById('statsContainer').style.display = 'none';
    document.getElementById('statsRows').innerHTML = '';

    // Check if item is a rune (runes don't need quality or stats)
    const isRune = selectedItem.category === 'Runes';

    // Handle quality selector
    const qualityContainer = document.getElementById('qualityContainer');
    const qualityRadios = document.querySelectorAll('input[name="quality"]');

    if (isRune) {
        qualityContainer.style.display = 'none';
        qualityRadios.forEach(radio => radio.checked = false);
    } else {
        qualityContainer.style.display = 'block';

        // Pre-select quality if item only has one
        if (selectedItem.quality && selectedItem.quality.length === 1) {
            const qualityValue = selectedItem.quality[0].toLowerCase();
            qualityRadios.forEach(radio => {
                radio.checked = radio.value === qualityValue;
            });
        } else {
            qualityRadios.forEach(radio => radio.checked = false);
        }
    }

    // Handle stats selector - hide for runes
    const enableStatsContainer = document.querySelector('.form-group:has(#enableStats)');
    if (enableStatsContainer) {
        enableStatsContainer.style.display = isRune ? 'none' : 'block';
    }

    updateNIPPreview();
}

// Templates feature removed
/*
function applyTemplate(templateIndex) {
    const template = window.pickitTemplates[templateIndex];
    if (!template || !template.rule) {
        console.error('Invalid template or missing rule:', template);
        return;
    }

    const rule = template.rule;

    // Extract item type/name from left conditions
    let itemType = '';
    let itemQuality = '';

    if (rule.leftConditions) {
        for (const cond of rule.leftConditions) {
            if (cond.property === 'type') {
                itemType = cond.value;
            }
            if (cond.property === 'quality') {
                itemQuality = cond.value;
            }
        }
    }

    // Find the item by type or name
    const item = allItems.find(i =>
        i.type.toLowerCase() === itemType.toLowerCase() ||
        i.name.toLowerCase().includes(itemType.toLowerCase())
    );

    if (!item) {
        showValidation('error', `Item type "${itemType}" not found for template`);
        return;
    }

    // Select the item (this will populate the form)
    selectItem(item.name);

    // Apply quality if specified
    if (itemQuality) {
        const qualityRadio = document.querySelector(`input[name="quality"][value="${itemQuality.toLowerCase()}"]`);
        if (qualityRadio) {
            qualityRadio.checked = true;
        }
    }

    // Apply max quantity if specified
    if (rule.maxQuantity !== undefined && rule.maxQuantity > 0) {
        document.getElementById('maxQuantity').value = rule.maxQuantity;
    }

    // Apply stats from right conditions
    if (rule.rightConditions && rule.rightConditions.length > 0) {
        document.getElementById('enableStats').checked = true;
        document.getElementById('statsContainer').style.display = 'block';

        // Clear existing stats
        document.getElementById('statsRows').innerHTML = '';

        // Add template stats
        rule.rightConditions.forEach(cond => {
            addStatRow();
            const rows = document.querySelectorAll('.stat-row');
            const lastRow = rows[rows.length - 1];

            lastRow.querySelector('.stat-select').value = cond.property;
            lastRow.querySelector('.stat-operator').value = cond.operator;
            lastRow.querySelector('.stat-value').value = cond.value;
        });
    }

    // Apply comments if specified
    if (rule.comments) {
        document.getElementById('comments').value = rule.comments;
    }

    updateNIPPreview();
    showValidation('success', `Applied template: ${template.name}`);
}
    */

function showTab(tabName) {
    // Hide all tabs
    document.querySelectorAll('.tab-content').forEach(tab => {
        tab.classList.remove('active');
    });

    // Show selected tab
    document.getElementById(tabName + 'Tab').classList.add('active');

    // Update tab buttons
    document.querySelectorAll('.tab-button').forEach(btn => {
        btn.classList.remove('active');
    });
    document.querySelector(`[onclick="showTab('${tabName}')"]`).classList.add('active');
}

function toggleStats() {
    const enabled = document.getElementById('enableStats').checked;
    document.getElementById('statsContainer').style.display = enabled ? 'block' : 'none';
    updateNIPPreview();
}

function addStat() {
    addStatRow();
}

function addStatRow() {
    const container = document.getElementById('statsRows');
    const row = document.createElement('div');
    row.className = 'stat-row';

    row.innerHTML = `
        <select class="stat-select" onchange="updateNIPPreview()">
            <option value="">Select stat...</option>
            ${availableStats.map(stat => `<option value="${stat.id}">${stat.name}</option>`).join('')}
        </select>
        <select class="stat-operator" onchange="updateNIPPreview()">
            <option value=">=">>=</option>
            <option value=">">></option>
            <option value="==">=</option>
            <option value="<"><</option>
            <option value="<="><=</option>
            <option value="!=">!=</option>
        </select>
        <input type="number" class="stat-value" placeholder="Value" oninput="updateNIPPreview()">
        <button type="button" class="btn btn-danger" onclick="removeStat(this)" style="padding: 8px 12px;">✕</button>
    `;

    container.appendChild(row);
}

function removeStat(button) {
    button.parentElement.remove();
    updateNIPPreview();
}

function updateNIPPreview() {
    // Get item name from input field (either from selectedItem or manually typed)
    const itemNameInput = document.getElementById('itemName').value.trim();

    if (!itemNameInput) {
        document.getElementById('nipPreview').textContent = '// Enter an item name to see NIP syntax';
        return;
    }

    // Use nipName from selectedItem if available, otherwise convert manual input
    let nipName;
    if (selectedItem && selectedItem.name === itemNameInput) {
        // For unique items with a base item, use the base item name instead
        if (selectedItem.baseItem && selectedItem.category === 'Uniques') {
            nipName = selectedItem.baseItem;
        } else {
            nipName = selectedItem.nipName || itemNameInput.toLowerCase().replace(/\s+/g, '').replace(/'/g, '').replace(/-/g, '');
        }
    } else {
        // Manual input - convert to NIP format
        nipName = itemNameInput.toLowerCase().replace(/\s+/g, '').replace(/'/g, '').replace(/-/g, '');
    }

    let nip = `[name] == ${nipName}`;

    // Add quality (skip for runes)
    const isRune = selectedItem && selectedItem.category === 'Runes';
    const selectedQuality = document.querySelector('input[name="quality"]:checked');
    if (selectedQuality && selectedQuality.value && !isRune) {
        nip += ` && [quality] == ${selectedQuality.value}`;
    }

    // Add sockets
    const socketOperator = document.getElementById('socketOperator').value;
    const socketValue = document.getElementById('socketValue').value;
    if (socketOperator && socketValue) {
        nip += ` && [sockets] ${socketOperator} ${socketValue}`;
    }

    // Add ethereal
    const selectedEthereal = document.querySelector('input[name="ethereal"]:checked');
    if (selectedEthereal && selectedEthereal.value) {
        nip += ` && [flag] ${selectedEthereal.value}`;
    }

    // Add stats if enabled
    if (document.getElementById('enableStats').checked) {
        const statRows = document.querySelectorAll('.stat-row');
        const stats = [];

        statRows.forEach(row => {
            const stat = row.querySelector('.stat-select').value;
            const operator = row.querySelector('.stat-operator').value;
            const value = row.querySelector('.stat-value').value;

            if (stat && value) {
                stats.push(`[${stat}] ${operator} ${value}`);
            }
        });

        if (stats.length > 0) {
            nip += ' # ' + stats.join(' && ');
        }
    } else {
        nip += ' #';
    }

    // Add max quantity
    const maxQty = document.getElementById('maxQuantity').value;
    if (maxQty && maxQty !== '0') {
        nip += ` # [maxquantity] == ${maxQty}`;
    }

    // Add comments if any
    const comments = document.getElementById('comments').value;
    if (comments) {
        nip += ` // ${comments}`;
    }

    document.getElementById('nipPreview').textContent = nip;
}

async function validateRule() {
    const nipPreview = document.getElementById('nipPreview').textContent;

    if (!nipPreview || nipPreview.startsWith('//')) {
        showValidation('error', 'Please create a rule first');
        return;
    }

    try {
        // Use the simple NIP line validation endpoint
        const response = await fetch('/api/pickit/rules/validate-nip', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                nipLine: nipPreview
            })
        });

        if (!response.ok) {
            throw new Error('Validation request failed');
        }

        const result = await response.json();

        if (result.valid) {
            showValidation('success', 'Rule is valid! ✓');
        } else {
            const errors = result.errors || [];
            showValidation('error', 'Invalid: ' + (errors.join(', ') || 'Unknown error'));
        }

        // Show warnings if any
        if (result.warnings && result.warnings.length > 0) {
            console.log('Validation warnings:', result.warnings);
            // Don't show warnings as validation errors, just log them
        }
    } catch (error) {
        console.error('Validation error:', error);
        showValidation('error', 'Validation failed: ' + error.message);
    }
}

async function saveRule() {
    const nipPreview = document.getElementById('nipPreview').textContent;

    if (!nipPreview || nipPreview.startsWith('//')) {
        showValidation('error', 'Please create a rule first');
        return;
    }

    if (!currentPickitPath) {
        showValidation('error', 'Please select a pickit folder first (use the Select Pickit Folder button)');
        return;
    }

    const fileName = document.getElementById('pickitFile').value || 'general.nip';

    try {
        // Use the simple append endpoint that doesn't require validation
        const response = await fetch(`/api/pickit/files/rules/append?path=${encodeURIComponent(currentPickitPath)}&file=${fileName}`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                nipLine: nipPreview
            })
        });

        if (!response.ok) {
            const errorData = await response.json().catch(() => ({ error: 'Unknown error' }));
            throw new Error(errorData.error || 'Failed to save rule');
        }

        const result = await response.json();

        if (result.success) {
            showValidation('success', result.message || `Rule saved to ${fileName}! ✓`);
            console.log('Saved rule to:', fileName);
        } else {
            showValidation('error', 'Failed to save rule: ' + (result.error || 'Unknown error'));
        }
    } catch (error) {
        console.error('Save error:', error);
        showValidation('error', 'Save failed: ' + error.message);
    }
}

function showValidation(type, message) {
    const validationDiv = document.getElementById('validation');
    validationDiv.textContent = message;
    validationDiv.className = type; // Use just the type class (error/success/warning)
    validationDiv.style.display = 'block';

    // Auto-hide success messages
    if (type === 'success') {
        setTimeout(() => {
            validationDiv.style.display = 'none';
        }, 3000);
    }
}

function searchItems() {
    const query = document.getElementById('searchInput').value.toLowerCase();
    let filtered = allItems.filter(item =>
        item.name.toLowerCase().includes(query) ||
        item.type.toLowerCase().includes(query)
    );

    // Apply category filter
    const category = document.getElementById('categoryFilter').value;
    if (category) {
        filtered = filtered.filter(item => item.category === category);
    }

    // Apply rarity filter
    const rarity = document.getElementById('rarityFilter')?.value;
    if (rarity) {
        filtered = filtered.filter(item => item.rarity === rarity);
    }

    renderItems(filtered);
}

function clearSearch() {
    document.getElementById('searchInput').value = '';
    document.getElementById('categoryFilter').value = '';
    const rarityFilter = document.getElementById('rarityFilter');
    if (rarityFilter) {
        rarityFilter.value = '';
    }
    renderItems(allItems);
}

async function loadPickitFile() {
    const fileName = document.getElementById('pickitFile').value;
    if (!fileName) {
        showValidation('warning', 'Please select a file');
        return;
    }

    try {
        const response = await fetch(`/api/pickit/files?path=${encodeURIComponent(currentPickitPath)}&file=${fileName}`);
        if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.error || 'Failed to load file');
        }

        const rules = await response.json();

        console.log('Loaded response:', rules); // Debug output

        if (rules && Array.isArray(rules) && rules.length > 0) {
            showValidation('success', `Loaded ${rules.length} rules from ${fileName}`);
            displayLoadedRules(rules, fileName);
            console.log('Loaded rules:', rules);
        } else {
            console.warn('Empty response or not an array:', rules);
            showValidation('warning', `File ${fileName} returned no rules. Check console for details.`);
        }
    } catch (error) {
        console.error('Load file error:', error);
        showValidation('error', 'Failed to load file: ' + error.message);
    }
}

function createNewFile() {
    if (!currentPickitPath) {
        showValidation('error', 'Please select a pickit folder first (use the Select Pickit Folder button)');
        return;
    }

    const fileName = prompt('Enter new pickit file name (e.g., my_rules.nip):');
    if (!fileName) {
        return; // User cancelled
    }

    if (!fileName.endsWith('.nip')) {
        showValidation('error', 'File name must end with .nip');
        return;
    }

    // Add to dropdown
    const select = document.getElementById('pickitFile');
    const option = document.createElement('option');
    option.value = fileName;
    option.textContent = fileName;
    select.appendChild(option);
    select.value = fileName;

    showValidation('success', `Ready to create ${fileName}. Save a rule to create the file.`);
}

function displayLoadedRules(rules, fileName) {
    const section = document.getElementById('loadedRulesSection');
    const list = document.getElementById('loadedRulesList');
    const count = document.getElementById('rulesCount');

    // Store current rules and file for edit/delete operations
    currentLoadedRules = rules;
    currentLoadedFile = fileName;

    count.textContent = rules.length;

    list.innerHTML = rules.map((rule, index) => {
        // The JSON field is "generatedNip" (lowercase 'n')
        const nipLine = rule.generatedNip || rule.GeneratedNIP || rule.nipSyntax || 'No NIP syntax available';
        return `
        <div style="background: #1e1e1e; padding: 12px; margin-bottom: 8px; border-radius: 4px; border-left: 3px solid #4CAF50;">
            <div style="display: flex; justify-content: space-between; align-items: start; margin-bottom: 6px;">
                <span style="color: #888; font-size: 12px;">Rule ${index + 1}</span>
                <div style="display: flex; gap: 8px;">
                    <button class="btn btn-secondary" style="padding: 4px 10px; font-size: 12px;" onclick="editRule('${rule.id}')">Edit</button>
                    <button class="btn btn-secondary" style="padding: 4px 10px; font-size: 12px;" onclick="deleteRule('${rule.id}')">Delete</button>
                </div>
            </div>
            <div style="font-family: 'Courier New', monospace; color: #e0e0e0; font-size: 13px; word-break: break-all;">
                ${nipLine}
            </div>
        </div>
        `;
    }).join('');

    section.style.display = 'block';

    // Scroll to the loaded rules section
    section.scrollIntoView({ behavior: 'smooth', block: 'start' });
}

function closeLoadedRules() {
    document.getElementById('loadedRulesSection').style.display = 'none';
}

let currentLoadedRules = [];
let currentLoadedFile = '';

async function editRule(ruleId) {
    // Find the rule in the current loaded rules
    const rule = currentLoadedRules.find(r => r.id === ruleId);
    if (!rule) {
        showValidation('error', 'Rule not found');
        return;
    }

    const currentNIP = rule.generatedNip || '';
    const newNIP = prompt('Edit NIP line:\n\n(Advanced: Be careful with syntax!)', currentNIP);

    if (newNIP === null) {
        // User cancelled
        return;
    }

    if (newNIP.trim() === '') {
        showValidation('error', 'NIP line cannot be empty');
        return;
    }

    if (newNIP === currentNIP) {
        showValidation('info', 'No changes made');
        return;
    }

    try {
        const response = await fetch(`/api/pickit/files/rules/update?path=${encodeURIComponent(currentPickitPath)}&file=${currentLoadedFile}&id=${ruleId}`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                newNipLine: newNIP
            })
        });

        if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.error || 'Failed to update rule');
        }

        const result = await response.json();
        showValidation('success', 'Rule updated successfully!');

        // Reload the file to show updated rules
        await reloadCurrentFile();
    } catch (error) {
        console.error('Update rule error:', error);
        showValidation('error', 'Failed to update rule: ' + error.message);
    }
}

async function deleteRule(ruleId) {
    // Find the rule in the current loaded rules
    const rule = currentLoadedRules.find(r => r.id === ruleId);
    if (!rule) {
        showValidation('error', 'Rule not found');
        return;
    }

    const nipLine = rule.generatedNip || 'this rule';

    // Show a custom confirmation dialog with the NIP line
    const confirmed = confirm(
        '⚠️ DELETE CONFIRMATION ⚠️\n\n' +
        'Are you sure you want to permanently delete this rule?\n\n' +
        'Rule: ' + nipLine + '\n\n' +
        'This action CANNOT be undone!'
    );

    if (!confirmed) {
        return;
    }

    try {
        const response = await fetch(`/api/pickit/files/rules/delete?path=${encodeURIComponent(currentPickitPath)}&file=${currentLoadedFile}&id=${ruleId}`, {
            method: 'POST'
        });

        if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.error || 'Failed to delete rule');
        }

        const result = await response.json();
        showValidation('success', 'Rule deleted successfully!');

        // Reload the file to show updated rules
        await reloadCurrentFile();
    } catch (error) {
        console.error('Delete rule error:', error);
        showValidation('error', 'Failed to delete rule: ' + error.message);
    }
}

async function reloadCurrentFile() {
    if (!currentLoadedFile) {
        return;
    }

    try {
        const response = await fetch(`/api/pickit/files?path=${encodeURIComponent(currentPickitPath)}&file=${currentLoadedFile}`);
        if (!response.ok) {
            const errorData = await response.json();
            throw new Error(errorData.error || 'Failed to reload file');
        }

        const rules = await response.json();
        currentLoadedRules = rules;
        displayLoadedRules(rules, currentLoadedFile);
    } catch (error) {
        console.error('Reload file error:', error);
        showValidation('error', 'Failed to reload file: ' + error.message);
    }
}

// Event listeners setup
document.addEventListener('DOMContentLoaded', function () {
    // Load data on page load
    loadData();
});
