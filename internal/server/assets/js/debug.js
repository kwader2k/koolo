/**
 * Debug Interface
 * Tree view and real-time debugging interface
 */

// ========================================
// CONSTANTS
// ========================================
const DEBUG_CONFIG = {
    DEFAULT_REFRESH_INTERVAL: 1000,
    MIN_REFRESH_INTERVAL: 100,
    MAX_REFRESH_INTERVAL: 10000,
    LARGE_ARRAY_THRESHOLD: 100,
    LARGE_ARRAY_PREVIEW: 100
};

// ========================================
// STATE MANAGEMENT
// ========================================
class DebugState {
    constructor() {
        this.refreshInterval = DEBUG_CONFIG.DEFAULT_REFRESH_INTERVAL;
        this.refreshIntervalId = null;
        this.previousData = null;
        this.isAllExpanded = true;
        this.searchMatches = [];
        this.currentSearchMatch = -1;
        this.lastSearchTerm = '';
        this.expandedState = {};
        this.currentSearchMatchPath = null;
    }

    setRefreshInterval(interval) {
        this.refreshInterval = Math.max(
            DEBUG_CONFIG.MIN_REFRESH_INTERVAL,
            Math.min(DEBUG_CONFIG.MAX_REFRESH_INTERVAL, interval)
        );
    }

    toggleExpandAll() {
        this.isAllExpanded = !this.isAllExpanded;
        return this.isAllExpanded;
    }

    setExpanded(path, value) {
        this.expandedState[path] = value;
    }

    isExpanded(path) {
        return this.expandedState[path] !== false;
    }
}

// ========================================
// DOM UTILITIES
// ========================================
const DebugDOM = {
    elements: {
        container: null,
        refreshIntervalInput: null,
        setIntervalBtn: null,
        expandAllBtn: null,
        supervisorName: null,
        searchInput: null,
        searchPrevBtn: null,
        searchNextBtn: null,
        searchResults: null,
        copyDataBtn: null
    },

    init() {
        this.elements.container = document.getElementById('debug-container');
        this.elements.refreshIntervalInput = document.getElementById('refresh-interval');
        this.elements.setIntervalBtn = document.getElementById('set-interval-btn');
        this.elements.expandAllBtn = document.getElementById('expand-all-btn');
        this.elements.supervisorName = document.getElementById('supervisor-name');
        this.elements.searchInput = document.getElementById('search-input');
        this.elements.searchPrevBtn = document.getElementById('search-prev-btn');
        this.elements.searchNextBtn = document.getElementById('search-next-btn');
        this.elements.searchResults = document.getElementById('search-results');
        this.elements.copyDataBtn = document.getElementById('copy-data-btn');
    },

    createElement(tag, attributes = {}, children = []) {
        const element = document.createElement(tag);

        Object.entries(attributes).forEach(([key, value]) => {
            if (key === 'className') {
                element.className = value;
            } else if (key === 'dataset') {
                Object.entries(value).forEach(([dataKey, dataValue]) => {
                    element.dataset[dataKey] = dataValue;
                });
            } else if (key.startsWith('on')) {
                const eventName = key.slice(2).toLowerCase();
                element.addEventListener(eventName, value);
            } else {
                element.setAttribute(key, value);
            }
        });

        children.forEach(child => {
            if (typeof child === 'string') {
                element.appendChild(document.createTextNode(child));
            } else if (child instanceof Node) {
                element.appendChild(child);
            }
        });

        return element;
    },

    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
};

// ========================================
// TREE VIEW BUILDER
// ========================================
class TreeViewBuilder {
    constructor(state) {
        this.state = state;
    }

    createTreeView(data, path = '') {
        const fragment = document.createDocumentFragment();

        Object.entries(data).forEach(([key, value]) => {
            if (key === 'CollisionGrid') return; // Skip CollisionGrid

            const currentPath = path ? `${path}.${key}` : key;
            const node = this.createTreeNode(key, value, currentPath);
            fragment.appendChild(node);
        });

        return fragment;
    }

    createTreeNode(key, value, currentPath) {
        const node = DebugDOM.createElement('div', {
            className: 'tree-node',
            dataset: { path: currentPath }
        });

        const label = DebugDOM.createElement('span', { className: 'tree-label' });

        if (typeof value === 'object' && value !== null) {
            this.createObjectNode(node, label, key, value, currentPath);
        } else {
            this.createLeafNode(label, key, value, currentPath);
        }

        node.appendChild(label);
        return node;
    }

    createObjectNode(node, label, key, value, currentPath) {
        const toggle = DebugDOM.createElement('span', {
            className: 'tree-toggle',
            onClick: () => TreeViewController.toggleNode(currentPath)
        }, [this.state.isExpanded(currentPath) ? '▼' : '▶']);

        const keySpan = DebugDOM.createElement('span', {
            className: 'tree-key'
        }, [DebugDOM.escapeHtml(key)]);

        const copyBtn = this.createCopyButton(currentPath);

        label.appendChild(toggle);
        label.appendChild(keySpan);
        label.appendChild(copyBtn);

        const isLargeArray = Array.isArray(value) && value.length > DEBUG_CONFIG.LARGE_ARRAY_THRESHOLD;

        if (isLargeArray) {
            const arrayPreview = DebugDOM.createElement('div', {
                className: 'large-dataset-notice'
            }, [`Array with ${value.length} items (showing first ${DEBUG_CONFIG.LARGE_ARRAY_PREVIEW})`]);
            node.appendChild(arrayPreview);

            const previewData = Object.fromEntries(
                value.slice(0, DEBUG_CONFIG.LARGE_ARRAY_PREVIEW).map((item, index) => [index, item])
            );
            this.createChildContainer(node, previewData, currentPath);
        } else {
            this.createChildContainer(node, value, currentPath);
        }
    }

    createLeafNode(label, key, value, currentPath) {
        const keySpan = DebugDOM.createElement('span', {
            className: 'tree-key'
        }, [DebugDOM.escapeHtml(`${key}: `)]);

        const valueClass = value === null ? 'tree-value null' : 'tree-value';
        const valueSpan = DebugDOM.createElement('span', {
            className: valueClass
        }, [JSON.stringify(value)]);

        const copyBtn = this.createCopyButton(currentPath);

        label.appendChild(keySpan);
        label.appendChild(valueSpan);
        label.appendChild(copyBtn);
    }

    createChildContainer(node, value, currentPath) {
        const childrenContainer = DebugDOM.createElement('div', {
            style: `display: ${this.state.isExpanded(currentPath) ? 'block' : 'none'}`
        });

        childrenContainer.appendChild(this.createTreeView(value, currentPath));
        node.appendChild(childrenContainer);
    }

    createCopyButton(path) {
        return DebugDOM.createElement('button', {
            className: 'copy-btn',
            dataset: { clipboardPath: path },
            onClick: (e) => this.handleCopy(e, path)
        }, ['Copy']);
    }

    async handleCopy(event, path) {
        event.stopPropagation();
        const button = event.target;

        try {
            const value = this.getValueByPath(this.state.previousData, path);
            const textToCopy = JSON.stringify(value, null, 2);
            await navigator.clipboard.writeText(textToCopy);

            button.textContent = 'Copied!';
            setTimeout(() => {
                button.textContent = 'Copy';
            }, 2000);
        } catch (error) {
            console.error('Failed to copy:', error);
        }
    }

    getValueByPath(obj, path) {
        return path.split('.').reduce((acc, part) => acc && acc[part], obj);
    }
}

// ========================================
// TREE VIEW CONTROLLER
// ========================================
class TreeViewController {
    static init(state) {
        this.state = state;
        this.builder = new TreeViewBuilder(state);
    }

    static updateDebugContainer(data) {
        if (!DebugDOM.elements.container) return;

        const newTree = this.builder.createTreeView(data);
        DebugDOM.elements.container.innerHTML = '';
        DebugDOM.elements.container.appendChild(newTree);

        this.state.previousData = JSON.parse(JSON.stringify(data));
        this.updateExpandAllButton();

        if (this.state.lastSearchTerm) {
            SearchController.performSearch(this.state.lastSearchTerm, false);
        }
    }

    static toggleNode(path) {
        this.state.setExpanded(path, !this.state.isExpanded(path));

        const node = DebugDOM.elements.container.querySelector(`[data-path="${path}"]`);
        if (!node) return;

        const label = node.querySelector('.tree-label');
        const toggle = label.querySelector('.tree-toggle');
        const childContainer = label.nextElementSibling;

        if (toggle && childContainer) {
            toggle.textContent = this.state.isExpanded(path) ? '▼' : '▶';
            childContainer.style.display = this.state.isExpanded(path) ? 'block' : 'none';
        }
    }

    static toggleExpandAll() {
        const isExpanded = this.state.toggleExpandAll();
        const allNodes = DebugDOM.elements.container.querySelectorAll('.tree-node');

        allNodes.forEach(node => {
            const path = node.dataset.path;
            this.state.setExpanded(path, isExpanded);

            const label = node.querySelector('.tree-label');
            const toggle = label.querySelector('.tree-toggle');
            const childContainer = label.nextElementSibling;

            if (toggle && childContainer) {
                toggle.textContent = isExpanded ? '▼' : '▶';
                childContainer.style.display = isExpanded ? 'block' : 'none';
            }
        });

        this.updateExpandAllButton();
    }

    static updateExpandAllButton() {
        if (!DebugDOM.elements.expandAllBtn) return;

        const span = DebugDOM.elements.expandAllBtn.querySelector('span');
        if (span) {
            span.textContent = this.state.isAllExpanded ? 'Collapse All' : 'Expand All';
        }
    }
}

// ========================================
// SEARCH CONTROLLER
// ========================================
class SearchController {
    static init(state) {
        this.state = state;
    }

    static performSearch(searchTerm = DebugDOM.elements.searchInput?.value, shouldScroll = true) {
        searchTerm = searchTerm.toLowerCase();
        this.state.lastSearchTerm = searchTerm;
        this.state.searchMatches = [];
        this.state.currentSearchMatch = -1;

        if (searchTerm) {
            this.searchRecursive(DebugDOM.elements.container, searchTerm);
            this.updateSearchResults();

            if (this.state.searchMatches.length > 0) {
                if (this.state.currentSearchMatchPath) {
                    this.state.currentSearchMatch = this.state.searchMatches.findIndex(
                        node => node.dataset.path === this.state.currentSearchMatchPath
                    );
                    if (this.state.currentSearchMatch === -1) {
                        this.state.currentSearchMatch = 0;
                    }
                } else {
                    this.state.currentSearchMatch = 0;
                }
                this.highlightAndScrollToCurrentSearchResult(shouldScroll);
            }
        } else {
            this.clearSearchHighlights();
            this.state.currentSearchMatchPath = null;
            this.updateSearchResults();
        }
    }

    static searchRecursive(node, searchTerm) {
        if (node.classList && node.classList.contains('tree-node')) {
            const label = node.querySelector('.tree-label');
            const text = label.textContent.toLowerCase();
            if (text.includes(searchTerm)) {
                this.state.searchMatches.push(node);
            }
        }

        Array.from(node.children).forEach(child => {
            this.searchRecursive(child, searchTerm);
        });
    }

    static updateSearchResults() {
        if (!DebugDOM.elements.searchResults) return;

        DebugDOM.elements.searchResults.textContent = this.state.searchMatches.length > 0
            ? `${this.state.currentSearchMatch + 1}/${this.state.searchMatches.length}`
            : '0/0';
    }

    static goToNextSearchResult() {
        if (this.state.searchMatches.length === 0) return;

        this.state.currentSearchMatch = (this.state.currentSearchMatch + 1) % this.state.searchMatches.length;
        this.highlightAndScrollToCurrentSearchResult(true);
    }

    static goToPreviousSearchResult() {
        if (this.state.searchMatches.length === 0) return;

        this.state.currentSearchMatch =
            (this.state.currentSearchMatch - 1 + this.state.searchMatches.length) % this.state.searchMatches.length;
        this.highlightAndScrollToCurrentSearchResult(true);
    }

    static highlightAndScrollToCurrentSearchResult(shouldScroll = true) {
        this.clearSearchHighlights();

        if (this.state.currentSearchMatch >= 0 &&
            this.state.currentSearchMatch < this.state.searchMatches.length) {

            const node = this.state.searchMatches[this.state.currentSearchMatch];
            if (!node) return;

            const label = node.querySelector('.tree-label');
            if (label) {
                label.classList.add('highlight');
                this.expandToNode(node);
                this.state.currentSearchMatchPath = node.dataset.path;

                if (shouldScroll) {
                    setTimeout(() => {
                        label.scrollIntoView({ behavior: 'smooth', block: 'center' });
                    }, 100);
                }
            }
        }

        this.updateSearchResults();
    }

    static clearSearchHighlights() {
        const highlightedElements = DebugDOM.elements.container.querySelectorAll('.highlight');
        highlightedElements.forEach(el => el.classList.remove('highlight'));
    }

    static expandToNode(node) {
        let current = node;
        while (current && current !== DebugDOM.elements.container) {
            if (current.classList && current.classList.contains('tree-node')) {
                const path = current.dataset.path;
                if (path) {
                    TreeViewController.state.setExpanded(path, true);
                    const label = current.querySelector('.tree-label');
                    const toggle = label?.querySelector('.tree-toggle');
                    const childContainer = label?.nextElementSibling;

                    if (toggle && childContainer) {
                        toggle.textContent = '▼';
                        childContainer.style.display = 'block';
                    }
                }
            }
            current = current.parentElement;
        }
    }
}

// ========================================
// DATA FETCHER
// ========================================
class DataFetcher {
    static async fetchDebugData() {
        const urlParams = new URLSearchParams(window.location.search);
        const characterName = urlParams.get('characterName') || 'nullref';

        try {
            const response = await fetch(`/debug-data?characterName=${encodeURIComponent(characterName)}`);
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data = await response.json();
            delete data.CollisionGrid;

            TreeViewController.updateDebugContainer(data);

            if (DebugDOM.elements.supervisorName) {
                const supervisorName = data.PlayerUnit?.Name || characterName;
                DebugDOM.elements.supervisorName.textContent = `Supervisor: ${supervisorName}`;
            }
        } catch (error) {
            console.error('Error fetching debug data:', error);
            if (DebugDOM.elements.container) {
                DebugDOM.elements.container.innerHTML = '<p class="error-message">Error fetching debug data</p>';
            }
        }
    }
}

// ========================================
// REFRESH CONTROLLER
// ========================================
class RefreshController {
    static init(state) {
        this.state = state;
        this.startAutoRefresh();
    }

    static setRefreshInterval() {
        if (!DebugDOM.elements.refreshIntervalInput) return;

        const newInterval = parseInt(DebugDOM.elements.refreshIntervalInput.value, 10) * 1000;

        if (isNaN(newInterval) || newInterval <= 0) {
            alert('Please enter a valid refresh interval in seconds');
            return;
        }

        this.state.setRefreshInterval(newInterval);
        this.stopAutoRefresh();
        this.startAutoRefresh();

        console.log(`Refresh interval set to ${newInterval}ms`);
    }

    static startAutoRefresh() {
        this.state.refreshIntervalId = setInterval(
            () => DataFetcher.fetchDebugData(),
            this.state.refreshInterval
        );
    }

    static stopAutoRefresh() {
        if (this.state.refreshIntervalId) {
            clearInterval(this.state.refreshIntervalId);
            this.state.refreshIntervalId = null;
        }
    }
}

// ========================================
// COPY DATA HANDLER
// ========================================
class CopyDataHandler {
    static init(state) {
        this.state = state;
        this.setupCopyButton();
    }

    static setupCopyButton() {
        if (!DebugDOM.elements.copyDataBtn) return;

        DebugDOM.elements.copyDataBtn.addEventListener('click', async () => {
            try {
                const textToCopy = JSON.stringify(this.state.previousData, null, 2);
                await navigator.clipboard.writeText(textToCopy);

                const originalHTML = DebugDOM.elements.copyDataBtn.innerHTML;
                DebugDOM.elements.copyDataBtn.innerHTML = 'Copied!';

                setTimeout(() => {
                    DebugDOM.elements.copyDataBtn.innerHTML = originalHTML;
                }, 2000);
            } catch (error) {
                console.error('Failed to copy data:', error);
                alert('Failed to copy data to clipboard');
            }
        });
    }
}

// ========================================
// EVENT LISTENERS SETUP
// ========================================
class EventListeners {
    static init(state) {
        this.state = state;
        this.setupAllListeners();
    }

    static setupAllListeners() {
        // Refresh interval button
        if (DebugDOM.elements.setIntervalBtn) {
            DebugDOM.elements.setIntervalBtn.addEventListener('click', () => {
                RefreshController.setRefreshInterval();
            });
        }

        // Expand/Collapse all button
        if (DebugDOM.elements.expandAllBtn) {
            DebugDOM.elements.expandAllBtn.addEventListener('click', () => {
                TreeViewController.toggleExpandAll();
            });
        }

        // Search input
        if (DebugDOM.elements.searchInput) {
            DebugDOM.elements.searchInput.addEventListener('input', () => {
                SearchController.performSearch();
            });

            // Allow Enter key to search
            DebugDOM.elements.searchInput.addEventListener('keydown', (e) => {
                if (e.key === 'Enter') {
                    SearchController.goToNextSearchResult();
                }
            });
        }

        // Search navigation buttons
        if (DebugDOM.elements.searchNextBtn) {
            DebugDOM.elements.searchNextBtn.addEventListener('click', () => {
                SearchController.goToNextSearchResult();
            });
        }

        if (DebugDOM.elements.searchPrevBtn) {
            DebugDOM.elements.searchPrevBtn.addEventListener('click', () => {
                SearchController.goToPreviousSearchResult();
            });
        }

        // Keyboard shortcuts
        document.addEventListener('keydown', (e) => {
            // Ctrl/Cmd + F to focus search
            if ((e.ctrlKey || e.metaKey) && e.key === 'f') {
                e.preventDefault();
                DebugDOM.elements.searchInput?.focus();
            }

            // Escape to clear search
            if (e.key === 'Escape' && document.activeElement === DebugDOM.elements.searchInput) {
                DebugDOM.elements.searchInput.value = '';
                SearchController.performSearch('');
            }
        });
    }
}

// ========================================
// INITIALIZATION
// ========================================
const debugState = new DebugState();

// Initialize on DOM ready
document.addEventListener('DOMContentLoaded', () => {
    // Initialize DOM elements
    DebugDOM.init();

    // Initialize controllers
    TreeViewController.init(debugState);
    SearchController.init(debugState);
    RefreshController.init(debugState);
    CopyDataHandler.init(debugState);
    EventListeners.init(debugState);

    // Fetch initial data
    DataFetcher.fetchDebugData();
});

// Cleanup on page unload
window.addEventListener('beforeunload', () => {
    RefreshController.stopAutoRefresh();
});

// Expose global functions for inline event handlers (if needed)
window.toggleNode = (path) => TreeViewController.toggleNode(path);
