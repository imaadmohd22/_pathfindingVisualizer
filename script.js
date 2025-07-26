// script.js

// --- Global Variables ---
// Default grid dimensions
let numRows = 20;
let numCols = 50;
let grid = []; // This 2D array will store the state of each cell (e.g., wall, visited, path)

// Start and End node coordinates
// These will be objects like {row: 5, col: 10}
let startNode = null;
let endNode = null;

// Flags for mouse drawing
let isDrawing = false; // True when mouse is down and drawing walls/weights
let nodeTypeToDraw = 'wall'; // What type of node to draw (wall, start, end, weight)
let movingStartOrEnd = false; // True if we are moving the start or end node

// Animation speed control
let animationSpeed = 50; // Milliseconds per step
let animationTimeoutIds = []; // To store timeouts for clearing animations

// Diagonal movement toggle
let allowDiagonal = false; // Default to no diagonal movement

// --- DOM Element References ---
const gridContainer = document.getElementById('gridContainer');
const visualizeBtn = document.getElementById('visualizeBtn');
const clearBoardBtn = document.getElementById('clearBoardBtn');
const clearPathBtn = document.getElementById('clearPathBtn');
const algorithmSelect = document.getElementById('algorithmSelect');
const nodeTypeRadios = document.querySelectorAll('input[name="nodeType"]');
const animationSpeedSlider = document.getElementById('animationSpeed');
const speedValueSpan = document.getElementById('speedValue');
const numRowsInput = document.getElementById('numRows');
const numColsInput = document.getElementById('numCols');
const applyGridSizeBtn = document.getElementById('applyGridSizeBtn');
const generateMazeBtn = document.getElementById('generateMazeBtn');
const allowDiagonalCheckbox = document.getElementById('allowDiagonal'); // New DOM reference

// New DOM references for statistics
const statisticsPanel = document.getElementById('statistics');
const pathLengthSpan = document.getElementById('pathLength');
const pathCostSpan = document.getElementById('pathCost');
const nodesVisitedSpan = document.getElementById('nodesVisited');
const timeTakenSpan = document.getElementById('timeTaken');

// New DOM references for Save/Load
const saveGridBtn = document.getElementById('saveGridBtn');
const loadGridBtn = document.getElementById('loadGridBtn');

// --- Helper Function: Node Constructor ---
// Represents a single node (cell) in our grid
class Node {
    constructor(row, col) {
        this.row = row;
        this.col = col;
        this.isStart = false;
        this.isEnd = false;
        this.isWall = false;
        this.isWeight = false; // Represents a weighted node (e.g., higher cost to traverse)
        this.distance = Infinity; // For Dijkstra/A*: distance from start node (gScore in A*)
        this.isVisited = false; // Has this node been visited by the algorithm?
        this.previousNode = null; // To reconstruct the shortest path
        this.gScore = Infinity; // For A*: cost from start to current node
        this.fScore = Infinity; // For A*: gScore + heuristic (total estimated cost)
        this.heuristic = 0; // For A*: estimated cost from current node to end node
        this.weight = 1; // Default weight, can be changed for weighted nodes (e.g., 5)
    }
}

// --- Priority Queue (Min-Heap) Implementation for Dijkstra's and A* ---
// This is a simplified Priority Queue. For very large grids, a more optimized
// binary heap implementation would be beneficial for performance.
class PriorityQueue {
    constructor() {
        this.values = [];
    }

    enqueue(node, priority) {
        this.values.push({ node, priority });
        this.sort(); // Simple sort, not efficient for large queues
    }

    dequeue() {
        return this.values.shift(); // Removes and returns the first element (lowest priority)
    }

    sort() {
        this.values.sort((a, b) => a.priority - b.priority);
    }

    isEmpty() {
        return this.values.length === 0;
    }
}


// --- Core Function: Create Grid ---
function createGrid(loadedGridData = null) { // Added loadedGridData parameter
    console.log('createGrid() called.');
    console.log('gridContainer:', gridContainer);

    if (!gridContainer) {
        console.error('Error: gridContainer element not found!');
        return; // Stop execution if container is missing
    }

    gridContainer.innerHTML = ''; // Clear existing grid
    grid = []; // Reset the grid array

    // Set grid dimensions from inputs, ensuring they are valid numbers
    // If loading from data, use loaded dimensions
    numRows = loadedGridData ? loadedGridData.numRows : (parseInt(numRowsInput.value) || 20);
    numCols = loadedGridData ? loadedGridData.numCols : (parseInt(numColsInput.value) || 50);

    // Clamp values to reasonable limits to prevent performance issues or absurdly large grids
    numRows = Math.max(5, Math.min(numRows, 50));
    numCols = Math.max(5, Math.min(numCols, 100));

    // Update input values to reflect clamped numbers (important if loaded data changed them)
    numRowsInput.value = numRows;
    numColsInput.value = numCols;

    console.log(`Grid dimensions: ${numRows} rows, ${numCols} columns.`);

    // Dynamically set CSS grid properties
    gridContainer.style.gridTemplateColumns = `repeat(${numCols}, 1fr)`;
    console.log(`gridContainer.style.gridTemplateColumns set to: ${gridContainer.style.gridTemplateColumns}`);

    // --- CRITICAL FIX: Ensure gridContainer has a width before calculating cellSize ---
    // If gridContainer.offsetWidth is 0, cells will have 0 height.
    // We'll try to get the width, and if it's 0, we'll set a default or wait.
    let containerWidth = gridContainer.offsetWidth;
    let containerHeight = gridContainer.offsetHeight; // Also check height
    console.log(`gridContainer.offsetWidth: ${containerWidth}, gridContainer.offsetHeight: ${containerHeight}`);

    let cellSize;
    if (containerWidth === 0) {
        console.warn("gridContainer.offsetWidth is 0. Using default cell size of 25px.");
        cellSize = 25; // Fallback to 25px if container width is not available
        gridContainer.style.width = '100%'; // Ensure it takes full width of its parent
        // Also ensure gridContainer has some height if it's collapsing
        gridContainer.style.minHeight = `${numRows * cellSize}px`;
    } else {
        cellSize = Math.floor(containerWidth / numCols);
        // Ensure minimum cell size
        cellSize = Math.max(cellSize, 10); // Don't let cells become too small
        console.log(`Calculated cellSize: ${cellSize}px.`);
    }
    gridContainer.style.gridAutoRows = `${cellSize}px`; // Ensure rows have same height as columns
    console.log(`gridContainer.style.gridAutoRows set to: ${gridContainer.style.gridAutoRows}`);


    for (let r = 0; r < numRows; r++) {
        const currentRow = [];
        for (let c = 0; c < numCols; c++) {
            const node = new Node(r, c);
            currentRow.push(node);

            const cellElement = document.createElement('div');
            cellElement.classList.add('grid-cell');
            cellElement.dataset.row = r; // Store row and column as data attributes
            cellElement.dataset.col = c;

            // Apply loaded state if available
            if (loadedGridData && loadedGridData.grid[r][c]) {
                const loadedNode = loadedGridData.grid[r][c];
                node.isWall = loadedNode.isWall;
                node.isWeight = loadedNode.isWeight;
                node.weight = loadedNode.weight;

                if (node.isWall) cellElement.classList.add('wall');
                if (node.isWeight) cellElement.classList.add('weight');
            }

            // Add event listeners for drawing/placing nodes
            cellElement.addEventListener('mousedown', handleMouseDown);
            cellElement.addEventListener('mouseenter', handleMouseEnter);

            gridContainer.appendChild(cellElement);
            // Log for the first few cells to confirm they are created
            if (r === 0 && c < 5) {
                console.log(`Created cell [${r},${c}]:`, cellElement);
            }
        }
        grid.push(currentRow);
    }
    console.log(`Total cells created and appended: ${numRows * numCols}`);

    // Set default start and end nodes if not already set
    // Or reset them if grid size changed significantly
    if (loadedGridData && loadedGridData.startNode && loadedGridData.endNode) {
        // Load start/end from saved data
        startNode = grid[loadedGridData.startNode.row][loadedGridData.startNode.col];
        endNode = grid[loadedGridData.endNode.row][loadedGridData.endNode.col];
        allowDiagonal = loadedGridData.allowDiagonal || false; // Load diagonal setting
        allowDiagonalCheckbox.checked = allowDiagonal; // Update checkbox state
        console.log('Loaded start/end nodes and diagonal setting from saved data.');
    } else {
        // Use default start/end nodes if no loaded data or invalid
        console.log('Setting default start/end nodes.');
        startNode = grid[Math.floor(numRows / 2)][Math.floor(numCols / 4)];
        endNode = grid[Math.floor(numRows / 2)][Math.floor(numCols * 3 / 4)];
    }

    // Ensure start/end nodes are marked and displayed correctly
    if (startNode) {
        startNode.isStart = true;
        startNode.isWall = false; // Ensure start/end are not walls
        startNode.isWeight = false;
        startNode.weight = 1;
        document.querySelector(`[data-row="${startNode.row}"][data-col="${startNode.col}"]`).classList.add('start');
        document.querySelector(`[data-row="${startNode.row}"][data-col="${startNode.col}"]`).classList.remove('wall', 'weight');
    }
    if (endNode) {
        endNode.isEnd = true;
        endNode.isWall = false; // Ensure start/end are not walls
        endNode.isWeight = false;
        endNode.weight = 1;
        document.querySelector(`[data-row="${endNode.row}"][data-col="${endNode.col}"]`).classList.add('end');
        document.querySelector(`[data-row="${endNode.row}"][data-col="${endNode.col}"]`).classList.remove('wall', 'weight');
    }
    console.log('createGrid() finished.');
}

// --- Event Handlers for Drawing ---
function handleMouseDown(e) {
    isDrawing = true;
    const row = parseInt(e.target.dataset.row);
    const col = parseInt(e.target.dataset.col);
    const node = grid[row][col];

    // Determine what type of node we are interacting with
    if (node.isStart) {
        movingStartOrEnd = 'start';
    } else if (node.isEnd) {
        movingStartOrEnd = 'end';
    } else {
        // If not moving start/end, set the node type based on current selection
        // Toggle wall/weight if clicked directly, otherwise apply selected type
        if (nodeTypeToDraw === 'wall') {
            node.isWall = !node.isWall;
            e.target.classList.toggle('wall', node.isWall);
            e.target.classList.remove('weight'); // Remove weight if it was a wall
            node.isWeight = false;
            node.weight = 1;
        } else if (nodeTypeToDraw === 'weight') {
            node.isWeight = !node.isWeight;
            e.target.classList.toggle('weight', node.isWeight);
            e.target.classList.remove('wall'); // Remove wall if it was a weight
            node.isWall = false;
            node.weight = node.isWeight ? 5 : 1; // Assign a higher weight (e.g., 5)
        }
    }
}

function handleMouseUp() {
    isDrawing = false;
    movingStartOrEnd = false; // Reset moving flag
}

function handleMouseEnter(e) {
    if (!isDrawing) return; // Only draw if mouse is down

    const row = parseInt(e.target.dataset.row);
    const col = parseInt(e.target.dataset.col);
    const node = grid[row][col];

    if (movingStartOrEnd === 'start') {
        // Clear old start node
        if (startNode) {
            document.querySelector(`[data-row="${startNode.row}"][data-col="${startNode.col}"]`).classList.remove('start');
            startNode.isStart = false;
        }
        // Set new start node
        startNode = node;
        startNode.isStart = true;
        e.target.classList.add('start');
        // Ensure it's not a wall or weight
        node.isWall = false;
        node.isWeight = false;
        node.weight = 1;
        e.target.classList.remove('wall', 'weight');
    } else if (movingStartOrEnd === 'end') {
        // Clear old end node
        if (endNode) {
            document.querySelector(`[data-row="${endNode.row}"][data-col="${endNode.col}"]`).classList.remove('end');
            endNode.isEnd = false;
        }
        // Set new end node
        endNode = node;
        endNode.isEnd = true;
        e.target.classList.add('end');
        // Ensure it's not a wall or weight
        node.isWall = false;
        node.isWeight = false;
        node.weight = 1;
        e.target.classList.remove('wall', 'weight');
    } else {
        // If drawing walls/weights
        if (!node.isStart && !node.isEnd) { // Cannot draw over start/end nodes
            if (nodeTypeToDraw === 'wall') {
                node.isWall = true;
                node.isWeight = false; // Ensure it's not a weight
                node.weight = 1;
                e.target.classList.add('wall');
                e.target.classList.remove('weight');
            } else if (nodeTypeToDraw === 'weight') {
                node.isWeight = true;
                node.isWall = false; // Ensure it's not a wall
                node.weight = 5; // Assign higher weight
                e.target.classList.add('weight');
                e.target.classList.remove('wall');
            }
        }
    }
}


// --- Pathfinding Algorithms ---

// BFS (Breadth-First Search) Algorithm
async function bfs(startNode, endNode) {
    const queue = [startNode];
    const visitedNodesInOrder = []; // To store nodes in the order they are visited for animation
    startNode.isVisited = true;

    while (queue.length > 0) {
        const currentNode = queue.shift(); // Get the first node from the queue

        // If we reached the end node, we can stop
        if (currentNode === endNode) {
            return visitedNodesInOrder; // Return all visited nodes for animation
        }

        // Skip walls
        if (currentNode.isWall) continue;

        visitedNodesInOrder.push(currentNode);

        // Get neighbors (up, down, left, right)
        // Note: For BFS, the order of neighbors doesn't strictly matter for correctness,
        // but it can affect the visual "spread" slightly.
        const neighbors = getNeighbors(currentNode); // Use general getNeighbors

        for (const neighbor of neighbors) {
            if (!neighbor.isVisited && !neighbor.isWall) {
                neighbor.isVisited = true;
                neighbor.previousNode = currentNode; // Set previous node to reconstruct path
                queue.push(neighbor);
            }
        }
    }
    return visitedNodesInOrder; // No path found
}

// DFS (Depth-First Search) Algorithm
async function dfs(startNode, endNode) {
    // DFS typically uses a stack or recursion. We'll use an array as a stack.
    const stack = [startNode];
    const visitedNodesInOrder = [];
    startNode.isVisited = true;

    while (stack.length > 0) {
        const currentNode = stack.pop(); // Get the last node from the stack (LIFO)

        // If we reached the end node, we can stop
        if (currentNode === endNode) {
            return visitedNodesInOrder;
        }

        // Skip walls
        if (currentNode.isWall) continue;

        visitedNodesInOrder.push(currentNode);

        // Get neighbors (up, down, left, right).
        // For DFS, the order of adding neighbors to the stack matters for the path found
        // if multiple paths exist. Reversing ensures a consistent "depth-first" exploration.
        const neighbors = getNeighbors(currentNode); // Use general getNeighbors
        // Reverse neighbors to make sure the first neighbor (e.g., top) is processed last (pushed first)
        // so that the last neighbor (e.g., right) is processed first (popped last).
        // This simulates a typical recursive DFS behavior.
        neighbors.reverse(); 

        for (const neighbor of neighbors) {
            if (!neighbor.isVisited && !neighbor.isWall) {
                neighbor.isVisited = true;
                neighbor.previousNode = currentNode;
                stack.push(neighbor);
            }
        }
    }
    return visitedNodesInOrder; // No path found
}

// Dijkstra's Algorithm
async function dijkstra(startNode, endNode) {
    const visitedNodesInOrder = [];
    const priorityQueue = new PriorityQueue();

    // Initialize distances: all to Infinity, startNode to 0
    // Reset all node properties relevant to Dijkstra's
    for (let r = 0; r < numRows; r++) {
        for (let c = 0; c < numCols; c++) {
            const node = grid[r][c];
            node.distance = Infinity; // Reset distance property on the node itself
            node.previousNode = null; // Reset previousNode
            node.isVisited = false; // Reset visited status
        }
    }
    startNode.distance = 0;
    priorityQueue.enqueue(startNode, 0); // Add start node to priority queue with distance 0

    while (!priorityQueue.isEmpty()) {
        const { node: currentNode, priority: currentDistance } = priorityQueue.dequeue();

        // If we already processed this node with a shorter path, skip
        // This check is important for efficiency with the simple PriorityQueue
        if (currentNode.isVisited) continue;

        // Mark as visited and add to visited order for animation
        currentNode.isVisited = true;
        visitedNodesInOrder.push(currentNode);

        // If we reached the end node, we can stop
        if (currentNode === endNode) {
            return visitedNodesInOrder;
        }

        // Skip walls
        if (currentNode.isWall) continue;

        // Explore neighbors
        const neighbors = getNeighbors(currentNode); // Use general getNeighbors

        for (const neighbor of neighbors) {
            if (neighbor.isWall) continue; // Cannot pass through walls

            // Calculate new distance to neighbor
            // Cost of moving to a diagonal neighbor is sqrt(2)
            const moveCost = (currentNode.row !== neighbor.row && currentNode.col !== neighbor.col) ? Math.SQRT2 : 1;
            const newDistance = currentNode.distance + (neighbor.weight * moveCost); // Add neighbor's weight and move cost

            // If a shorter path to the neighbor is found
            if (newDistance < neighbor.distance) {
                neighbor.distance = newDistance; // Update distance on the node
                neighbor.previousNode = currentNode; // Set previous node
                priorityQueue.enqueue(neighbor, newDistance); // Add/update in priority queue
            }
        }
    }
    return visitedNodesInOrder; // No path found
}

// Heuristic function (Manhattan or Euclidean distance for A*)
function heuristic(nodeA, nodeB) {
    const dx = Math.abs(nodeA.row - nodeB.row);
    const dy = Math.abs(nodeA.col - nodeB.col);

    if (allowDiagonal) {
        // Euclidean distance for diagonal movement
        // Use D = 1, D2 = sqrt(2) for diagonal cost
        // return Math.sqrt(dx * dx + dy * dy); // This is pure Euclidean
        // For grid, a common optimization is D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy)
        // where D=1, D2=sqrt(2). This is Chebyshev distance if D=1, D2=1, or Octile if D=1, D2=sqrt(2)
        const D = 1;
        const D2 = Math.SQRT2;
        return D * (dx + dy) + (D2 - 2 * D) * Math.min(dx, dy);
    } else {
        // Manhattan distance for cardinal movement
        return dx + dy;
    }
}

// A* Search Algorithm
async function astar(startNode, endNode) {
    console.log("A* algorithm started.");
    const visitedNodesInOrder = [];
    const priorityQueue = new PriorityQueue();

    // Reset all node properties relevant to A*
    for (let r = 0; r < numRows; r++) {
        for (let c = 0; c < numCols; c++) {
            const node = grid[r][c];
            node.gScore = Infinity; // Cost from start to current node
            node.fScore = Infinity; // gScore + heuristic (estimated total total cost)
            node.previousNode = null;
            node.isVisited = false; // A* also marks nodes as visited
        }
    }

    startNode.gScore = 0;
    startNode.fScore = heuristic(startNode, endNode);
    priorityQueue.enqueue(startNode, startNode.fScore); // Enqueue with fScore as priority
    console.log("Start node enqueued:", startNode);

    while (!priorityQueue.isEmpty()) {
        const { node: currentNode, priority: currentFScore } = priorityQueue.dequeue();
        console.log(`Dequeued node [${currentNode.row},${currentNode.col}] with fScore: ${currentFScore}`);

        // If we already processed this node with a better path (lower fScore), skip
        // This check is crucial for efficiency with the simple PriorityQueue
        if (currentNode.isVisited) {
            console.log(`Node [${currentNode.row},${currentNode.col}] already visited with a better path. Skipping.`);
            continue;
        }

        // Mark as visited and add to visited order for animation
        currentNode.isVisited = true;
        visitedNodesInOrder.push(currentNode);
        console.log(`Visiting node [${currentNode.row},${currentNode.col}]`);


        // If we reached the end node, we can stop
        if (currentNode === endNode) {
            console.log("End node reached!");
            return visitedNodesInOrder;
        }

        // Skip walls
        if (currentNode.isWall) {
            console.log(`Node [${currentNode.row},${currentNode.col}] is a wall. Skipping.`);
            continue;
        }

        // Explore neighbors
        const neighbors = getNeighbors(currentNode);
        console.log(`Exploring neighbors of [${currentNode.row},${currentNode.col}]:`, neighbors.map(n => `[${n.row},${n.col}]`));


        for (const neighbor of neighbors) {
            if (neighbor.isWall) {
                console.log(`Neighbor [${neighbor.row},${neighbor.col}] is a wall. Skipping.`);
                continue; // Cannot pass through walls
            }

            // Calculate tentative gScore (cost from start to neighbor through current)
            // Cost of moving to a diagonal neighbor is sqrt(2)
            const moveCost = (currentNode.row !== neighbor.row && currentNode.col !== neighbor.col) ? Math.SQRT2 : 1;
            const tentativeGScore = currentNode.gScore + (neighbor.weight * moveCost);
            console.log(`Neighbor [${neighbor.row},${neighbor.col}]: tentativeGScore = ${tentativeGScore}, currentGScore = ${neighbor.gScore}`);

            // If a shorter path to the neighbor is found
            if (tentativeGScore < neighbor.gScore) {
                neighbor.previousNode = currentNode;
                neighbor.gScore = tentativeGScore;
                neighbor.fScore = neighbor.gScore + heuristic(neighbor, endNode); // Calculate new fScore
                priorityQueue.enqueue(neighbor, neighbor.fScore); // CORRECTED: Enqueue with fScore
                console.log(`Updated neighbor [${neighbor.row},${neighbor.col}]: gScore=${neighbor.gScore}, fScore=${neighbor.fScore}. Enqueued.`);
            }
        }
    }
    console.log('No path found by A*.');
    return visitedNodesInOrder; // No path found
}


// Get neighbors of a node (used by all algorithms now)
function getNeighbors(node) {
    const neighbors = [];
    const { row, col } = node;

    // Cardinal directions (Up, Down, Left, Right)
    const cardinalDirections = [
        { dr: -1, dc: 0 }, // Up
        { dr: 1, dc: 0 },  // Down
        { dr: 0, dc: -1 }, // Left
        { dr: 0, dc: 1 }   // Right
    ];

    // Diagonal directions
    const diagonalDirections = [
        { dr: -1, dc: -1 }, // Top-Left
        { dr: -1, dc: 1 },  // Top-Right
        { dr: 1, dc: -1 },  // Bottom-Left
        { dr: 1, dc: 1 }    // Bottom-Right
    ];

    let directionsToConsider = [...cardinalDirections];
    if (allowDiagonal) {
        directionsToConsider = [...cardinalDirections, ...diagonalDirections];
    }

    for (const dir of directionsToConsider) {
        const newRow = row + dir.dr;
        const newCol = col + dir.dc;

        // Check bounds
        if (newRow >= 0 && newRow < numRows && newCol >= 0 && newCol < numCols) {
            const neighbor = grid[newRow][newCol];
            // Get all valid non-wall neighbors. Algorithms will handle visited status.
            if (!neighbor.isWall) {
                neighbors.push(neighbor);
            }
        }
    }
    return neighbors;
}

// Reconstruct the shortest path from endNode to startNode
function getShortestPathNodes(endNode) {
    const shortestPathNodes = [];
    let currentNode = endNode;
    while (currentNode !== null) {
        shortestPathNodes.unshift(currentNode); // Add to the beginning
        currentNode = currentNode.previousNode;
    }
    return shortestPathNodes;
}

// --- Visualization Functions ---

// Animates the visited nodes and then the shortest path
async function animatePathfinding(visitedNodesInOrder, shortestPathNodes) {
    // Clear any previous animations
    clearAnimationTimeouts();

    // Animate visited nodes
    for (let i = 0; i < visitedNodesInOrder.length; i++) {
        const node = visitedNodesInOrder[i];
        // Skip start and end nodes as they have their own colors
        if (node.isStart || node.isEnd) continue;

        const cellElement = document.querySelector(`[data-row="${node.row}"][data-col="${node.col}"]`);
        if (cellElement) {
            const timeoutId = setTimeout(() => {
                cellElement.classList.add('visited');
            }, animationSpeed * i);
            animationTimeoutIds.push(timeoutId);
        }
    }

    // Animate shortest path after visited nodes animation is complete
    // Add a small buffer after visited nodes animation to ensure it completes before path starts
    const pathAnimationDelay = animationSpeed * visitedNodesInOrder.length + 100; // Added 100ms buffer
    for (let i = 0; i < shortestPathNodes.length; i++) {
        const node = shortestPathNodes[i];
        // Skip start and end nodes
        if (node.isStart || node.isEnd) continue;

        const cellElement = document.querySelector(`[data-row="${node.row}"][data-col="${node.col}"]`);
        if (cellElement) {
            const timeoutId = setTimeout(() => {
                cellElement.classList.add('path');
            }, pathAnimationDelay + (animationSpeed * i));
            animationTimeoutIds.push(timeoutId);
        }
    }
}

// Clears all scheduled animation timeouts
function clearAnimationTimeouts() {
    animationTimeoutIds.forEach(id => clearTimeout(id));
    animationTimeoutIds = [];
}

// --- Maze Generation Algorithm: Recursive Division ---
async function generateMaze() {
    console.log('Generating maze...');
    clearBoard(); // Start with a clear board

    // Make all cells walls initially
    for (let r = 0; r < numRows; r++) {
        for (let c = 0; c < numCols; c++) {
            const node = grid[r][c];
            node.isWall = true;
            document.querySelector(`[data-row="${r}"][data-col="${c}"]`).classList.add('wall');
        }
    }

    // Clear the start and end nodes from being walls
    if (startNode) {
        startNode.isWall = false;
        document.querySelector(`[data-row="${startNode.row}"][data-col="${startNode.col}"]`).classList.remove('wall');
    }
    if (endNode) {
        endNode.isWall = false;
        document.querySelector(`[data-row="${endNode.row}"][data-col="${endNode.col}"]`).classList.remove('wall');
    }

    // Define the outer boundaries of the maze (all walls)
    // Then call recursive division on the inner chamber
    // We need to carve out paths from the walls, so start with everything as a wall
    // and then recursively open up paths.

    // Recursive Division function
    // R_min, R_max, C_min, C_max define the current chamber boundaries
    async function recursiveDivide(rMin, rMax, cMin, cMax) {
        const height = rMax - rMin + 1;
        const width = cMax - cMin + 1;

        if (height < 3 || width < 3) { // Base case: chamber is too small to divide further
            return;
        }

        // Choose orientation: horizontal or vertical
        const horizontal = height > width;

        if (horizontal) {
            // Choose a random row to place the wall (must be an even row within the chamber)
            // The wall itself will be on an odd row index if the chamber starts on an even row,
            // or an even row index if the chamber starts on an odd row.
            // This ensures passages can be placed on even/odd rows.
            let wallRow = rMin + 1 + 2 * Math.floor(Math.random() * ((rMax - 1 - (rMin + 1)) / 2 + 1));
            if (wallRow % 2 !== 0 && (rMax - rMin + 1) % 2 === 0) { // Adjust if wallRow is odd and chamber height is even
                wallRow = rMin + 1 + 2 * Math.floor(Math.random() * ((rMax - 1 - (rMin + 1)) / 2));
            }

            // Choose a random column for the passage (must be an even column within the chamber)
            let passageCol = cMin + 2 * Math.floor(Math.random() * ((cMax - cMin + 1) / 2));
            if (passageCol % 2 !== 0 && (cMax - cMin + 1) % 2 === 0) { // Adjust if passageCol is odd and chamber width is even
                 passageCol = cMin + 2 * Math.floor(Math.random() * ((cMax - cMin + 1) / 2));
            }

            // Draw horizontal wall
            for (let c = cMin; c <= cMax; c++) {
                if (c === passageCol) { // Create a passage
                    grid[wallRow][c].isWall = false;
                    document.querySelector(`[data-row="${wallRow}"][data-col="${c}"]`).classList.remove('wall');
                } else {
                    grid[wallRow][c].isWall = true;
                    document.querySelector(`[data-row="${wallRow}"][data-col="${c}"]`).classList.add('wall');
                }
                await new Promise(resolve => setTimeout(resolve, animationSpeed / 5)); // Small delay for animation
            }

            // Recursively divide the two new sub-chambers
            await recursiveDivide(rMin, wallRow - 1, cMin, cMax);
            await recursiveDivide(wallRow + 1, rMax, cMin, cMax);

        } else { // Vertical division
            // Choose a random column to place the wall (must be an even column within the chamber)
            let wallCol = cMin + 1 + 2 * Math.floor(Math.random() * ((cMax - 1 - (cMin + 1)) / 2 + 1));
            if (wallCol % 2 !== 0 && (cMax - cMin + 1) % 2 === 0) { // Adjust if wallCol is odd and chamber width is even
                wallCol = cMin + 1 + 2 * Math.floor(Math.random() * ((cMax - 1 - (cMin + 1)) / 2));
            }

            // Choose a random row for the passage (must be an even row within the chamber)
            let passageRow = rMin + 2 * Math.floor(Math.random() * ((rMax - rMin + 1) / 2));
            if (passageRow % 2 !== 0 && (rMax - rMin + 1) % 2 === 0) { // Adjust if passageRow is odd and chamber height is even
                passageRow = rMin + 2 * Math.floor(Math.random() * ((rMax - rMin + 1) / 2));
            }

            // Draw vertical wall
            for (let r = rMin; r <= rMax; r++) {
                if (r === passageRow) { // Create a passage
                    grid[r][wallCol].isWall = false;
                    document.querySelector(`[data-row="${r}"][data-col="${wallCol}"]`).classList.remove('wall');
                } else {
                    grid[r][wallCol].isWall = true;
                    document.querySelector(`[data-row="${r}"][data-col="${wallCol}"]`).classList.add('wall');
                }
                await new Promise(resolve => setTimeout(resolve, animationSpeed / 5)); // Small delay for animation
            }

            // Recursively divide the two new sub-chambers
            await recursiveDivide(rMin, rMax, cMin, wallCol - 1);
            await recursiveDivide(rMin, rMax, wallCol + 1, cMax);
        }
    }

    // Initial call to recursiveDivide for the entire grid
    // Ensure start and end nodes are not walls after maze generation
    // We need to make sure the outer border is always a wall for recursive division to work well
    for (let r = 0; r < numRows; r++) {
        for (let c = 0; c < numCols; c++) {
            if (r === 0 || r === numRows - 1 || c === 0 || c === numCols - 1) {
                grid[r][c].isWall = true;
                document.querySelector(`[data-row="${r}"][data-col="${c}"]`).classList.add('wall');
            } else {
                grid[r][c].isWall = false; // Clear inner cells
                document.querySelector(`[data-row="${r}"][data-col="${c}"]`).classList.remove('wall');
            }
        }
    }

    // Ensure start and end nodes are clear paths
    if (startNode) {
        startNode.isWall = false;
        document.querySelector(`[data-row="${startNode.row}"][data-col="${startNode.col}"]`).classList.remove('wall');
    }
    if (endNode) {
        endNode.isWall = false;
        document.querySelector(`[data-row="${endNode.row}"][data-col="${endNode.col}"]`).classList.remove('wall');
    }

    await recursiveDivide(1, numRows - 2, 1, numCols - 2); // Divide the inner part of the grid

    // Ensure start and end nodes are not walls, just in case they were covered by maze generation
    if (startNode) {
        startNode.isWall = false;
        document.querySelector(`[data-row="${startNode.row}"][data-col="${startNode.col}"]`).classList.remove('wall');
    }
    if (endNode) {
        endNode.isWall = false;
        document.querySelector(`[data-row="${endNode.row}"][data-col="${endNode.col}"]`).classList.remove('wall');
    }
    console.log('Maze generation finished.');
}


// --- Event Listeners ---
// Changed from DOMContentLoaded to window.onload for better layout readiness
window.onload = () => {
    console.log('window.onload fired. Calling createGrid().');
    createGrid(); // Initial grid creation on page load
};

// Listen for mouse up anywhere on the document to stop drawing
document.addEventListener('mouseup', handleMouseUp);

// Handle node type selection
nodeTypeRadios.forEach(radio => {
    radio.addEventListener('change', (e) => {
        nodeTypeToDraw = e.target.value;
        console.log('Node type selected:', nodeTypeToDraw);
    });
});

// Handle animation speed slider
animationSpeedSlider.addEventListener('input', (e) => {
    animationSpeed = parseInt(e.target.value);
    speedValueSpan.textContent = `${animationSpeed}ms`;
    console.log('Animation speed set to:', animationSpeed);
});

// Handle Apply Grid Size button click
applyGridSizeBtn.addEventListener('click', () => {
    console.log('Apply Grid Size button clicked!');
    clearBoard(); // Clear board before regenerating grid
    createGrid(); // Regenerate grid with new dimensions
});

// Handle Allow Diagonal Movement checkbox
allowDiagonalCheckbox.addEventListener('change', (e) => {
    allowDiagonal = e.target.checked;
    console.log('Allow Diagonal Movement:', allowDiagonal);
    // It's good practice to clear the path when this changes, as path cost changes
    clearPath();
});


// Visualize button click handler
visualizeBtn.addEventListener('click', async () => {
    console.log('Visualize button clicked!');
    clearPath(); // Clear any previous path/visited nodes
    clearAnimationTimeouts(); // Stop any ongoing animations
    hideStatistics(); // Hide statistics until new ones are calculated


    // Reset node states for the algorithm
    grid.forEach(row => row.forEach(node => {
        node.isVisited = false;
        node.distance = Infinity; // Used by Dijkstra
        node.previousNode = null;
        node.gScore = Infinity; // Used by A*
        node.fScore = Infinity; // Used by A*
        node.heuristic = 0; // Recalculated by A*
    }));

    // Ensure start and end nodes are correctly set in the grid object
    // This is crucial if the grid was regenerated or nodes were moved
    startNode = grid[startNode.row][startNode.col];
    endNode = grid[endNode.row][endNode.col];
    startNode.isStart = true;
    endNode.isEnd = true;


    const selectedAlgorithm = algorithmSelect.value;
    let visitedNodesInOrder = [];
    let shortestPathNodes = [];

    // Check if start or end node is a wall
    if (startNode.isWall) {
        console.warn("Start node cannot be a wall!");
        // You might want to add a visual message to the user here
        return;
    }
    if (endNode.isWall) {
        console.warn("End node cannot be a wall!");
        // You might want to add a visual message to the user here
        return;
    }

    const startTime = performance.now(); // Start timer
    switch (selectedAlgorithm) {
        case 'bfs':
            console.log('Running BFS...');
            visitedNodesInOrder = await bfs(startNode, endNode);
            shortestPathNodes = getShortestPathNodes(endNode);
            break;
        case 'dfs':
            console.log('Running DFS...');
            visitedNodesInOrder = await dfs(startNode, endNode);
            shortestPathNodes = getShortestPathNodes(endNode); // DFS also uses previousNode for path
            break;
        case 'dijkstra':
            console.log('Running Dijkstra...');
            visitedNodesInOrder = await dijkstra(startNode, endNode);
            shortestPathNodes = getShortestPathNodes(endNode);
            break;
        case 'astar':
            console.log('Running A* Search...');
            visitedNodesInOrder = await astar(startNode, endNode);
            shortestPathNodes = getShortestPathNodes(endNode);
            break;
        default:
            console.warn('No algorithm selected or unknown algorithm.');
            return;
    }
    const endTime = performance.now(); // End timer
    const timeTaken = (endTime - startTime).toFixed(2); // Format to 2 decimal places

    if (visitedNodesInOrder.length > 0) {
        await animatePathfinding(visitedNodesInOrder, shortestPathNodes);
        displayStatistics(shortestPathNodes, visitedNodesInOrder.length, timeTaken, selectedAlgorithm);
    } else {
        console.log('No path found or algorithm did not run.');
        displayStatistics([], visitedNodesInOrder.length, timeTaken, selectedAlgorithm); // Display stats even if no path
        // You might want to add a visual message to the user here
    }
});

// Clear Board button click handler
clearBoardBtn.addEventListener('click', () => {
    console.log('Clear Board button clicked!');
    clearAnimationTimeouts(); // Stop any ongoing animations
    hideStatistics(); // Hide statistics
    clearBoard();
    createGrid(); // Re-create grid to reset start/end nodes cleanly
});

// Clear Path button click handler
clearPathBtn.addEventListener('click', () => {
    console.log('Clear Path button clicked!');
    clearAnimationTimeouts(); // Stop any ongoing animations
    hideStatistics(); // Hide statistics
    clearPath();
});

// Generate Maze button click handler
generateMazeBtn.addEventListener('click', async () => {
    console.log('Generate Maze button clicked!');
    clearAnimationTimeouts();
    hideStatistics(); // Hide statistics
    await generateMaze(); // Call the new maze generation function
});

// Save Grid button click handler
saveGridBtn.addEventListener('click', () => {
    console.log('Save Grid button clicked!');
    saveGrid();
});

// Load Grid button click handler
loadGridBtn.addEventListener('click', () => {
    console.log('Load Grid button clicked!');
    loadGrid();
});

// --- Utility Functions (will be expanded) ---
function clearBoard() {
    grid.forEach(row => {
        row.forEach(node => {
            node.isWall = false;
            node.isWeight = false;
            node.isVisited = false;
            node.isPath = false; // Will be used later for path visualization
            node.distance = Infinity;
            node.previousNode = null;
            node.gScore = Infinity;
            node.fScore = Infinity;
            node.heuristic = 0;
            node.weight = 1;
            const cellElement = document.querySelector(`[data-row="${node.row}"][data-col="${node.col}"]`);
            if (cellElement) {
                cellElement.className = 'grid-cell'; // Reset all classes
                // Ensure start/end nodes retain their properties in the Node object
                if (node.isStart) cellElement.classList.add('start');
                if (node.isEnd) cellElement.classList.add('end');
            }
        });
    });
    // Re-apply start/end node classes after clearing all others
    // This is important because clearBoard() resets all classes, including 'start'/'end'
    if (startNode) {
        document.querySelector(`[data-row="${startNode.row}"][data-col="${startNode.col}"]`).classList.add('start');
    }
    if (endNode) {
        document.querySelector(`[data-row="${endNode.row}"][data-col="${endNode.col}"]`).classList.add('end');
    }
    console.log('Board cleared.');
}

function clearPath() {
    grid.forEach(row => {
        row.forEach(node => {
            if (!node.isStart && !node.isEnd && !node.isWall && !node.isWeight) {
                // Only clear visited/path nodes, leave walls/weights
                node.isVisited = false;
                node.isPath = false;
                node.distance = Infinity;
                node.previousNode = null;
                node.gScore = Infinity;
                node.fScore = Infinity;
                node.heuristic = 0;
                const cellElement = document.querySelector(`[data-row="${node.row}"][data-col="${node.col}"]`);
                if (cellElement) {
                    cellElement.classList.remove('visited', 'path');
                }
            }
        });
    });
    console.log('Path cleared.');
}

// --- Statistics Functions ---
function displayStatistics(shortestPathNodes, nodesVisitedCount, timeTaken, algorithmName) {
    statisticsPanel.classList.remove('hidden'); // Show the statistics panel

    pathLengthSpan.textContent = shortestPathNodes.length > 0 ? (shortestPathNodes.length - 1) : 0; // Path length is nodes - 1 edges

    let totalPathCost = 0;
    if (shortestPathNodes.length > 1) {
        // Calculate path cost by summing weights of nodes in the path (excluding start)
        // and adding move costs for diagonal moves
        for (let i = 1; i < shortestPathNodes.length; i++) {
            const currentNode = shortestPathNodes[i];
            const previousNode = shortestPathNodes[i - 1];
            // Recalculate moveCost based on whether it was a diagonal move
            const moveCost = (currentNode.row !== previousNode.row && currentNode.col !== previousNode.col) ? Math.SQRT2 : 1;
            totalPathCost += currentNode.weight * moveCost;
        }
    }
    pathCostSpan.textContent = totalPathCost.toFixed(2); // Display cost with 2 decimal places

    nodesVisitedSpan.textContent = nodesVisitedCount;
    timeTakenSpan.textContent = `${timeTaken} ms`;
    console.log(`Statistics for ${algorithmName}: Path Length = ${pathLengthSpan.textContent}, Path Cost = ${pathCostSpan.textContent}, Nodes Visited = ${nodesVisitedSpan.textContent}, Time Taken = ${timeTakenSpan.textContent}`);
}

function hideStatistics() {
    statisticsPanel.classList.add('hidden');
    pathLengthSpan.textContent = '0';
    pathCostSpan.textContent = '0';
    nodesVisitedSpan.textContent = '0';
    timeTakenSpan.textContent = '0 ms';
}

// --- Save/Load Grid Functions ---
function saveGrid() {
    // Prepare data to be saved: only essential properties
    const serializableGrid = grid.map(row =>
        row.map(node => ({
            row: node.row,
            col: node.col,
            isWall: node.isWall,
            isWeight: node.isWeight,
            weight: node.weight
        }))
    );

    const savedData = {
        grid: serializableGrid,
        startNode: { row: startNode.row, col: startNode.col },
        endNode: { row: endNode.row, col: endNode.col },
        numRows: numRows,
        numCols: numCols,
        allowDiagonal: allowDiagonal
    };

    try {
        localStorage.setItem('pathfindingGrid', JSON.stringify(savedData));
        console.log('Grid saved successfully!');
        alert('Grid saved successfully!'); // Use alert for simplicity, could be a custom modal
    } catch (e) {
        console.error('Failed to save grid to local storage:', e);
        alert('Error saving grid. Local storage might be full or unavailable.');
    }
}

function loadGrid() {
    try {
        const savedDataString = localStorage.getItem('pathfindingGrid');
        if (savedDataString) {
            const loadedData = JSON.parse(savedDataString);
            console.log('Loaded data:', loadedData);

            // Clear current board and re-create grid with loaded dimensions and node types
            clearBoard(); // Clear existing visual and logical state
            createGrid(loadedData); // Pass loaded data to createGrid

            // The createGrid(loadedData) already sets numRows, numCols, allowDiagonal, startNode, endNode
            // We just need to ensure the radio buttons for nodeTypeToDraw are reset if needed
            // For now, nodeTypeToDraw remains as it was before loading, which is fine.

            alert('Grid loaded successfully!');
            console.log('Grid loaded successfully!');
        } else {
            alert('No saved grid found!');
            console.log('No saved grid found.');
        }
    } catch (e) {
        console.error('Failed to load grid from local storage:', e);
        alert('Error loading grid. Saved data might be corrupted.');
    }
}
