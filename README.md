# RescueSimulationCM  
**Author:** Mattis Schulte  
**Date:** 2025-05-06  

---

## Overview

**RescueSimulationCM** is a browser-based demo where a single **Rescuer** agent navigates a procedurally generated circular maze to pick up scattered survivors and exit through one of several openings. It integrates:

- A **circular-maze generator** producing a perfect maze on concentric rings.  
- **Graph algorithms** for pathfinding and route planning:  
  - **BFS** to compute exact shortest-path distances and parent pointers.  
  - **Greedy nearest-neighbor** heuristic followed by **2-Opt** edge-swaps to refine the pickup tour.  
- An **AgentPy** model to simulate the rescue mission step by step.  
- A **PyScript**-powered UI with **SVG** rendering for interactive parameter tuning and real-time visualization.

Key steps in the pipeline:

1. Build an `R*S` circular grid of cells, each with four potential walls (`in`, `out`, `cw`, `ccw`).  
2. Carve a perfect maze by randomized DFS on rings 1 ... R − 1 (ring 0 remains a solid core).  
3. Randomly place `n_surv` survivors and designate `n_entr` entrances on the outer ring.  
4. Run BFS from each “key” node (entrances + survivors) to build a complete graph of pairwise distances and parent pointers.  
5. Compute a rescue tour:  
   - Use **Greedy nearest-neighbor** to get an initial route visiting all survivors.  
   - Improve it with **2-Opt** swaps to eliminate path cross-overs.  
6. Spawn agents:  
   - **Rescuer** follows the precomputed route cell by cell.  
   - **Survivor** agents are flagged rescued when the Rescuer reaches them.  
7. Render metrics and SVG: walls as arcs/lines, the rescuer’s path, survivors and exits as circles.

> For an interactive experience, explore the demo available [here](https://cloud-a2c8a5890d5a.mattisschulte.io/)

---

## Maze Data Structures

### Cell

Each cell in the circular grid is identified by `(r, s)` -> ring index `r` (0 = core) and sector index `s` (0 ... S - 1) and tracks which of its four boundaries are intact.

```python
class Cell:
    def __init__(self, ring: int, sector: int):
        self.ring = ring
        self.sector = sector
        # Walls: True = intact, False = open passage
        self.walls = {'in': True, 'out': True, 'cw': True, 'ccw': True}

    @property
    def key(self) -> tuple[int, int]:
        return (self.ring, self.sector)
```

- `'in'` / `'out'`: radial walls toward the core (ring 0) or perimeter (ring R - 1).  
- `'cw'` / `'ccw'`: angular walls to the next/previous sector (with wrap-around for `s = 0` or `s = S - 1`).

### CircularMaze

Constructs all cells and precomputes adjacency lists for O(1) neighbor lookups:

```python
class CircularMaze:
    def __init__(self, R: int, S: int):
        self.R, self.S = R, S
        # Map (r,s) → Cell
        self.cells = {
            (r, s): Cell(r, s)
            for r in range(R)
            for s in range(S)
        }
        self.neighbors = self._compute_neighbors()

    def _compute_neighbors(self) -> dict[tuple[int,int], list[tuple[str, Cell]]]:
        neigh = {}
        for (r, s), cell in self.cells.items():
            nbrs = []
            # radial inward
            if r > 0:
                nbrs.append(('in', self.cells[(r-1, s)]))
            # radial outward
            if r < self.R - 1:
                nbrs.append(('out', self.cells[(r+1, s)]))
            # angular neighbors (with wrap-around)
            nbrs.append(('cw',  self.cells[(r, (s+1) % self.S)]))
            nbrs.append(('ccw', self.cells[(r, (s-1) % self.S)]))
            neigh[(r, s)] = nbrs
        return neigh
```

- Time complexity: O(R*S) to build cells and adjacency.  
- Each neighbor entry is `(direction_label, neighbor_cell)`, so we can quickly test wall existence when moving.

---

## Perfect Maze Carving

We carve a **perfect maze** (exactly one simple path between any two cells, i.e., no loops) on rings 1 ... R - 1 using randomized Depth-First Search (DFS). Ring 0 remains a solid core.

```python
def carve_maze(self, seed: int = None):
    if seed is not None:
        random.seed(seed)

    # 1) Treat all ring-0 cells as 'visited' so they stay intact
    visited = {(0, s) for s in range(self.S)}
    if self.R < 2:
        return  # nothing to carve if only the core exists

    # 2) Start DFS from cell (1, 0)
    start = (1, 0)
    visited.add(start)
    stack = [start]

    # Helper to knock down opposite walls
    opposite = {'in':'out', 'out':'in', 'cw':'ccw', 'ccw':'cw'}

    # 3) Main DFS loop
    while stack:
        current = stack[-1]
        # a) Gather all adjacent unvisited neighbors
        choices = [
            (dir_, nbr.key)
            for dir_, nbr in self.neighbors[current]
            if nbr.key not in visited
        ]
        # b) If none left, backtrack
        if not choices:
            stack.pop()
            continue

        # c) Otherwise, pick one at random and carve a passage
        dir_, next_cell = random.choice(choices)
        # Remove the wall in both cells
        self.cells[current].walls[dir_] = False
        self.cells[next_cell].walls[opposite[dir_]] = False

        # d) Mark new cell visited and push onto stack
        visited.add(next_cell)
        stack.append(next_cell)
```

Algorithmic details:

- We maintain an explicit **stack** for backtracking (iterative DFS).  
- At each step we randomly choose an unvisited neighbor, remove the shared wall, and continue.  
- When no unvisited neighbors remain, we pop the stack to backtrack.  
- **Result**: each cell on rings 1 ... R − 1 is visited exactly once, producing a loop-free maze.

Time complexity remains O(R*S) since each cell is pushed/popped at most once, and neighbor checks are constant-time.

---

## Distance Mapping via BFS

To plan optimal movements, we run BFS from each key (entrance or survivor). BFS gives us both:

- `dist[v]`: the minimum number of steps from the source to `v`.  
- `parent[v]`: the predecessor of `v` in the BFS tree (for path reconstruction).

```python
from collections import deque

def bfs(self, start: tuple[int,int]) -> tuple[dict, dict]:
    dist = {start: 0}
    parent = {}
    queue = deque([start])

    while queue:
        u = queue.popleft()
        for direction, nbr in self.neighbors[u]:
            v = nbr.key
            # We can move only if that wall is open and we haven't visited v
            if not self.cells[u].walls[direction] and v not in dist:
                dist[v] = dist[u] + 1
                parent[v] = u
                queue.append(v)

    return dist, parent
```

How BFS works here:

1. Initialize `dist[start] = 0` and enqueue `start`.  
2. Repeatedly dequeue a cell `u`; for each neighbor `(direction, nbr)`:  
   - If the wall in `direction` is removed (`False`) and `nbr` unvisited, set its `dist` and `parent`, then enqueue.  
3. Continue until the queue is empty, covering the entire reachable component.

Running BFS from each of the `n_key = n_entr + n_surv` nodes costs O(n_key * R * S). We store:

- `dist_map[u][v]` = distance from `u` to `v`.  
- `parent_map[u]` = the parent dictionary from the BFS rooted at `u`.

We can later reconstruct the actual cell-by-cell path between any two keys by walking backwards from `v` to `u` via `parent_map[u]`.

---

## Route Planning: Greedy Nearest-Neighbor + 2-Opt

Given the complete graph on our key nodes (entrances and survivors) with exact distances, we estimate a optimal route using a two-step approach:

1. **Initial Construction: Greedy Nearest-Neighbor**
- For each possible **starting entrance**:  
  a. Initialize `current = start`, `remaining = {all survivors}`, `order = []`, `total = 0`.  
  b. While `remaining` is nonempty:  
    - Pick the **closest** survivor `s` in `remaining` according to `dist_map[current][s]`.  
    - Append `s` to `order`, add the distance to `total`, remove `s` from `remaining`, and set `current = s`.  
  c. Finally, return to the **nearest entrance** `e`:  
    - Find `e` minimizing `dist_map[current][e]`, add that to `total`.  
  d. Record the route `[start] + order + [e]` and its `total`.  
- Keep the best route over all starting entrances.

```python
best_route = None
best_cost  = float('inf')

for start in entrances:
    remaining = set(survivors)
    order = []
    total = 0
    current = start

    # Visit nearest survivor repeatedly
    while remaining:
        nxt = min(remaining, key=lambda s: dist_map[current].get(s, float('inf')))
        total += dist_map[current][nxt]
        order.append(nxt)
        remaining.remove(nxt)
        current = nxt

    # Return to nearest exit
    back_dist, best_exit = min(
        ((dist_map[current].get(e, float('inf')), e) for e in entrances),
        key=lambda x: x[0]
    )
    total += back_dist

    route = [start] + order + [best_exit]
    if total < best_cost:
        best_cost  = total
        best_route = (start, best_exit, order, total)
```

- Complexity: O(n_entr * n_surv^2), since each `min` over `remaining` costs O(n_surv).

2. **Route Refinement: 2-Opt**
The 2-Opt heuristic iteratively looks for two edges in the current key-node tour whose endpoints can be reconnected to shorten the total distance, effectively “uncrossing” paths.

```python
def two_opt(path: list, dist_map: dict, max_loops: int = 100) -> list:
    best = path.copy()
    n = len(best)
    for _ in range(max_loops):
        improved = False
        # Try all possible edge-pairs (i-1 → i) and (j → j+1)
        for i in range(1, n - 2):
            for j in range(i + 1, n - 1):
                a, b = best[i-1], best[i]
                c, d = best[j], best[j+1]
                curr = dist_map[a][b] + dist_map[c][d]
                swap = dist_map[a][c] + dist_map[b][d]
                if swap + 1e-9 < curr:
                    # Perform the 2-opt swap: reverse the sub-route [i ... j]
                    best[i:j+1] = reversed(best[i:j+1])
                    improved = True
        if not improved:
            break
    return best
```

- At each iteration, we scan O(n^2) pairs and swap if the total length reduces.  
- Loop until no improvement or `max_loops` reached.  
- Complexity: O(max_loops * (n_entr + n_surv)^2).

3. **Finalize**
For the best greedy route, apply `two_opt` to its `[start, pickups, end]` sequence. Recompute the true total cost from `dist_map` and accept if improved.

---

## AgentPy Model

We simulate the rescue mission using AgentPy with two agent types:

### Survivor Agent

```python
class Survivor(ap.Agent):
    def setup(self):
        self.rescued = False
        # self.position will be assigned by the model
```

- Remains stationary at its sampled cell.
- Flagged `rescued = True` when the Rescuer arrives.

### Rescuer Agent

```python
class Rescuer(ap.Agent):
    def setup(self, path: list[tuple[int,int]]):
        self.path = path
        self.step_index = 0
        self.position = path[0]
        self.carry = []

    def step(self):
        if self.step_index < len(self.path) - 1:
            self.step_index += 1
            self.position = self.path[self.step_index]
            # Pick up survivors at the new position
            for s in self.model.survivors:
                if not s.rescued and s.position == self.position:
                    s.rescued = True
                    self.carry.append(s)
```

- Follows exactly the planned route, one cell per step.
- On arrival, it marks survivors as `rescued` and adds them to its `carry` list.

### RescueModel

```python
class RescueModel(ap.Model):
    def setup(self):
        # 1) Generate maze
        self.maze = CircularMaze(self.p.R, self.p.S)
        self.maze.carve_maze(self.p.seed)

        # 2) Sample survivors & entrances
        all_inner = [(r, s) for r in range(1, self.p.R) for s in range(self.p.S)]
        surv_positions = random.sample(all_inner, self.p.n_surv)
        entr_sectors = random.sample(range(self.p.S), self.p.n_entr)
        entrances = [(self.p.R - 1, s) for s in entr_sectors]

        # 3) Compute best route
        (start, end, order, total), parent_map = find_best_route(
            self.maze, entrances, surv_positions
        )
        full_route = [start] + order + [end]

        # Store for reporting & rendering
        self.start_cell      = start
        self.end_cell        = end
        self.pickup_order    = order
        self.total_steps     = total
        self.parent_map      = parent_map
        self.full_route      = full_route
        self.entrance_sectors= set(entr_sectors)

        # 4) Spawn agents
        self.survivors = ap.AgentList(self, self.p.n_surv, Survivor)
        for agent, pos in zip(self.survivors, surv_positions):
            agent.position = pos

        self.rescuer = ap.AgentList(self, 1, Rescuer, path=full_route)
        self.steps_to_run = len(full_route) - 1

    def step(self):
        self.rescuer.step()

    def end(self):
        # Optionally collect final statistics
        pass
```

- Runs exactly `len(full_route) - 1` steps, one per cell move.
- Collects survivors along the way.

---

## SVG Rendering

We generate an inline SVG that visualizes:

1. Maze walls as circular arcs (for angular walls) and radial lines (for in/out walls).
2. Planned route by reconstructing each segment cell-by-cell using the corresponding `parent_map`.  
3. Survivors and entrances/exits as colored markers.

```python
def render_svg(
    maze: CircularMaze,
    survivors: list[tuple[int,int]],
    start: tuple[int,int],
    pickups: list[tuple[int,int]],
    end: tuple[int,int],
    entrance_sectors: set[int],
    parent_map: dict
) -> str:
    # Compute drawing parameters (radii, angles, styles)...
    # 1) Draw walls
    # 2) Draw path segments
    # 3) Draw survivor & entrance markers
    return svg_string
```

---

## PyScript UI Integration

We bind our model and rendering to HTML controls so users can interactively tune parameters.

```python
from pyodide.ffi import create_proxy
from pyscript import document

def on_generate(event):
    # 1) Read & clamp form inputs (R, S, n_surv, n_entr, seed, etc.)
    # 2) Instantiate and run RescueModel
    # 3) Compute metrics (avg/max BFS distance from start, wall density, symmetry, etc.)
    # 4) Update <div id="metrics"> and inject SVG into <div id="svg-container">

generate_proxy = create_proxy(on_generate)
document.getElementById("generate") \
        .addEventListener("click", generate_proxy)
```

---

## Performance & Scaling (approximately)

- Maze carving: O(R * S)
- Single BFS: O(R * S)
- Complete-graph build (n_key BFS runs): O((n_entr + n_surv) * R * S)
- Greedy route: O(n_entr * n_surv^2)
- 2-Opt refinement: O(max_loops * (n_entr + n_surv)^2)
- SVG rendering: O(R * S) primitives

## References
- Maze generation algorithms, including randomized DFS: [Wikipedia, "Maze generation algorithm"](https://en.wikipedia.org/wiki/Maze_generation_algorithm#Iterative_implementation_(with_stack))
- Breadth-First Search (BFS) for shortest paths: [Wikipedia, "Breadth-first search"](https://en.wikipedia.org/wiki/Breadth-first_search#Pseudocode)
- Nearest-neighbor heuristics and 2-Opt for TSP: [Wikipedia, "Traveling salesman problem"](https://en.wikipedia.org/wiki/Travelling_salesman_problem#Constructive_heuristics); [Wikipedia, "2-opt"](https://en.wikipedia.org/wiki/2-opt#Pseudocode)
- AgentPy for agent-based modeling: [AgentPy Documentation](https://agentpy.readthedocs.io/en/latest/)
- PyScript for Python in the browser: [PyScript Documentation](https://docs.pyscript.net/)
- SVG rendering in HTML: [MDN Web Docs, "Scalable Vector Graphics (SVG)"](https://developer.mozilla.org/en-US/docs/Web/SVG)