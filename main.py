import math
import random
from collections import deque

import agentpy as ap
from pyodide.ffi import create_proxy
from pyscript import document


# ----------------------------------------
#  Helper classes & functions
# ----------------------------------------

class Cell:
    """A single cell in the circular maze."""
    def __init__(self, ring: int, sector: int):
        """
        Initialize a cell.

        :param ring: The index of the concentric ring this cell belongs to.
        :param sector: The sector index within the ring.
        """
        self.ring, self.sector = ring, sector
        self.walls = {'in': True, 'out': True, 'cw': True, 'ccw': True}

    @property
    def key(self) -> tuple[int, int]:
        """
        The unique key identifying this cell.

        :return: Tuple of (ring, sector).
        """
        return (self.ring, self.sector)


class CircularMaze:
    """A maze on concentric rings divided into sectors."""

    def __init__(self, R: int, S: int):
        """
        Initialize a circular maze.

        :param R: Number of rings.
        :param S: Number of sectors per ring.
        """
        self.R, self.S = R, S
        self.cells = {
            (r, s): Cell(r, s)
            for r in range(R)
            for s in range(S)
        }
        self.neighbors = self._compute_neighbors()

    def _compute_neighbors(self) -> dict:
        """
        Precompute neighbor cells for each cell in the maze.

        :return: A dict mapping cell keys to lists of (direction, neighbor_cell).
        """
        neigh = {}
        for (r, s), cell in self.cells.items():
            nbrs = []
            if r > 0:
                nbrs.append(('in', self.cells[(r - 1, s)]))
            if r < self.R - 1:
                nbrs.append(('out', self.cells[(r + 1, s)]))
            nbrs.append(('cw', self.cells[(r, (s + 1) % self.S)]))
            nbrs.append(('ccw', self.cells[(r, (s - 1) % self.S)]))
            neigh[(r, s)] = nbrs
        return neigh

    def carve_maze(self, seed: int = None):
        """
        Carve a perfect maze using depth‐first search backtracking,
        but leave ring 0 completely untouched.
        """
        if seed is not None:
            random.seed(seed)

        # 1) Mark all cells in ring 0 as visited so we never carve into them
        visited = {(0, s) for s in range(self.S)}

        # If there's no ring 1, there is nothing to carve
        if self.R < 2:
            return

        # 2) Start the DFS from an arbitrary cell in ring 1 (say sector 0)
        start = (1, 0)
        visited.add(start)
        stack = [start]

        opposite = {'in': 'out', 'out': 'in', 'cw': 'ccw', 'ccw': 'cw'}

        while stack:
            current = stack[-1]
            # Only consider neighbors that are not yet visited
            choices = [
                (dir_, nbr.key)
                for dir_, nbr in self.neighbors[current]
                if nbr.key not in visited
            ]
            if not choices:
                stack.pop()
                continue

            # Carve to a random unvisited neighbor
            dir_, nxt = random.choice(choices)
            self.cells[current].walls[dir_] = False
            self.cells[nxt].walls[opposite[dir_]] = False

            visited.add(nxt)
            stack.append(nxt)

    def bfs(self, start: tuple[int, int]) -> tuple[dict, dict]:
        """
        Perform BFS from a start cell to compute distances and parent pointers.

        :param start: The starting cell key (ring, sector).
        :return: A tuple (distances, parents):
                 - distances: mapping each reachable cell key to its distance from start.
                 - parents: mapping each reachable cell key (except start) to its predecessor key.
        """
        dist = {start: 0}
        parent = {}
        queue = deque([start])

        while queue:
            u = queue.popleft()
            for direction, neighbor in self.neighbors[u]:
                v = neighbor.key
                if not self.cells[u].walls[direction] and v not in dist:
                    dist[v] = dist[u] + 1
                    parent[v] = u
                    queue.append(v)

        return dist, parent


def two_opt_path(path: list, dist_map: dict, max_loops: int = 100) -> list:
    """
    Improve a route by performing 2‐opt swaps based on complete‐graph distances.

    :param path: The initial list of node keys defining the route.
    :param dist_map: A dict mapping each node to its distance dict to every other node.
    :param max_loops: Maximum number of full improvement loops.
    :return: An improved route as a list of node keys.
    """
    best = list(path)
    n = len(best)
    INF = math.inf

    for _ in range(max_loops):
        improved = False
        for i in range(1, n - 2):
            for j in range(i + 1, n - 1):
                a, b = best[i - 1], best[i]
                c, d = best[j], best[j + 1]
                curr = dist_map[a].get(b, INF) + dist_map[c].get(d, INF)
                swap = dist_map[a].get(c, INF) + dist_map[b].get(d, INF)
                if swap + 1e-9 < curr:
                    best[i : j + 1] = reversed(best[i : j + 1])
                    improved = True
        if not improved:
            break

    return best


def find_best_route(maze: CircularMaze, entrances: list, survivors: list) -> tuple:
    """
    Compute the best rescue route starting and ending at any entrance, picking up all survivors.

    :param maze: The CircularMaze instance.
    :param entrances: List of entrance cell keys.
    :param survivors: List of survivor cell keys.
    :return: A tuple containing:
             - best_route: (start_cell, end_cell, pickup_order, total_distance)
             - parent_map: BFS parent pointers for each node used in path reconstruction.
    """
    nodes = entrances + survivors
    dist_map = {}
    parent_map = {}

    # Precompute BFS distances & parents for all nodes
    for node in nodes:
        d, p = maze.bfs(node)
        dist_map[node] = d
        parent_map[node] = p

    # Trivial case: no survivors
    if not survivors:
        e = entrances[0]
        return (e, e, [], 0), parent_map

    best_route = (None, None, [], math.inf)

    for start in entrances:
        remaining = set(survivors)
        order = []
        total = 0
        current = start

        # Greedy nearest‐neighbor to pickup
        while remaining:
            nxt = min(remaining, key=lambda s: dist_map[current].get(s, math.inf))
            total += dist_map[current].get(nxt, math.inf)
            order.append(nxt)
            remaining.remove(nxt)
            current = nxt

        # Return to closest entrance
        back_dist, best_exit = min(
            ((dist_map[current].get(e, math.inf), e) for e in entrances),
            key=lambda x: x[0],
        )
        total += back_dist

        full = [start] + order + [best_exit]
        improved = two_opt_path(full, dist_map)
        new_cost = sum(
            dist_map[u].get(v, math.inf) for u, v in zip(improved, improved[1:])
        )

        if new_cost < best_route[3]:
            s0, *mid, s1 = improved
            best_route = (s0, s1, mid, new_cost)

    return best_route, parent_map


# ----------------------------------------
#  Agent definitions
# ----------------------------------------

class Survivor(ap.Agent):
    """Agent representing a survivor to be rescued."""
    def setup(self):
        """
        Initialize survivor state.
        """
        self.rescued = False
        # self.position is set externally in the Model


class Rescuer(ap.Agent):
    """Agent that follows the precomputed rescue path."""

    def setup(self, path: list):
        """
        Initialize rescuer with its route.

        :param path: Ordered list of cell keys defining the rescue path.
        """
        self.path = path
        self.current_step = 0
        self.position = path[0]
        self.carry = []

    def step(self):
        """
        Move one step along the path and pick up survivors if present.
        """
        if self.current_step < len(self.path) - 1:
            self.current_step += 1
            self.position = self.path[self.current_step]
            # Pick up any survivors located here
            for s in self.model.survivors:
                if not s.rescued and s.position == self.position:
                    s.rescued = True
                    self.carry.append(s)


# ----------------------------------------
#  The AgentPy model
# ----------------------------------------

class RescueModel(ap.Model):
    """AgentPy model orchestrating the maze rescue scenario."""

    def setup(self):
        """
        Create maze, place survivors and entrances, compute route, and spawn agents.
        """
        # 1) Maze generation
        self.maze = CircularMaze(self.p.R, self.p.S)
        self.maze.carve_maze(self.p.seed)

        # 2) Randomly sample survivors and entrance‐sectors
        all_cells = [(r, s) for r in range(1, self.p.R) for s in range(self.p.S)]
        survivors_cells = random.sample(all_cells, self.p.nsurv)
        entrance_sectors = random.sample(range(self.p.S), self.p.nentr)
        entrances = [(self.p.R - 1, s) for s in entrance_sectors]

        # 3) Compute optimal route
        (start, end, pickup_order, total_steps), parent_map = find_best_route(
            self.maze, entrances, survivors_cells
        )
        full_route = [start] + pickup_order + [end]

        # Store for reporting & rendering
        self.start_cell = start
        self.end_cell = end
        self.order = pickup_order
        self.total = total_steps
        self.parent_map = parent_map
        self.full_route = full_route
        self.entrance_sectors = set(entrance_sectors)

        # 4) Create agents
        self.survivors = ap.AgentList(self, self.p.nsurv, Survivor)
        for agent, cell in zip(self.survivors, survivors_cells):
            agent.position = cell

        self.rescuer = ap.AgentList(self, 1, Rescuer, path=full_route)

    def step(self):
        """
        Advance the rescuer one step.
        """
        self.rescuer.step()

    def end(self):
        """
        Called at simulation end; results are stored in agents.
        """
        pass


# ----------------------------------------
#  SVG rendering
# ----------------------------------------

def render_svg(maze: CircularMaze, survivors: list, start: tuple, pickups: list, end: tuple, entrance_sectors: list, parent_map: dict) -> str:
    """
    Render the maze, walls, planned route, survivors, and entrances to an SVG string.

    :param maze: The maze to render.
    :param survivors: Positions of survivors.
    :param start: Starting cell key.
    :param pickups: Ordered list of pickup cell keys.
    :param end: Ending cell key.
    :param entrance_sectors: Set of sector indices that are valid entrances/exits.
    :param parent_map: BFS parent pointers for reconstructing paths.
    :return: SVG markup as a string.
    """
    R, S = maze.R, maze.S
    W = H = 800
    cx = cy = W / 2
    maxrad = W / 2 - 20
    dr = maxrad / R

    wall_w = max(min(dr / 3, 6), 1)
    route_w = max(wall_w * 0.65, 0.75)
    pt_r = max(wall_w, 2)

    svg = [f'''
<svg id="svg-maze" viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg">
<style>
    .wall  {{ stroke-width:{wall_w:.2f}; }}
    .route {{ stroke-width:{route_w:.2f}; }}
</style>
<g id="zoomgrp">
    <circle cx="{cx:.2f}" cy="{cy:.2f}" r="{dr:.2f}" fill="var(--wall)" />
''']

    def coord(r: int, s: int) -> tuple[float, float, float]:
        """
        Compute the center point and radius of a given cell.

        :param r: Ring index.
        :param s: Sector index.
        :return: (x, y, radius).
        """
        rm = dr * (r + 0.5)
        θ = 2 * math.pi * (s + 0.5) / S
        return cx + rm * math.cos(θ), cy + rm * math.sin(θ), rm

    # Draw walls
    for cell in maze.cells.values():
        r, s = cell.ring, cell.sector
        θ1 = 2 * math.pi * s / S
        θ2 = 2 * math.pi * (s + 1) / S
        inner_r, outer_r = dr * r, dr * (r + 1)

        # Radial walls
        if cell.walls['cw'] and r != 0:
            x1, y1 = cx + inner_r * math.cos(θ2), cy + inner_r * math.sin(θ2)
            x2, y2 = cx + outer_r * math.cos(θ2), cy + outer_r * math.sin(θ2)
            svg.append(f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" class="wall"/>')

        # Circular walls
        if cell.walls['out'] and (r != R - 1 or s not in entrance_sectors):
            x1, y1 = cx + outer_r * math.cos(θ1), cy + outer_r * math.sin(θ1)
            x2, y2 = cx + outer_r * math.cos(θ2), cy + outer_r * math.sin(θ2)
            svg.append(f'<path d="M{x1:.2f},{y1:.2f} A{outer_r:.2f},{outer_r:.2f} 0 0,1 {x2:.2f},{y2:.2f}" class="wall"/>')

    # Draw planned route
    route = [start] + pickups + [end]
    for src, dst in zip(route, route[1:]):
        parents = parent_map[src]
        path = [dst]
        while path[-1] != src:
            path.append(parents[path[-1]])
        path.reverse()

        # Move & arcs
        d = []
        x0, y0, _ = coord(*path[0])
        d.append(f'M{x0:.2f},{y0:.2f}')
        for (r0, s0), (r1, s1) in zip(path, path[1:]):
            x1, y1, rad1 = coord(r1, s1)
            if r0 == r1:
                ds = (s1 - s0) % S
                sweep = 1 if ds == 1 else 0
                d.append(f'A{rad1:.2f},{rad1:.2f} 0 0,{sweep} {x1:.2f},{y1:.2f}')
            else:
                d.append(f'L{x1:.2f},{y1:.2f}')

        svg.append(f'<path class="route" d="{" ".join(d)}"/>')

    # Survivors
    for cell in survivors:
        x, y, _ = coord(*cell)
        svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{pt_r:.2f}" class="survivor"/>')

    # Entrances/exits
    for cell in (start, end):
        x, y, _ = coord(*cell)
        svg.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{pt_r:.2f}" class="exit"/>')

    svg.append('</g></svg>')
    return ''.join(svg)


# ----------------------------------------
#  PyScript UI hook
# ----------------------------------------

def on_generate(event):
    """
    UI callback to generate a new maze scenario, run the model, and update the page.
    """
    # Read inputs
    R = max(2, int(document.getElementById("rings").value))
    S = max(3, int(document.getElementById("sectors").value))
    N = min((R-1)*S, max(0, int(document.getElementById("nsurv").value)))
    ne= min(S, max(1, int(document.getElementById("nentr").value)))
    seed = random.getrandbits(32)

    # Build & setup model
    params = {'R':R,'S':S,'nsurv':N,'nentr':ne,'seed':seed}
    model = RescueModel(params); model.setup()
    steps = len(model.full_route)-1

    # Run
    model.run(steps=steps)

    # Pull results
    maze = model.maze
    start, end = model.start_cell, model.end_cell
    total = model.total
    rescued = sum(a.rescued for a in model.survivors)
    entr_sectors = model.entrance_sectors

    # Compute smallest & average path length from start
    dists,_ = maze.bfs(start)
    nonzero = [d for d in dists.values() if d>0]
    avg_path = sum(nonzero)/len(nonzero) if nonzero else 0
    max_path = max(nonzero) if nonzero else 0


    # Compute wall‐density
    cells = list(maze.cells.values())
    wall_count = sum(sum(cell.walls.values()) for cell in cells)
    density = wall_count / (len(cells)*4)

    # Compute best rotation‐symmetry %
    best = max(
        sum(1 for (r,s),c in maze.cells.items() if c.walls == maze.cells[(r,(s+k)%S)].walls) for k in range(1, S)
    )
    sym_pct = best/len(cells)*100

    # Update metrics
    document.getElementById("metrics").innerHTML = f"""
<h2>Run Summary</h2>
<ul>
  <li><strong>Average BFS distance:</strong> {avg_path:.2f}</li>
  <li><strong>Maximum BFS distance:</strong> {max_path}</li>
  <li><strong>Wall density:</strong> {density:.2%}</li>
  <li><strong>Rotational symmetry:</strong> {sym_pct:.2f}%</li>
  <li><strong>Rescue route steps:</strong> {total}</li>
  <li><strong>Actually rescued:</strong> {rescued}</li>
</ul>"""

    # Render SVG
    svg = render_svg(
        maze,
        [s.position for s in model.survivors],
        start,
        model.order,
        end,
        entr_sectors,
        model.parent_map
    )
    document.getElementById("svg-container").innerHTML = svg

# Hook up the button
generate_proxy = create_proxy(on_generate)
generateButton = document.getElementById("generate")
generateButton.addEventListener("click", generate_proxy)
generateButton.disabled = False
