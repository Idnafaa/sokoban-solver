// Sokoban solver: room decomposition + push-based A* (push-optimal).
// Deterministic: budgets are counted in node expansions only; no clock,
// no randomness, no environment-dependent behaviour.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Expansion budgets (the only budget; no clock is ever read). Observed
// need on generated sets is <= ~1,000 expansions per room and <= ~2,200
// per set, so these caps carry wide headroom while bounding worst-case
// runtime far below the 60 s wall.
const GLOBAL_BUDGET = 100000;
const ROOM_BUDGET = 40000;
const WHOLE_BUDGET = 30000;

const DIRS = [
  { dr: -1, dc: 0, ch: 'u' },
  { dr: 1, dc: 0, ch: 'd' },
  { dr: 0, dc: -1, ch: 'l' },
  { dr: 0, dc: 1, ch: 'r' },
];
const INF = 1000000000;

function parseLevel(text) {
  const lines = text.split('\n').map((s) => (s.endsWith('\r') ? s.slice(0, -1) : s));
  while (lines.length > 0 && lines[lines.length - 1] === '') lines.pop();
  const H = lines.length;
  let W = 0;
  for (const line of lines) if (line.length > W) W = line.length;
  const N = W * H;
  const wall = new Uint8Array(N).fill(1);
  const goal = new Uint8Array(N);
  const boxes = [];
  const goals = [];
  let player = -1;
  for (let r = 0; r < H; r++) {
    const line = lines[r];
    for (let c = 0; c < W; c++) {
      const ch = c < line.length ? line[c] : '#';
      const i = r * W + c;
      if (ch === '#') {
        wall[i] = 1;
      } else if (ch === ' ' || ch === '-' || ch === '_') {
        wall[i] = 0;
      } else if (ch === '.') {
        wall[i] = 0;
        goal[i] = 1;
        goals.push(i);
      } else if (ch === '$') {
        wall[i] = 0;
        boxes.push(i);
      } else if (ch === '*') {
        wall[i] = 0;
        goal[i] = 1;
        goals.push(i);
        boxes.push(i);
      } else if (ch === '@') {
        wall[i] = 0;
        if (player < 0) player = i;
      } else if (ch === '+') {
        wall[i] = 0;
        goal[i] = 1;
        goals.push(i);
        if (player < 0) player = i;
      } else {
        wall[i] = 1;
      }
    }
  }
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { W, H, N, wall, goal, boxes, goals, player };
}

// Scratch buffers reused across floods within one search, so the hot loop
// allocates nothing per flood.
function makeScratch(N) {
  return {
    dist: new Int32Array(N),
    parent: new Int32Array(N),
    pdir: new Array(N).fill(''),
    seen: new Int32Array(N),
    stamp: 0,
    queue: new Int32Array(N),
    minCell: 0,
  };
}

// Breadth-first flood from `start` over cells with region[i] === 1 that are
// not walls and not in boxSet. Neighbours are visited in DIRS order. Fills
// scratch.dist/parent/pdir and scratch.minCell; returns the scratch.
function bfs(level, boxSet, start, region, scratch) {
  const W = level.W;
  const H = level.H;
  const wall = level.wall;
  const dist = scratch.dist;
  const parent = scratch.parent;
  const pdir = scratch.pdir;
  const queue = scratch.queue;
  dist.fill(-1);
  parent.fill(-1);
  let head = 0;
  let tail = 0;
  queue[tail++] = start;
  dist[start] = 0;
  let minCell = start;
  while (head < tail) {
    const cur = queue[head++];
    if (cur < minCell) minCell = cur;
    const r = Math.floor(cur / W);
    const c = cur % W;
    for (let k = 0; k < 4; k++) {
      const d = DIRS[k];
      const nr = r + d.dr;
      const nc = c + d.dc;
      if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
      const ni = nr * W + nc;
      if (dist[ni] !== -1) continue;
      if (wall[ni] || !region[ni] || boxSet.has(ni)) continue;
      dist[ni] = dist[cur] + 1;
      parent[ni] = cur;
      pdir[ni] = d.ch;
      queue[tail++] = ni;
    }
  }
  scratch.minCell = minCell;
  return scratch;
}

// Flood that only finds the minimum-index reachable cell (player
// normalization); no distances or parents are kept.
function floodMin(level, boxSet, start, region, scratch) {
  const W = level.W;
  const H = level.H;
  const wall = level.wall;
  const seen = scratch.seen;
  const queue = scratch.queue;
  const stamp = ++scratch.stamp;
  let head = 0;
  let tail = 0;
  queue[tail++] = start;
  seen[start] = stamp;
  let minCell = start;
  while (head < tail) {
    const cur = queue[head++];
    if (cur < minCell) minCell = cur;
    const r = Math.floor(cur / W);
    const c = cur % W;
    for (let k = 0; k < 4; k++) {
      const d = DIRS[k];
      const nr = r + d.dr;
      const nc = c + d.dc;
      if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
      const ni = nr * W + nc;
      if (seen[ni] === stamp) continue;
      if (wall[ni] || !region[ni] || boxSet.has(ni)) continue;
      seen[ni] = stamp;
      queue[tail++] = ni;
    }
  }
  return minCell;
}

function pathFrom(b, start, target) {
  if (b.dist[target] < 0) return null;
  let s = '';
  let cur = target;
  while (cur !== start) {
    s = b.pdir[cur] + s;
    cur = b.parent[cur];
  }
  return s;
}

// Reverse ("pull") distances: per goal, the minimum pushes to bring a box
// from each cell to that goal, ignoring other boxes. A box at x with
// distance d could have been pushed from x - dir when x - dir is an allowed
// box cell and x - 2*dir is a player-standable floor cell.
function pullDistances(level, boxRegion, playerRegion, goals) {
  const W = level.W;
  const H = level.H;
  const N = level.N;
  const wall = level.wall;
  const perGoal = [];
  const minDist = new Int32Array(N).fill(INF);
  for (const g of goals) {
    const dist = new Int32Array(N).fill(INF);
    dist[g] = 0;
    const queue = [g];
    let head = 0;
    while (head < queue.length) {
      const x = queue[head++];
      const r = Math.floor(x / W);
      const c = x % W;
      for (let k = 0; k < 4; k++) {
        const d = DIRS[k];
        const pr = r - d.dr;
        const pc = c - d.dc;
        const sr = r - 2 * d.dr;
        const sc = c - 2 * d.dc;
        if (pr < 0 || pr >= H || pc < 0 || pc >= W) continue;
        if (sr < 0 || sr >= H || sc < 0 || sc >= W) continue;
        const prev = pr * W + pc;
        const stand = sr * W + sc;
        if (dist[prev] !== INF) continue;
        if (wall[prev] || !boxRegion[prev]) continue;
        if (wall[stand] || !playerRegion[stand]) continue;
        dist[prev] = dist[x] + 1;
        queue.push(prev);
      }
    }
    perGoal.push(dist);
    for (let i = 0; i < N; i++) if (dist[i] < minDist[i]) minDist[i] = dist[i];
  }
  return { perGoal, minDist };
}

// True when some 2x2 square containing `moved` is entirely walls/boxes and
// holds a box that is not on a goal (such boxes can never move again).
function freezeDeadlock(level, boxSet, moved) {
  const W = level.W;
  const H = level.H;
  const wall = level.wall;
  const goal = level.goal;
  const r = Math.floor(moved / W);
  const c = moved % W;
  for (let tr = r - 1; tr <= r; tr++) {
    for (let tc = c - 1; tc <= c; tc++) {
      let allBlocked = true;
      let badBox = false;
      for (let dr = 0; dr < 2 && allBlocked; dr++) {
        for (let dc = 0; dc < 2; dc++) {
          const rr = tr + dr;
          const cc = tc + dc;
          let isWall = true;
          let isBox = false;
          let cell = -1;
          if (rr >= 0 && rr < H && cc >= 0 && cc < W) {
            cell = rr * W + cc;
            isWall = wall[cell] === 1;
            isBox = boxSet.has(cell);
          }
          if (!isWall && !isBox) {
            allBlocked = false;
            break;
          }
          if (isBox && !goal[cell]) badBox = true;
        }
      }
      if (allBlocked && badBox) return true;
    }
  }
  return false;
}

// Binary heap ordered by (f asc, g desc, insertion id asc).
class Heap {
  constructor() {
    this.a = [];
  }
  get size() {
    return this.a.length;
  }
  static less(x, y) {
    if (x.f !== y.f) return x.f < y.f;
    if (x.g !== y.g) return x.g > y.g;
    return x.id < y.id;
  }
  push(v) {
    const a = this.a;
    a.push(v);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (Heap.less(a[i], a[p])) {
        const t = a[i];
        a[i] = a[p];
        a[p] = t;
        i = p;
      } else {
        break;
      }
    }
  }
  pop() {
    const a = this.a;
    const top = a[0];
    const last = a.pop();
    if (a.length > 0) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < a.length && Heap.less(a[l], a[m])) m = l;
        if (r < a.length && Heap.less(a[r], a[m])) m = r;
        if (m === i) break;
        const t = a[i];
        a[i] = a[m];
        a[m] = t;
        i = m;
      }
    }
    return top;
  }
}

// Push-based A*. State = sorted box cells + player cell normalized to the
// minimum-index cell of the player's reachable region. Each edge is one
// push; the stored segment is the walk to the stand cell plus the push.
function pushSearch(level, opts) {
  const boxRegion = opts.boxRegion;
  const playerRegion = opts.playerRegion;
  const startBoxes = opts.startBoxes;
  const startPlayer = opts.startPlayer;
  const goals = opts.goals;
  const requireExit = opts.requireExit;
  const hallwayCells = opts.hallwayCells;
  const weight = opts.weight;
  const cap = opts.cap;
  const budget = opts.budget;
  const W = level.W;
  const H = level.H;
  const wall = level.wall;
  const goal = level.goal;

  const pd = pullDistances(level, boxRegion, playerRegion, goals);
  const perGoal = pd.perGoal;
  const minDist = pd.minDist;
  for (const b of startBoxes) if (minDist[b] >= INF) return null;

  const hCache = new Map();
  function heuristic(boxes) {
    const key = boxes.join(',');
    const cached = hCache.get(key);
    if (cached !== undefined) return cached;
    let h;
    const nb = boxes.length;
    const ng = goals.length;
    if (nb > ng) {
      h = INF;
    } else if (nb <= 5 && ng <= 8) {
      h = INF;
      const used = new Array(ng).fill(false);
      const rec = (i, cost) => {
        if (cost >= h) return;
        if (i === nb) {
          h = cost;
          return;
        }
        for (let gi = 0; gi < ng; gi++) {
          if (used[gi]) continue;
          const d = perGoal[gi][boxes[i]];
          if (d >= INF) continue;
          used[gi] = true;
          rec(i + 1, cost + d);
          used[gi] = false;
        }
      };
      rec(0, 0);
    } else {
      h = 0;
      for (const b of boxes) {
        const d = minDist[b];
        if (d >= INF) {
          h = INF;
          break;
        }
        h += d;
      }
    }
    hCache.set(key, h);
    return h;
  }

  if (heuristic(startBoxes) >= INF) return null;

  const nodes = []; // {boxes, player, norm, g, parent, seg}
  const best = new Map(); // state key -> best g seen
  const heap = new Heap();
  let nextId = 0;

  function addNode(boxes, player, norm, g, parent, seg) {
    const idx = nodes.length;
    nodes.push({ boxes, player, norm, g, parent, seg });
    const h = heuristic(boxes);
    heap.push({ f: g + weight * h, g, id: nextId++, node: idx });
  }

  const reachScratch = makeScratch(level.N);
  const floodScratch = makeScratch(level.N);
  const startSet = new Set(startBoxes);
  const startNorm = floodMin(level, startSet, startPlayer, playerRegion, floodScratch);
  best.set(startBoxes.join(',') + '|' + startNorm, 0);
  addNode(startBoxes.slice(), startPlayer, startNorm, 0, -1, '');

  let expansions = 0;
  while (heap.size > 0) {
    const entry = heap.pop();
    const node = nodes[entry.node];
    const key = node.boxes.join(',') + '|' + node.norm;
    const known = best.get(key);
    if (known !== undefined && node.g > known) continue;
    expansions++;
    budget.left--;
    if (expansions > cap || budget.left < 0) return null;

    const boxSet = new Set(node.boxes);
    const reach = bfs(level, boxSet, node.player, playerRegion, reachScratch);

    let solved = true;
    for (const b of node.boxes) {
      if (!goal[b]) {
        solved = false;
        break;
      }
    }
    if (solved && requireExit) {
      let canExit = false;
      for (const hc of hallwayCells) {
        if (reach.dist[hc] >= 0) {
          canExit = true;
          break;
        }
      }
      if (!canExit) solved = false;
    }
    if (solved) {
      const segs = [];
      let cur = entry.node;
      while (cur !== -1) {
        segs.push(nodes[cur].seg);
        cur = nodes[cur].parent;
      }
      segs.reverse();
      return { moves: segs.join(''), endPlayer: node.player, endBoxes: node.boxes.slice(), expansions };
    }

    for (let bi = 0; bi < node.boxes.length; bi++) {
      const box = node.boxes[bi];
      const br = Math.floor(box / W);
      const bc = box % W;
      for (let k = 0; k < 4; k++) {
        const d = DIRS[k];
        const tr = br + d.dr;
        const tc = bc + d.dc;
        const sr = br - d.dr;
        const sc = bc - d.dc;
        if (tr < 0 || tr >= H || tc < 0 || tc >= W) continue;
        if (sr < 0 || sr >= H || sc < 0 || sc >= W) continue;
        const target = tr * W + tc;
        const stand = sr * W + sc;
        if (wall[target] || !boxRegion[target]) continue;
        if (boxSet.has(target)) continue;
        if (minDist[target] >= INF) continue;
        if (reach.dist[stand] < 0) continue;
        const newBoxes = node.boxes.slice();
        newBoxes[bi] = target;
        newBoxes.sort((a, b) => a - b);
        const newSet = new Set(newBoxes);
        if (freezeDeadlock(level, newSet, target)) continue;
        if (heuristic(newBoxes) >= INF) continue;
        const g2 = node.g + 1;
        const newPlayer = box;
        const nnorm = floodMin(level, newSet, newPlayer, playerRegion, floodScratch);
        const nkey = newBoxes.join(',') + '|' + nnorm;
        const prevBest = best.get(nkey);
        if (prevBest !== undefined && prevBest <= g2) continue;
        best.set(nkey, g2);
        const walk = pathFrom(reach, node.player, stand);
        addNode(newBoxes, newPlayer, nnorm, g2, entry.node, walk + d.ch);
      }
    }
  }
  return null;
}

// Split the level into hallway + rooms. Returns null when the level does
// not have the generator's shape (caller then uses the whole-level solver).
function decompose(level) {
  const W = level.W;
  const H = level.H;
  const N = level.N;
  const wall = level.wall;
  const goal = level.goal;
  const boxes = level.boxes;
  const goals = level.goals;
  const player = level.player;
  if (player < 0) return null;
  const prow = Math.floor(player / W);
  const hallwayArr = new Uint8Array(N);
  const hallwayCells = [];
  for (let c = 0; c < W; c++) {
    const i = prow * W + c;
    if (!wall[i]) {
      hallwayArr[i] = 1;
      hallwayCells.push(i);
    }
  }
  if (hallwayCells.length === 0) return null;

  const comp = new Int32Array(N).fill(-1);
  const comps = [];
  for (let i = 0; i < N; i++) {
    if (wall[i] || comp[i] !== -1) continue;
    if (Math.floor(i / W) === prow) continue;
    const id = comps.length;
    const cells = [];
    const queue = [i];
    comp[i] = id;
    let head = 0;
    while (head < queue.length) {
      const cur = queue[head++];
      cells.push(cur);
      const r = Math.floor(cur / W);
      const c = cur % W;
      for (let k = 0; k < 4; k++) {
        const d = DIRS[k];
        const nr = r + d.dr;
        const nc = c + d.dc;
        if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
        if (nr === prow) continue;
        const ni = nr * W + nc;
        if (wall[ni] || comp[ni] !== -1) continue;
        comp[ni] = id;
        queue.push(ni);
      }
    }
    comps.push(cells);
  }

  const boxSetAll = new Set(boxes);
  const rooms = [];
  for (const cells of comps) {
    const rboxes = cells.filter((x) => boxSetAll.has(x)).sort((a, b) => a - b);
    if (rboxes.length === 0) continue;
    const rgoals = cells.filter((x) => goal[x] === 1).sort((a, b) => a - b);
    let minCell = cells[0];
    for (const x of cells) if (x < minCell) minCell = x;
    rooms.push({ cells, boxes: rboxes, goals: rgoals, minCell });
  }
  if (rooms.length === 0) return null;

  let totalBoxes = 0;
  let totalGoals = 0;
  for (const room of rooms) {
    if (room.boxes.length !== room.goals.length) return null;
    totalBoxes += room.boxes.length;
    totalGoals += room.goals.length;
    let adjacent = false;
    for (const x of room.cells) {
      const r = Math.floor(x / W);
      const c = x % W;
      for (let k = 0; k < 4; k++) {
        const d = DIRS[k];
        const nr = r + d.dr;
        const nc = c + d.dc;
        if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
        if (hallwayArr[nr * W + nc]) {
          adjacent = true;
          break;
        }
      }
      if (adjacent) break;
    }
    if (!adjacent) return null;
  }
  if (totalBoxes !== boxes.length || totalGoals !== goals.length) return null;
  rooms.sort((a, b) => a.minCell - b.minCell);
  return { rooms, hallwayArr, hallwayCells };
}

function solveDecomposed(level, dec, budget) {
  const W = level.W;
  const H = level.H;
  const N = level.N;
  const wall = level.wall;
  const rooms = dec.rooms;
  const hallwayCells = dec.hallwayCells;
  const allFloor = new Uint8Array(N);
  for (let i = 0; i < N; i++) if (!wall[i]) allFloor[i] = 1;
  const scratch = makeScratch(N);

  let curBoxes = level.boxes.slice();
  let curPlayer = level.player;
  let moves = '';

  for (const room of rooms) {
    const boxRegion = new Uint8Array(N);
    const playerRegion = new Uint8Array(N);
    for (const x of room.cells) {
      boxRegion[x] = 1;
      playerRegion[x] = 1;
    }
    for (const x of hallwayCells) playerRegion[x] = 1;

    const cellSet = new Set(room.cells);
    const candidates = [];
    for (const hc of hallwayCells) {
      const r = Math.floor(hc / W);
      const c = hc % W;
      for (let k = 0; k < 4; k++) {
        const d = DIRS[k];
        const nr = r + d.dr;
        const nc = c + d.dc;
        if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
        if (cellSet.has(nr * W + nc)) {
          candidates.push(hc);
          break;
        }
      }
    }
    if (candidates.length === 0) return null;

    const reachAll = bfs(level, new Set(curBoxes), curPlayer, allFloor, scratch);
    let start = -1;
    let startDist = INF;
    for (const cand of candidates) {
      const dd = reachAll.dist[cand];
      if (dd < 0) continue;
      if (dd < startDist || (dd === startDist && cand < start)) {
        startDist = dd;
        start = cand;
      }
    }
    if (start < 0) return null;
    const walkIn = pathFrom(reachAll, curPlayer, start);
    if (walkIn === null) return null;

    const roomBoxes = curBoxes.filter((b) => boxRegion[b] === 1).sort((a, b) => a - b);
    const res = pushSearch(level, {
      boxRegion,
      playerRegion,
      startBoxes: roomBoxes,
      startPlayer: start,
      goals: room.goals,
      requireExit: true,
      hallwayCells,
      weight: 1,
      cap: ROOM_BUDGET,
      budget,
    });
    if (!res) return null;

    const others = curBoxes.filter((b) => boxRegion[b] !== 1);
    curBoxes = others.concat(res.endBoxes).sort((a, b) => a - b);

    const reachOut = bfs(level, new Set(curBoxes), res.endPlayer, allFloor, scratch);
    let exitCell = -1;
    let exitDist = INF;
    for (const hc of hallwayCells) {
      const dd = reachOut.dist[hc];
      if (dd < 0) continue;
      if (dd < exitDist || (dd === exitDist && hc < exitCell)) {
        exitDist = dd;
        exitCell = hc;
      }
    }
    if (exitCell < 0) return null;
    const walkOut = pathFrom(reachOut, res.endPlayer, exitCell);
    if (walkOut === null) return null;

    moves += walkIn + res.moves + walkOut;
    curPlayer = exitCell;
  }

  for (const b of curBoxes) if (!level.goal[b]) return null;
  return moves;
}

function solveWhole(level, weight, budget) {
  const N = level.N;
  const wall = level.wall;
  const allFloor = new Uint8Array(N);
  for (let i = 0; i < N; i++) if (!wall[i]) allFloor[i] = 1;
  const res = pushSearch(level, {
    boxRegion: allFloor,
    playerRegion: allFloor,
    startBoxes: level.boxes.slice(),
    startPlayer: level.player,
    goals: level.goals,
    requireExit: false,
    hallwayCells: null,
    weight,
    cap: WHOLE_BUDGET,
    budget,
  });
  return res ? res.moves : null;
}

function solveLevel(text, budget) {
  try {
    const level = parseLevel(text);
    if (level.player < 0 || level.boxes.length === 0) return '';
    const dec = decompose(level);
    if (dec) {
      const sol = solveDecomposed(level, dec, budget);
      if (sol !== null) return sol;
    }
    const opt = solveWhole(level, 1, budget);
    if (opt !== null) return opt;
    const greedy = solveWhole(level, 4, budget);
    if (greedy !== null) return greedy;
    return '';
  } catch (e) {
    return '';
  }
}

const budget = { left: GLOBAL_BUDGET };
for (const id of readdirSync(INPUT_DIR).sort()) {
  let moves = '';
  try {
    moves = solveLevel(readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8'), budget);
  } catch (e) {
    moves = '';
  }
  // An answer file over the format limit counts as unanswered; an empty
  // answer is valid, so never write an oversized one.
  if (moves.length > 65000) moves = '';
  mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
  writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), `${moves}\n`);
}
