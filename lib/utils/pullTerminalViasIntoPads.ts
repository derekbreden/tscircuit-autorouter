import type { ConnectivityMap } from "circuit-json-to-connectivity-map"
import type { Obstacle } from "lib/types"
import type { HighDensityIntraNodeRoute } from "lib/types/high-density-types"
import { isObstacleConnectedToRoute } from "lib/solvers/TraceWidthSolver/isObstacleConnectedToRoute"
import { mapLayerNameToZ } from "./mapLayerNameToZ"

/**
 * viaInPad post-routing rewrite.
 *
 * The routing mesh never births a via inside a pad — the pad occupies its own layer, so the
 * node there is not via-capable — which is why a route that leaves a top pad for the inner
 * copper lands its transition via a short hop *into* the trace, not on the pad. This pass moves
 * that first/last transition via back onto its terminal SMD pad (via-in-pad) wherever doing so is
 * provably clear of foreign copper. Genuine mid-channel vias (not adjacent to their route's
 * terminal pad) are left untouched.
 *
 * Soundness: the barrel of a moved via lands in its own terminal pad (same net, fine) and, on
 * every other layer, in whatever the mesh already kept clear at the pad — but we re-check against
 * every *foreign* pad / plated-hole / trace / via the router knows about and only move the via
 * when the barrel column AND the replacement same-net segment clear them all by `clearance`.
 * Copper pours are not the router's concern here: a top↔bottom via is antipadded by every foreign
 * pour downstream (copper-pour-solver), and an inner-layer segment is carved by the pour like any
 * inner signal — so pours can neither short the moved via nor the replacement segment. The board
 * DRC (clearance.ts) is the independent referee on the final geometry.
 */
export interface PullTerminalViasIntoPadsOptions {
  obstacles?: ReadonlyArray<Obstacle>
  connMap?: ConnectivityMap
  layerCount: number
  /** Via pad diameter (barrel copper). */
  viaDiameter: number
  /** Required edge-to-edge gap from the moved copper to any foreign copper. */
  clearance: number
  /**
   * Only pull a via whose XY sits within this distance of its terminal pad. Bounds the length of
   * the replacement segment (and so the blast radius of the rewrite); genuine mid-channel vias are
   * farther from the route's terminal than this.
   */
  maxPullDistance?: number
}

const DEFAULT_MAX_PULL_DISTANCE = 3

type Pt = { x: number; y: number }
type Box = { minX: number; maxX: number; minY: number; maxY: number }
type ForeignObstacle = { box: Box; zSet: Set<number> }
type ForeignSeg = { x1: number; y1: number; x2: number; y2: number; z: number; r: number }
type ForeignVia = { x: number; y: number; r: number }

const EPS = 1e-9

/** Axis-aligned bounding box of a (possibly rotated) rect obstacle — conservative (⊇ the rect). */
const obstacleAabb = (o: Obstacle): Box => {
  const theta = ((o.ccwRotationDegrees ?? 0) * Math.PI) / 180
  const c = Math.abs(Math.cos(theta))
  const s = Math.abs(Math.sin(theta))
  const ax = (o.width / 2) * c + (o.height / 2) * s
  const ay = (o.width / 2) * s + (o.height / 2) * c
  return {
    minX: o.center.x - ax,
    maxX: o.center.x + ax,
    minY: o.center.y - ay,
    maxY: o.center.y + ay,
  }
}

const obstacleZSet = (o: Obstacle, layerCount: number): Set<number> => {
  if (o.zLayers && o.zLayers.length) return new Set(o.zLayers)
  if (o.layers && o.layers.length) {
    return new Set(o.layers.map((l) => mapLayerNameToZ(l, layerCount)))
  }
  return new Set()
}

const isSingleLayerObstacle = (o: Obstacle): boolean => {
  if (o.zLayers && o.zLayers.length) return o.zLayers.length === 1
  return (o.layers?.length ?? 0) === 1
}

const pointToAabbDistance = (p: Pt, b: Box): number => {
  const dx = Math.max(b.minX - p.x, 0, p.x - b.maxX)
  const dy = Math.max(b.minY - p.y, 0, p.y - b.maxY)
  return Math.hypot(dx, dy)
}

const pointInAabb = (p: Pt, b: Box): boolean =>
  p.x >= b.minX - EPS &&
  p.x <= b.maxX + EPS &&
  p.y >= b.minY - EPS &&
  p.y <= b.maxY + EPS

/** Min distance between segments p1p2 and p3p4 (handles zero-length = point). */
const segSeg = (p1: Pt, p2: Pt, p3: Pt, p4: Pt): number => {
  const d1x = p2.x - p1.x
  const d1y = p2.y - p1.y
  const d2x = p4.x - p3.x
  const d2y = p4.y - p3.y
  const rx = p1.x - p3.x
  const ry = p1.y - p3.y
  const a = d1x * d1x + d1y * d1y
  const e = d2x * d2x + d2y * d2y
  const f = d2x * rx + d2y * ry
  const cl = (v: number) => Math.max(0, Math.min(1, v))
  let s: number
  let t: number
  if (a <= EPS && e <= EPS) return Math.hypot(rx, ry)
  if (a <= EPS) {
    s = 0
    t = cl(f / e)
  } else {
    const c = d1x * rx + d1y * ry
    if (e <= EPS) {
      t = 0
      s = cl(-c / a)
    } else {
      const b = d1x * d2x + d1y * d2y
      const den = a * e - b * b
      s = den > EPS ? cl((b * f - c * e) / den) : 0
      t = (b * s + f) / e
      if (t < 0) {
        t = 0
        s = cl(-c / a)
      } else if (t > 1) {
        t = 1
        s = cl((b - c) / a)
      }
    }
  }
  return Math.hypot(
    p1.x + d1x * s - (p3.x + d2x * t),
    p1.y + d1y * s - (p3.y + d2y * t),
  )
}

const pointToSeg = (p: Pt, a: Pt, b: Pt): number => segSeg(p, p, a, b)

/** Min distance from a segment to an axis-aligned box (0 if it enters the box). */
const segToAabb = (a: Pt, b: Pt, box: Box): number => {
  if (pointInAabb(a, box) || pointInAabb(b, box)) return 0
  const c1 = { x: box.minX, y: box.minY }
  const c2 = { x: box.maxX, y: box.minY }
  const c3 = { x: box.maxX, y: box.maxY }
  const c4 = { x: box.minX, y: box.maxY }
  return Math.min(
    segSeg(a, b, c1, c2),
    segSeg(a, b, c2, c3),
    segSeg(a, b, c3, c4),
    segSeg(a, b, c4, c1),
  )
}

type RouteIds = { connectionName: string; rootConnectionName?: string }

const routesSameNet = (
  a: RouteIds,
  b: RouteIds,
  connMap?: ConnectivityMap,
): boolean => {
  if (a.connectionName === b.connectionName) return true
  const bIds = [b.connectionName, b.rootConnectionName].filter(
    (id): id is string => Boolean(id),
  )
  return bIds.some(
    (id) =>
      id === a.connectionName ||
      id === a.rootConnectionName ||
      (connMap?.areIdsConnected(a.connectionName, id) ?? false) ||
      (a.rootConnectionName !== undefined &&
        (connMap?.areIdsConnected(a.rootConnectionName, id) ?? false)),
  )
}

/** Foreign (different-net) copper the moved via / replacement segment must clear. */
const collectForeign = (
  route: HighDensityIntraNodeRoute,
  others: HighDensityIntraNodeRoute[],
  obstacles: ReadonlyArray<Obstacle>,
  layerCount: number,
  connMap?: ConnectivityMap,
): { obstacles: ForeignObstacle[]; segs: ForeignSeg[]; vias: ForeignVia[] } => {
  const foreignObstacles: ForeignObstacle[] = []
  for (const o of obstacles) {
    if (isObstacleConnectedToRoute(o, route, connMap)) continue
    foreignObstacles.push({ box: obstacleAabb(o), zSet: obstacleZSet(o, layerCount) })
  }

  const segs: ForeignSeg[] = []
  const vias: ForeignVia[] = []
  for (const other of others) {
    if (routesSameNet(route, other, connMap)) continue
    const r = other.route
    for (let i = 0; i + 1 < r.length; i++) {
      const p1 = r[i]
      const p2 = r[i + 1]
      if (p1.z !== p2.z) continue
      if (p1.x === p2.x && p1.y === p2.y) continue
      segs.push({
        x1: p1.x,
        y1: p1.y,
        x2: p2.x,
        y2: p2.y,
        z: p1.z,
        r: (p1.traceThickness ?? other.traceThickness) / 2,
      })
    }
    for (const v of other.vias) {
      vias.push({ x: v.x, y: v.y, r: other.viaDiameter / 2 })
    }
  }
  return { obstacles: foreignObstacles, segs, vias }
}

/**
 * The via barrel (all layers, radius `viaR`) at `at`, and the replacement segment `at`→`toward`
 * on layer `segZ` (radius `segR`), must clear every foreign feature by `clearance`.
 */
type Foreign = {
  obstacles: ForeignObstacle[]
  segs: ForeignSeg[]
  vias: ForeignVia[]
}

/** A wire segment a→b on layer `z` (radius `segR`) clears all foreign copper by `clearance`. */
const segClearOfForeign = (
  a: Pt,
  b: Pt,
  z: number,
  segR: number,
  clearance: number,
  foreign: Foreign,
): boolean => {
  for (const o of foreign.obstacles) {
    if (!o.zSet.has(z)) continue
    if (segToAabb(a, b, o.box) < segR + clearance - EPS) return false
  }
  for (const s of foreign.segs) {
    if (s.z !== z) continue
    if (
      segSeg(a, b, { x: s.x1, y: s.y1 }, { x: s.x2, y: s.y2 }) <
      segR + s.r + clearance - EPS
    )
      return false
  }
  for (const v of foreign.vias) {
    if (pointToSeg({ x: v.x, y: v.y }, a, b) < segR + v.r + clearance - EPS)
      return false
  }
  return true
}

const isPullClear = (
  at: Pt,
  toward: Pt,
  segZ: number,
  viaR: number,
  segR: number,
  clearance: number,
  foreign: Foreign,
): boolean => {
  // Moved via — a full-column barrel, so it must clear foreign copper on every layer.
  for (const o of foreign.obstacles) {
    if (pointToAabbDistance(at, o.box) < viaR + clearance - EPS) return false
  }
  for (const s of foreign.segs) {
    if (
      segSeg(at, at, { x: s.x1, y: s.y1 }, { x: s.x2, y: s.y2 }) <
      viaR + s.r + clearance - EPS
    )
      return false
  }
  for (const v of foreign.vias) {
    if (Math.hypot(at.x - v.x, at.y - v.y) < viaR + v.r + clearance - EPS)
      return false
  }
  // Replacement segment — copper only on segZ.
  return segClearOfForeign(at, toward, segZ, segR, clearance, foreign)
}

const findTerminalPad = (
  route: HighDensityIntraNodeRoute,
  terminal: Pt,
  obstacles: ReadonlyArray<Obstacle>,
  connMap?: ConnectivityMap,
): Obstacle | undefined =>
  obstacles.find(
    (o) =>
      isSingleLayerObstacle(o) &&
      isObstacleConnectedToRoute(o, route, connMap) &&
      pointInAabb(terminal, obstacleAabb(o)),
  )

const removeViaAt = (vias: Array<Pt>, at: Pt): void => {
  const idx = vias.findIndex(
    (v) => Math.abs(v.x - at.x) < 1e-3 && Math.abs(v.y - at.y) < 1e-3,
  )
  if (idx !== -1) vias.splice(idx, 1)
}

export const pullTerminalViasIntoPads = (
  hdRoutes: ReadonlyArray<HighDensityIntraNodeRoute>,
  opts: PullTerminalViasIntoPadsOptions,
): HighDensityIntraNodeRoute[] => {
  const {
    obstacles = [],
    connMap,
    layerCount,
    viaDiameter,
    clearance,
    maxPullDistance = DEFAULT_MAX_PULL_DISTANCE,
  } = opts
  const viaR = viaDiameter / 2

  const out: HighDensityIntraNodeRoute[] = hdRoutes.map((r) => ({
    ...r,
    route: r.route.map((p) => ({ ...p })),
    vias: r.vias.map((v) => ({ ...v })),
  }))

  for (let i = 0; i < out.length; i++) {
    const route = out[i]
    if (route.route.length < 3) continue
    const others = out.filter((_, k) => k !== i)
    const foreign = collectForeign(route, others, obstacles, layerCount, connMap)
    const segR = route.traceThickness / 2

    const countTransitions = (pts: HighDensityIntraNodeRoute["route"]) => {
      let n = 0
      for (let k = 1; k < pts.length; k++) if (pts[k].z !== pts[k - 1].z) n++
      return n
    }

    // Once the via lands on the pad, the wire leaving it may still detour out to the old via
    // location (that dip existed only to reach a via-capable spot off the pad). Pull that run taut:
    // starting from the pad's via-exit point (`anchor`), drop leading same-layer vertices whose
    // straight shortcut from the anchor clears foreign copper — bounded to vertices within
    // maxPullDistance of the pad, so it's a local cleanup of the detour, not a global re-route.
    // `dir` is +1 to walk forward from a start-pad anchor, -1 to walk back from an end-pad anchor.
    const tautenRunFromPad = (
      ptsIn: HighDensityIntraNodeRoute["route"],
      anchor: HighDensityIntraNodeRoute["route"][number],
      dir: 1 | -1,
    ): HighDensityIntraNodeRoute["route"] => {
      const z = anchor.z
      const result = ptsIn.slice()
      while (true) {
        const aIdx = result.indexOf(anchor)
        if (aIdx === -1) break
        const dropIdx = aIdx + dir
        const nextIdx = aIdx + 2 * dir
        if (dropIdx < 0 || dropIdx >= result.length) break
        if (nextIdx < 0 || nextIdx >= result.length) break
        const drop = result[dropIdx]
        const next = result[nextIdx]
        if (drop.z !== z || next.z !== z) break
        if (Math.hypot(drop.x - anchor.x, drop.y - anchor.y) > maxPullDistance)
          break
        if (!segClearOfForeign(anchor, next, z, segR, clearance, foreign)) break
        result.splice(dropIdx, 1)
      }
      return result
    }

    // ---- START: pull the first transition via onto the start pad ----
    let startPulled = false
    {
      const pts = route.route
      const startZ = pts[0].z
      let vi = -1
      for (let k = 1; k < pts.length; k++) {
        if (pts[k].z !== startZ) {
          vi = k
          break
        }
      }
      if (vi !== -1) {
        const pad = pts[0]
        const via = pts[vi] // at the transition XY, on the new layer
        const newZ = via.z
        const pullDist = Math.hypot(via.x - pad.x, via.y - pad.y)
        const terminalPad =
          pullDist > EPS && pullDist <= maxPullDistance
            ? findTerminalPad(route, pad, obstacles, connMap)
            : undefined
        if (
          terminalPad &&
          isPullClear(pad, via, newZ, viaR, segR, clearance, foreign)
        ) {
          // pad(startZ) → via@pad(startZ→newZ) → newseg to old via XY → rest on newZ
          const rest = pts.slice(vi)
          const viaExit = { x: pad.x, y: pad.y, z: newZ }
          route.route = [pts[0], viaExit, ...rest]
          removeViaAt(route.vias, { x: via.x, y: via.y })
          route.vias.push({ x: pad.x, y: pad.y })
          route.route = tautenRunFromPad(route.route, viaExit, 1)
          startPulled = true
        }
      }
    }

    // ---- END: pull the last transition via onto the end pad ----
    // Skip when the last transition is the one START already owns: with a single transition it is
    // both first and last, so END runs only if START left it (the via may be near the end pad).
    {
      const pts = route.route
      const last = pts.length - 1
      const endZ = pts[last].z
      const runEnd = countTransitions(pts) >= 2 || !startPulled
      let ci = -1 // index of the first point of the final same-layer run
      for (let k = last; runEnd && k >= 1; k--) {
        if (pts[k].z === endZ && pts[k - 1].z !== endZ) {
          ci = k
          break
        }
      }
      if (ci !== -1) {
        const pad = pts[last]
        const via = pts[ci - 1] // at the transition XY, on the pre-terminal layer
        const preZ = via.z
        const pullDist = Math.hypot(via.x - pad.x, via.y - pad.y)
        const terminalPad =
          pullDist > EPS && pullDist <= maxPullDistance
            ? findTerminalPad(route, pad, obstacles, connMap)
            : undefined
        if (
          terminalPad &&
          isPullClear(pad, via, preZ, viaR, segR, clearance, foreign)
        ) {
          // head up to the pre-terminal via XY (preZ) → newseg to pad → via@pad(preZ→endZ)
          const head = pts.slice(0, ci)
          const viaExit = { x: pad.x, y: pad.y, z: preZ }
          route.route = [...head, viaExit, pts[last]]
          removeViaAt(route.vias, { x: via.x, y: via.y })
          route.vias.push({ x: pad.x, y: pad.y })
          route.route = tautenRunFromPad(route.route, viaExit, -1)
        }
      }
    }
  }

  return out
}
