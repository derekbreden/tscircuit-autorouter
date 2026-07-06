import { expect, test } from "bun:test"
import type { Obstacle } from "lib/types"
import type { HighDensityIntraNodeRoute } from "lib/types/high-density-types"
import { pullTerminalViasIntoPads } from "lib/utils/pullTerminalViasIntoPads"

// z0 = top, z1 = bottom (2-layer)
const pad = (name: string, x: number, y: number): Obstacle => ({
  type: "rect",
  layers: ["top"],
  zLayers: [0],
  center: { x, y },
  width: 1,
  height: 1,
  connectedTo: [name],
})

// A route from a top pad at (0,0) to a top pad at (10,0) that dips to the bottom layer in the
// middle, so it carries a transition via a short hop off each pad.
const dipRoute = (): HighDensityIntraNodeRoute => ({
  connectionName: "A",
  traceThickness: 0.2,
  viaDiameter: 0.5,
  route: [
    { x: 0, y: 0, z: 0 },
    { x: 1, y: 0, z: 0 },
    { x: 1, y: 0, z: 1 },
    { x: 9, y: 0, z: 1 },
    { x: 9, y: 0, z: 0 },
    { x: 10, y: 0, z: 0 },
  ],
  vias: [
    { x: 1, y: 0 },
    { x: 9, y: 0 },
  ],
})

const opts = {
  layerCount: 2,
  viaDiameter: 0.5,
  clearance: 0.2,
}

const viaXys = (r: HighDensityIntraNodeRoute) =>
  r.vias.map((v) => [v.x, v.y]).sort((a, b) => a[0] - b[0])

const hasVertexAt = (r: HighDensityIntraNodeRoute, x: number, y: number) =>
  r.route.some((p) => Math.abs(p.x - x) < 1e-6 && Math.abs(p.y - y) < 1e-6)

test("pulls both terminal vias onto their pads and removes the detour vertices", () => {
  const out = pullTerminalViasIntoPads([dipRoute()], {
    ...opts,
    obstacles: [pad("A", 0, 0), pad("A", 10, 0)],
  })
  // vias moved from (1,0)/(9,0) onto the pad centers (0,0)/(10,0)
  expect(viaXys(out[0])).toEqual([
    [0, 0],
    [10, 0],
  ])
  // the old via locations were vestigial detours — they must be gone, leaving a clean
  // pad → straight bottom run → pad (4 points, no dip out to the old via XY)
  expect(hasVertexAt(out[0], 1, 0)).toBe(false)
  expect(hasVertexAt(out[0], 9, 0)).toBe(false)
  expect(out[0].route).toEqual([
    { x: 0, y: 0, z: 0 },
    { x: 0, y: 0, z: 1 },
    { x: 10, y: 0, z: 1 },
    { x: 10, y: 0, z: 0 },
  ])
})

test("leaves a via off its pad when a foreign trace blocks the pad column", () => {
  // A different-net trace runs down the x=0 column on the bottom layer, through pad A1's barrel.
  const foreign: HighDensityIntraNodeRoute = {
    connectionName: "B",
    traceThickness: 0.2,
    viaDiameter: 0.5,
    route: [
      { x: 0, y: -3, z: 1 },
      { x: 0, y: 3, z: 1 },
    ],
    vias: [],
  }
  const out = pullTerminalViasIntoPads([dipRoute(), foreign], {
    ...opts,
    obstacles: [pad("A", 0, 0), pad("A", 10, 0)],
  })
  // start via cannot move onto (0,0) (foreign copper in the column) but the clear end via does
  expect(viaXys(out[0])).toEqual([
    [1, 0],
    [10, 0],
  ])
})

test("leaves a genuine mid-channel via untouched", () => {
  // Both vias sit ~4mm from their terminals — beyond maxPullDistance, so neither is a pad escape.
  const midRoute: HighDensityIntraNodeRoute = {
    connectionName: "A",
    traceThickness: 0.2,
    viaDiameter: 0.5,
    route: [
      { x: 0, y: 0, z: 0 },
      { x: 4, y: 0, z: 0 },
      { x: 4, y: 0, z: 1 },
      { x: 6, y: 0, z: 1 },
      { x: 6, y: 0, z: 0 },
      { x: 10, y: 0, z: 0 },
    ],
    vias: [
      { x: 4, y: 0 },
      { x: 6, y: 0 },
    ],
  }
  const out = pullTerminalViasIntoPads([midRoute], {
    ...opts,
    obstacles: [pad("A", 0, 0), pad("A", 10, 0)],
  })
  expect(viaXys(out[0])).toEqual([
    [4, 0],
    [6, 0],
  ])
})

// A single-pull route that dips DOWN off the pad to reach a via spot, then heads to an endpoint.
const vRoute = (): HighDensityIntraNodeRoute => ({
  connectionName: "A",
  traceThickness: 0.2,
  viaDiameter: 0.5,
  route: [
    { x: 0, y: 0, z: 0 }, // pad
    { x: 0, y: -2, z: 0 }, // dip down on top to reach a clear column
    { x: 0, y: -2, z: 1 }, // via location (off pad)
    { x: 4, y: -1, z: 1 }, // heads up-right toward destination
  ],
  vias: [{ x: 0, y: -2 }],
})

test("taut-strings the detour after pulling the via onto the pad", () => {
  const out = pullTerminalViasIntoPads([vRoute()], {
    ...opts,
    obstacles: [pad("A", 0, 0)],
  })
  expect(viaXys(out[0])).toEqual([[0, 0]])
  // the dip-to-(0,-2) detour is gone: pad → via-in-pad → straight to the destination
  expect(hasVertexAt(out[0], 0, -2)).toBe(false)
  expect(out[0].route).toEqual([
    { x: 0, y: 0, z: 0 },
    { x: 0, y: 0, z: 1 },
    { x: 4, y: -1, z: 1 },
  ])
})

test("keeps the detour when the straight shortcut would hit foreign copper", () => {
  // Foreign z1 copper sits just below the pad, on the straight (0,0)->(4,-1) shortcut but clear of
  // the original dip: the via still lands in the pad, but the detour cannot be pulled taut.
  const foreign: HighDensityIntraNodeRoute = {
    connectionName: "B",
    traceThickness: 0.2,
    viaDiameter: 0.5,
    route: [
      { x: 1, y: -0.4, z: 1 },
      { x: 1, y: -0.1, z: 1 },
    ],
    vias: [],
  }
  const out = pullTerminalViasIntoPads([vRoute(), foreign], {
    ...opts,
    obstacles: [pad("A", 0, 0)],
  })
  // via still pulled onto the pad
  expect(viaXys(out[0])).toEqual([[0, 0]])
  // but the detour vertex is preserved (shortcut blocked)
  expect(hasVertexAt(out[0], 0, -2)).toBe(true)
})

test("does not mutate the input routes", () => {
  const input = dipRoute()
  pullTerminalViasIntoPads([input], {
    ...opts,
    obstacles: [pad("A", 0, 0), pad("A", 10, 0)],
  })
  expect(viaXys(input)).toEqual([
    [1, 0],
    [9, 0],
  ])
  expect(input.route).toHaveLength(6)
})
