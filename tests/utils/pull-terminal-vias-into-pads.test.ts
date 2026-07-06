import { expect, test } from "bun:test"
import type { Obstacle } from "lib/types"
import type { HighDensityIntraNodeRoute } from "lib/types/high-density-types"
import { pullTerminalViasIntoPads } from "lib/utils/pullTerminalViasIntoPads"

// z0 = top, z1 = bottom (2-layer)
const pad = (
  name: string,
  x: number,
  y: number,
): Obstacle => ({
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

test("pulls both terminal vias onto their pads when the columns are clear", () => {
  const out = pullTerminalViasIntoPads([dipRoute()], {
    ...opts,
    obstacles: [pad("A", 0, 0), pad("A", 10, 0)],
  })
  // vias moved from (1,0)/(9,0) onto the pad centers (0,0)/(10,0)
  expect(viaXys(out[0])).toEqual([
    [0, 0],
    [10, 0],
  ])
  // the trace now vias immediately at each pad and runs on the bottom layer between them
  expect(out[0].route[0]).toMatchObject({ x: 0, y: 0, z: 0 })
  expect(out[0].route[1]).toMatchObject({ x: 0, y: 0, z: 1 })
  const last = out[0].route[out[0].route.length - 1]
  expect(last).toMatchObject({ x: 10, y: 0, z: 0 })
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
})
