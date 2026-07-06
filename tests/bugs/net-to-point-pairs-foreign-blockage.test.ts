import { expect, test } from "bun:test"
import { NetToPointPairsSolver } from "lib/solvers/NetToPointPairsSolver/NetToPointPairsSolver"
import type { SimpleRouteJson } from "lib/types"

// A 3-pad net where the two closest pads (A, B) have a FOREIGN pad (X) sitting on the straight run
// between them. The Euclidean MST would pick the short A–B edge, which no router can realize (the
// foreign pad fills it) — so a pad gets stranded. The blockage-aware MST must instead connect both
// A and B to the far hub H, whose longer spans have room to detour the foreign pad.
test("net-to-point-pairs avoids a foreign-pad-blocked short edge", () => {
  const A = { x: -59.83, y: 18.0, layer: "top", pointId: "A" }
  const B = { x: -59.83, y: 17.0, layer: "top", pointId: "B" }
  const H = { x: -57.15, y: 19.45, layer: "top", pointId: "H" }
  const pad = (x: number, y: number, net: string) => ({
    type: "rect" as const, layers: ["top"], center: { x, y }, width: 1.3, height: 0.3, connectedTo: [net],
  })
  const srj: SimpleRouteJson = {
    layerCount: 6,
    minTraceWidth: 0.2,
    minTraceClearance: 0.14,
    obstacles: [
      pad(A.x, A.y, "sig"),
      pad(B.x, B.y, "sig"),
      { ...pad(H.x, H.y, "sig"), width: 1, height: 1 },
      pad(-59.83, 17.5, "other"), // foreign pad ON the A–B line
      pad(-59.83, 18.5, "other"),
    ],
    connections: [{ name: "sig", pointsToConnect: [A, B, H] as any }],
    bounds: { minX: -68, maxX: 27, minY: -39, maxY: 37 },
  }

  const solver = new NetToPointPairsSolver(srj)
  solver.solve()
  expect(solver.solved).toBe(true)

  const pairs = solver.newConnections.map((c) =>
    c.pointsToConnect
      .map((p: any) => p.pointId)
      .sort()
      .join("-"),
  )
  // no cramped A–B edge; both pads reach the hub on their own (detourable) spans
  expect(pairs).not.toContain("A-B")
  expect(pairs).toContain("A-H")
  expect(pairs).toContain("B-H")
})
