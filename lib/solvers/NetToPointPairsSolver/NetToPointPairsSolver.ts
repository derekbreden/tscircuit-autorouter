import {
  ConnectionPoint,
  SimpleRouteConnection,
  SimpleRouteJson,
} from "lib/types"
import { BaseSolver } from "../BaseSolver"
import { buildMinimumSpanningTree } from "./buildMinimumSpanningTree"
import { GraphicsObject } from "graphics-debug"
import { mergeConnections } from "./mergeConnections"
import { seededRandom } from "lib/utils/cloneAndShuffleArray"

export const getExternalConnectionState = (
  connection: SimpleRouteConnection,
  srj?: SimpleRouteJson,
): {
  pointIdToGroup: Map<string, number>
  zeroWeightEdges: Array<{
    from: ConnectionPoint
    to: ConnectionPoint
    weight: number
  }>
} => {
  const externalGroups = connection.externallyConnectedPointIds ?? []
  const routedTraceGroups = getTraceConnectedPointGroups(connection, srj)
  const allExternalGroups = [...externalGroups, ...routedTraceGroups]
  const pointIdToGroup = new Map<string, number>()
  const pointById = new Map<string, ConnectionPoint>()

  for (const point of connection.pointsToConnect) {
    if (point.pointId) {
      pointById.set(point.pointId, point)
    }
  }

  const zeroWeightEdges: Array<{
    from: ConnectionPoint
    to: ConnectionPoint
    weight: number
  }> = []

  allExternalGroups.forEach((group, idx) => {
    const groupPoints = group
      .map((pointId) => pointById.get(pointId))
      .filter((point): point is ConnectionPoint => Boolean(point))

    for (const point of groupPoints) {
      if (point.pointId) {
        pointIdToGroup.set(point.pointId, idx)
      }
    }

    const representativePoint = groupPoints[0]
    if (!representativePoint) {
      return
    }

    for (let i = 1; i < groupPoints.length; i++) {
      zeroWeightEdges.push({
        from: representativePoint,
        to: groupPoints[i]!,
        weight: 0,
      })
    }
  })

  return { pointIdToGroup, zeroWeightEdges }
}

const getTraceConnectedPointGroups = (
  connection: SimpleRouteConnection,
  srj: SimpleRouteJson | undefined,
): string[][] => {
  if (!srj?.traces?.length) return []

  const connectionPointIds = new Set(
    connection.pointsToConnect
      .map((point) => point.pointId)
      .filter((pointId): pointId is string => Boolean(pointId)),
  )

  if (connectionPointIds.size === 0) return []

  const routedTraceGroups: string[][] = []
  for (const trace of srj.traces) {
    const traceConnectsTo = trace.connectsTo ?? []
    if (traceConnectsTo.length < 2) continue

    const connectedPointIds = traceConnectsTo.filter((connectsTo) =>
      connectionPointIds.has(connectsTo),
    )
    if (connectedPointIds.length >= 2) {
      routedTraceGroups.push(connectedPointIds)
    }
  }

  return routedTraceGroups
}

// True when segment a→b passes through the axis-aligned box centred at (cx,cy) with half-extents
// (hx,hy). Endpoints inside count; otherwise slab-clip the segment against the box.
const segmentIntersectsBox = (
  a: { x: number; y: number },
  b: { x: number; y: number },
  cx: number,
  cy: number,
  hx: number,
  hy: number,
): boolean => {
  const minX = cx - hx, maxX = cx + hx, minY = cy - hy, maxY = cy + hy
  const inside = (p: { x: number; y: number }) =>
    p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY
  if (inside(a) || inside(b)) return true
  const dx = b.x - a.x, dy = b.y - a.y
  let t0 = 0, t1 = 1
  for (const [p, q] of [
    [-dx, a.x - minX],
    [dx, maxX - a.x],
    [-dy, a.y - minY],
    [dy, maxY - a.y],
  ] as const) {
    if (p === 0) {
      if (q < 0) return false
      continue
    }
    const r = q / p
    if (p < 0) {
      if (r > t1) return false
      if (r > t0) t0 = r
    } else {
      if (r < t0) return false
      if (r < t1) t1 = r
    }
  }
  return t0 <= t1
}

// A same-net edge is "blocked" when its straight span runs through a foreign-net pad's keepout. The
// net's own ids come from the obstacles sitting on the connection's points, so any obstacle sharing
// none of them is foreign; pours are skipped (their antipads carve their own clearance).
export const buildForeignBlockagePredicate = (
  srj: SimpleRouteJson,
  connection: SimpleRouteConnection,
): ((a: ConnectionPoint, b: ConnectionPoint) => boolean) | undefined => {
  const obstacles = srj.obstacles ?? []
  if (!obstacles.length) return undefined
  const pts = connection.pointsToConnect
  const onPoint = (o: (typeof obstacles)[number]) =>
    pts.some((p) => Math.abs(o.center.x - p.x) < 0.05 && Math.abs(o.center.y - p.y) < 0.05)
  const netIds = new Set<string>()
  for (const o of obstacles) if (!o.isCopperPour && onPoint(o)) for (const id of o.connectedTo ?? []) netIds.add(id)
  if (netIds.size === 0) return undefined
  const foreign = obstacles.filter(
    (o) => !o.isCopperPour && !(o.connectedTo ?? []).some((id) => netIds.has(id)),
  )
  if (foreign.length === 0) return undefined
  const margin =
    (srj.minTraceWidth ?? 0.2) / 2 +
    (srj.minTraceClearance ?? srj.minTraceToPadEdgeClearance ?? 0.15)
  // Only a CRAMPED edge is blocked: the foreign pad lies on the run AND both endpoints hug that same
  // pad (each within a pad-radius+clearance of its centre). That is the no-detour-room case — two
  // pads with a foreign pad between them. An edge that merely grazes a foreign pad on a long span
  // has room to route around it and is left alone, so the decomposition changes only where it must.
  return (a, b) =>
    foreign.some((o) => {
      const reach = Math.max(o.width, o.height) / 2 + margin
      const da = Math.hypot(a.x - o.center.x, a.y - o.center.y)
      const db = Math.hypot(b.x - o.center.x, b.y - o.center.y)
      return (
        da <= reach &&
        db <= reach &&
        segmentIntersectsBox(a, b, o.center.x, o.center.y, o.width / 2 + margin, o.height / 2 + margin)
      )
    })
}

export const areExternallyConnected = (
  pointIdToGroup: Map<string, number>,
  a: { pointId?: string },
  b: { pointId?: string },
) => {
  if (!a.pointId || !b.pointId) return false
  const g1 = pointIdToGroup.get(a.pointId)
  const g2 = pointIdToGroup.get(b.pointId)
  return g1 !== undefined && g1 === g2
}

/**
 * Converts a net containing many points to connect into an array of point pair
 * connections.
 *
 * For example, a connection with 3 pointsToConnect could be turned into 2
 * connections of 2 points each.
 *
 * Where we create the minimum number of pairs, we're using a minimum spanning
 * tree (MST).
 *
 * Sometimes it can be used to add additional traces to help make sure we
 * distribute load effectively. In this version we don't do that!
 */
export class NetToPointPairsSolver extends BaseSolver {
  override getSolverName(): string {
    return "NetToPointPairsSolver"
  }

  unprocessedConnections: Array<SimpleRouteConnection>
  newConnections: Array<SimpleRouteConnection>

  constructor(
    public ogSrj: SimpleRouteJson,
    public colorMap: Record<string, string> = {},
  ) {
    super()
    this.unprocessedConnections = mergeConnections([...ogSrj.connections])
    this.newConnections = []
  }

  _step() {
    if (this.unprocessedConnections.length === 0) {
      this.solved = true
      return
    }
    const connection = this.unprocessedConnections.pop()!

    // ----------------------------------------------
    // 1.  Detect externally-connected point groups
    // ----------------------------------------------
    const { pointIdToGroup, zeroWeightEdges } = getExternalConnectionState(
      connection,
      this.ogSrj,
    )

    if (connection.pointsToConnect.length === 2) {
      if (
        areExternallyConnected(
          pointIdToGroup,
          connection.pointsToConnect[0],
          connection.pointsToConnect[1],
        )
      ) {
        // No routing required – they are already connected off-board
        return
      }
      this.newConnections.push({
        ...connection,
        rootConnectionName: connection.rootConnectionName ?? connection.name,
      })
      return
    }

    const edges = buildMinimumSpanningTree(connection.pointsToConnect, {
      extraEdges: zeroWeightEdges,
      isBlockedEdge: buildForeignBlockagePredicate(this.ogSrj, connection),
    })

    let mstIdx = 0
    for (const edge of edges) {
      if (areExternallyConnected(pointIdToGroup, edge.from, edge.to)) continue
      this.newConnections.push({
        pointsToConnect: [edge.from, edge.to],
        name: `${connection.name}_mst${mstIdx++}`,
        rootConnectionName: connection.rootConnectionName ?? connection.name,
        mergedConnectionNames: connection.mergedConnectionNames,
        netConnectionName: connection.netConnectionName,
      })
    }
  }

  getNewSimpleRouteJson(): SimpleRouteJson {
    const detachedSrj = structuredClone(this.ogSrj)
    return {
      ...detachedSrj,
      connections: structuredClone(this.newConnections),
    }
  }

  visualize(): GraphicsObject {
    const graphics: GraphicsObject = {
      lines: [],
      points: [],
      rects: [],
      circles: [],
      coordinateSystem: "cartesian",
      title: "Net To Point Pairs Visualization",
    }

    // Draw unprocessed connections in red
    this.unprocessedConnections.forEach((connection) => {
      // Draw points
      connection.pointsToConnect.forEach((point) => {
        graphics.points!.push({
          x: point.x,
          y: point.y,
          color: "red",
          label: connection.name,
        })
      })

      // Draw lines connecting all points in the connection
      const fullyConnectedEdgeCount = connection.pointsToConnect.length ** 2
      const random = seededRandom(0)
      const alreadyPlacedEdges = new Set<string>()
      for (
        let i = 0;
        i <
        Math.max(
          fullyConnectedEdgeCount,
          connection.pointsToConnect.length * 2,
        );
        i++
      ) {
        const a = Math.floor(random() * connection.pointsToConnect.length)
        const b = Math.floor(random() * connection.pointsToConnect.length)
        if (alreadyPlacedEdges.has(`${a}-${b}`)) continue
        alreadyPlacedEdges.add(`${a}-${b}`)
        graphics.lines!.push({
          points: [
            connection.pointsToConnect[a],
            connection.pointsToConnect[b],
          ],
          strokeColor: "rgba(255,0,0,0.25)",
        })
      }
    })

    // Draw processed connections with appropriate colors
    this.newConnections.forEach((connection) => {
      const color = this.colorMap?.[connection.name] || "blue"

      // Draw points
      connection.pointsToConnect.forEach((point) => {
        graphics.points!.push({
          x: point.x,
          y: point.y,
          color: color,
          label: connection.name,
        })
      })

      // Draw lines connecting all points in the connection
      for (let i = 0; i < connection.pointsToConnect.length - 1; i++) {
        for (let j = i + 1; j < connection.pointsToConnect.length; j++) {
          graphics.lines!.push({
            points: [
              connection.pointsToConnect[i],
              connection.pointsToConnect[j],
            ],
            strokeColor: color,
          })
        }
      }
    })

    return graphics
  }
}
