"""Factory layout rules that the mission and the agents depend on."""
import math

import factory_layout as L

MIN_STATION_CLEARANCE = 2.0   # m, a station closer than this to an agent route made laps fail (forecast on the goal)


def dist_to_segment(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def route_segments(agent):
    pts = [(w[0], w[1]) for w in agent[3]]
    return [(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]


def inside_hall(x, y, margin=0.0):
    return abs(x) <= L.HALL_X / 2 - margin and abs(y) <= L.HALL_Y / 2 - margin


def test_stations_inside_hall():
    for name, x, y, _ in L.STATIONS:
        assert inside_hall(x, y, margin=0.5), name


def test_stations_clear_of_agent_routes():
    for name, x, y, _ in L.STATIONS:
        for agent in L.AGENTS:
            d = min(dist_to_segment((x, y), a, b) for a, b in route_segments(agent))
            assert d >= MIN_STATION_CLEARANCE, "%s is %.2f m from %s route" % (name, d, agent[0])


def test_stations_clear_of_pillars():
    for name, x, y, _ in L.STATIONS:
        for px, py, label in L.PILLARS:
            assert math.hypot(x - px, y - py) >= 1.5, "%s too close to pillar %s" % (name, label)


def test_agent_waypoints_inside_hall():
    for agent in L.AGENTS:
        for w in agent[3]:
            assert inside_hall(w[0], w[1], margin=0.3), agent[0]


def test_park_positions_outside_hall():
    for name, (x, y) in L.PARK.items():
        assert not inside_hall(x, y), name


def test_every_agent_has_a_park_position():
    assert {a[0] for a in L.AGENTS} == set(L.PARK)
