#!/usr/bin/env python3
"""Fixed scenario tick-cost driver (skeletal spec §6.2): N walkers + M vehicles, static, one RGL lidar.
Measures wall time of world.tick() (server frame incl. RGL scene update) after warm-up.
Usage: python3 measure_skeletal_tick_cost.py --label C [--walkers 7 --vehicles 3 --warmup 100 --frames 600]"""
import argparse, json, statistics, time
import carla

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--label", required=True); ap.add_argument("--walkers", type=int, default=7); ap.add_argument("--vehicles", type=int, default=3)
    ap.add_argument("--warmup", type=int, default=100); ap.add_argument("--frames", type=int, default=600)
    a = ap.parse_args()
    client = carla.Client(a.host, a.port); client.set_timeout(20.0); world = client.get_world()
    original = world.get_settings(); actors = []
    try:
        s = world.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = 0.05; world.apply_settings(s)
        bp = world.get_blueprint_library()
        lidar = bp.find("sensor.lidar.rgl")
        for k, v in {"channels": 64, "range": 50, "points_per_second": 1300000, "rotation_frequency": 20, "sensor_tick": 0.05,
                     "rgl_lidar_topic_name": "/perf/rgl", "rgl_lidar_pointcloud_format": "PointXYZIRCAEDT"}.items():
            lidar.set_attribute(k, str(v))
        actors.append(world.spawn_actor(lidar, carla.Transform(carla.Location(0, 0, 1.65))))
        gp = world.ground_projection(carla.Location(10.0, 0.0, 5.0), 20.0); gz = gp.location.z if gp else 0.0
        walkers = bp.filter("walker.pedestrian.*"); vehicles = bp.filter("vehicle.*")
        for i in range(a.walkers):
            act = world.spawn_actor(walkers[i % len(walkers)], carla.Transform(carla.Location(10.0 + 2.0 * i, -3.0, gz + 0.2)))
            if act: actors.append(act)
        for i in range(a.vehicles):
            act = world.spawn_actor(vehicles[i % len(vehicles)], carla.Transform(carla.Location(12.0 + 6.0 * i, 3.5, gz + 0.3)))
            if act: actors.append(act)
        for _ in range(a.warmup): world.tick()
        dts = []
        for _ in range(a.frames):
            t0 = time.perf_counter(); world.tick(); dts.append((time.perf_counter() - t0) * 1000.0)
        dts.sort()
        print(json.dumps({"label": a.label, "actors": len(actors) - 1, "median_ms": statistics.median(dts), "p95_ms": dts[int(0.95 * len(dts)) - 1]}))
    finally:
        for x in reversed(actors):
            try: x.destroy()
            except Exception: pass
        try: world.apply_settings(original)
        except Exception: pass

if __name__ == "__main__":
    main()
