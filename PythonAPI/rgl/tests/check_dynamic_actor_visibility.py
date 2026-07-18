#!/usr/bin/env python3
"""GATE (spec 10-0 / 13-2): verify dynamic actors are visible to the RGL lidar.

Spawns sensor.lidar.rgl, measures a BASELINE point count inside each (empty) target
box, THEN spawns a vehicle and a walker 10 m in front (in separate, non-overlapping
boxes) and re-measures. Requiring a point-count DELTA over the baseline avoids a false
PASS from static geometry (ground/props) that already falls inside a box.

PASS  : vehicle box gains >= baseline + VEH_MIN_DELTA points AND
        walker  box gains >= baseline + WALK_MIN_DELTA points.
FAIL  : either delta is too small -> escalate per spec 13-2 (RadarTracks/classification
        acceptance is blocked until the root cause is understood/fixed).

IMPORTANT DEVIATION FROM THE ORIGINAL TASK BRIEF (empirically forced):
    The brief assumed sensor.listen(callback) on the carla-native LidarMeasurement
    pipeline (ARGLLidar::PostPhysTick -> SerializeAndSend when AreClientsListening())
    would work like any other CARLA lidar. In this build, subscribing via
    sensor.listen() and touching the resulting carla.LidarMeasurement (even just
    reading len()/raw_data, never mind iterating .point.x) reliably SEGFAULTS the
    Python client on the very first message -- reproduced with channels=1..64,
    points_per_second=100..1,300,000, with and without iterating. No listen() ->
    no crash (AreClientsListening() short-circuits and nothing is sent), so this is a
    real client-side deserialization bug in the ARGLLidar -> LidarData -> Python
    LidarMeasurement path, independent of this test.
    Workaround used here: publish via the sensor's built-in ROS2 PointCloud2 output
    (rgl_lidar_topic_name / PointXYZIRCAEDT format) and subscribe with rclpy, exactly
    like PythonAPI/rgl/tests/test_regression.py's capture_ros2_msg()/parse_points().
    This requires ROS2 to be sourced (e.g. `source /opt/ros/humble/setup.bash`) before
    running this script, even though the original brief said "ROS2 not required".

Usage:
    # CARLA server must be running (RGL build). Source ROS2 first:
    #   source /opt/ros/humble/setup.bash
    python3 check_dynamic_actor_visibility.py [--host 127.0.0.1] [--port 2000]
"""

import argparse
import struct
import sys
import time

import carla

try:
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
except ImportError:
    print(
        "ERROR: rclpy / sensor_msgs not importable. This gate script needs ROS2 "
        "sourced (e.g. `source /opt/ros/humble/setup.bash`) because the RGL lidar's "
        "carla-native sensor.listen() path segfaults the Python client in this build "
        "-- see the module docstring. Falling back to ROS2 PointCloud2 instead.",
        file=sys.stderr,
    )
    sys.exit(2)

VEH_MIN_DELTA = 50       # vehicle box must gain >= baseline + this many points
WALK_MIN_DELTA = 20      # walker box must gain >= baseline + this many points
FRONT_DIST_M = 10.0      # target distance in front of the sensor (meters)
BOX_HALF_X = 3.0         # sensor-local X (forward) half-extent of the count box (m)
BOX_HALF_Y = 1.5         # sensor-local Y half-extent (m); keeps the two boxes disjoint
VEH_Y = 3.0              # vehicle box center in Y -> box covers [1.5, 4.5]
WALK_Y = -3.0            # walker box center in Y  -> box covers [-4.5, -1.5] (disjoint)
Z_MIN, Z_MAX = -2.0, 3.0  # sensor-local Z window (m)
BASELINE_TICKS = 15      # ticks to measure the empty-scene baseline before spawning targets
TICKS = 30               # ticks to measure after spawning targets
SETTLE_TICKS = 5         # extra (uncounted) ticks after spawn, for physics/pose to settle

ROS2_TOPIC = "/gate/rgl_dynamic_actor_visibility"
TICK_WAIT_TIMEOUT_S = 2.0


def count_in_box(points, x_center, y_center):
    """points: iterable of (x, y, z) sensor-local meters."""
    n = 0
    for (x, y, z) in points:
        if (abs(x - x_center) <= BOX_HALF_X and abs(y - y_center) <= BOX_HALF_Y
                and Z_MIN <= z <= Z_MAX):
            n += 1
    return n


def parse_xyz(msg):
    """Parse the leading x,y,z float32 triplet out of each PointXYZIRCAEDT record."""
    pts = []
    off = 0
    step = msg.point_step
    for _ in range(msg.width):
        x, y, z = struct.unpack_from("<fff", msg.data, off)
        pts.append((x, y, z))
        off += step
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()

    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    rclpy.init()
    node = rclpy.create_node("rgl_dynamic_actor_visibility_gate")
    latest = {"msg": None, "count": 0}

    def on_pointcloud(msg):
        latest["msg"] = msg
        latest["count"] += 1

    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE
    node.create_subscription(PointCloud2, ROS2_TOPIC, on_pointcloud, qos)

    actors = []

    def tick_and_wait():
        """world.tick() then block (bounded) for the NEW lidar message for this tick.

        This is the ROS2-pathway equivalent of "match sensor callback frame to world
        snapshot frame" -- we can't correlate a CARLA frame number through ROS2
        directly, so we instead require a freshly-arrived message (msg counter
        incremented) before counting, which prevents racing a stale/previous-tick
        point cloud since sensor_tick == fixed_delta_seconds (one sweep per tick).
        """
        prev = latest["count"]
        frame = world.tick()
        t0 = time.time()
        while latest["count"] == prev and (time.time() - t0) < TICK_WAIT_TIMEOUT_S:
            rclpy.spin_once(node, timeout_sec=0.02)
        return frame, latest["msg"]

    def measure(n_ticks):
        veh_max = 0
        walk_max = 0
        got_any = False
        for _ in range(n_ticks):
            _, msg = tick_and_wait()
            if msg is None:
                continue
            got_any = True
            pts = parse_xyz(msg)
            veh_max = max(veh_max, count_in_box(pts, FRONT_DIST_M, VEH_Y))
            walk_max = max(walk_max, count_in_box(pts, FRONT_DIST_M, WALK_Y))
        return veh_max, walk_max, got_any

    try:
        bp_lib = world.get_blueprint_library()

        # Sensor on a pole at the origin, facing +X (default rotation). World coords
        # == sensor-local coords here since there is no parent actor and rotation is
        # identity, matching the box definitions above.
        lidar_bp = bp_lib.find("sensor.lidar.rgl")
        lidar_bp.set_attribute("channels", "64")
        lidar_bp.set_attribute("range", "50")
        lidar_bp.set_attribute("upper_fov", "15")
        lidar_bp.set_attribute("lower_fov", "-25")
        lidar_bp.set_attribute("points_per_second", "1300000")
        # Match rotation_frequency to the tick rate (1 / fixed_delta_seconds) so each
        # published point cloud is one complete, consistent 360-degree sweep per tick
        # instead of a partial arc -- verified empirically to make box counts stable
        # tick-to-tick for a static scene.
        lidar_bp.set_attribute("rotation_frequency", "20")
        lidar_bp.set_attribute("sensor_tick", "0.05")
        lidar_bp.set_attribute("rgl_lidar_topic_name", ROS2_TOPIC)
        lidar_bp.set_attribute("rgl_lidar_topic_frame_id", "lidar")
        lidar_bp.set_attribute("rgl_lidar_pointcloud_format", "PointXYZIRCAEDT")
        sensor_tf = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.65))
        sensor = world.spawn_actor(lidar_bp, sensor_tf)
        actors.append(sensor)

        # Warm-up: let the ROS2 pub/sub match and the first sweeps settle.
        for _ in range(5):
            tick_and_wait()

        # ---- Baseline: measure each box BEFORE spawning targets (static geometry only).
        veh_baseline, walk_baseline, base_ok = measure(BASELINE_TICKS)
        if not base_ok:
            print("ERROR: RGL lidar produced ZERO point-cloud messages during the "
                  "baseline phase. This is a sensor/pipeline problem, not a dynamic-"
                  "actor visibility result -- investigate sensor attachment, "
                  "AreClientsListening()/ROS2 topic wiring, range, and world ticking "
                  "before concluding anything about vehicles/walkers.", file=sys.stderr)
            return 2

        # Vehicle 10 m in front, in the +Y box; grounded via world.ground_projection.
        veh_bp = bp_lib.filter("vehicle.*")[0]
        walk_bp = bp_lib.filter("walker.pedestrian.*")[0]
        veh_gp = world.ground_projection(carla.Location(FRONT_DIST_M, VEH_Y, 5.0), 20.0)
        walk_gp = world.ground_projection(carla.Location(FRONT_DIST_M, WALK_Y, 5.0), 20.0)
        veh_ground_z = veh_gp.location.z if veh_gp is not None else 0.0
        walk_ground_z = walk_gp.location.z if walk_gp is not None else 0.0

        veh = world.spawn_actor(
            veh_bp, carla.Transform(carla.Location(x=FRONT_DIST_M, y=VEH_Y, z=veh_ground_z + 0.3)))
        actors.append(veh)

        # Walker 10 m in front, in the -Y box (non-overlapping with the vehicle box).
        walk = world.spawn_actor(
            walk_bp, carla.Transform(carla.Location(x=FRONT_DIST_M, y=WALK_Y, z=walk_ground_z + 0.2)))
        actors.append(walk)

        for _ in range(SETTLE_TICKS):
            tick_and_wait()

        veh_hits_max, walk_hits_max, meas_ok = measure(TICKS)
        if not meas_ok:
            print("ERROR: RGL lidar produced ZERO point-cloud messages after spawning "
                  "the targets (but produced points during baseline). Investigate "
                  "sensor/topic stability.", file=sys.stderr)
            return 2

        veh_need = veh_baseline + VEH_MIN_DELTA
        walk_need = walk_baseline + WALK_MIN_DELTA
        print(f"vehicle box: baseline={veh_baseline}, measured={veh_hits_max}, "
              f"need>={veh_need} (baseline+{VEH_MIN_DELTA})")
        print(f"walker  box: baseline={walk_baseline}, measured={walk_hits_max}, "
              f"need>={walk_need} (baseline+{WALK_MIN_DELTA})")

        veh_ok = veh_hits_max >= veh_need
        walk_ok = walk_hits_max >= walk_need
        print(f"vehicle target: {'PASS' if veh_ok else 'FAIL'}")
        print(f"walker  target: {'PASS' if walk_ok else 'FAIL'}")

        if veh_ok and walk_ok:
            print("GATE RESULT: PASS - dynamic actors add points over baseline (visible to the RGL lidar).")
            return 0

        print("GATE RESULT: FAIL - dynamic actor(s) did NOT add enough points over baseline.")
        print("  -> STOP. Escalate per spec 13-2 before RadarTracks/classification acceptance.")
        return 1
    finally:
        for a in reversed(actors):
            try:
                a.destroy()
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        try:
            world.apply_settings(original)
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
