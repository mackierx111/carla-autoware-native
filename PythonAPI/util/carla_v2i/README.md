# carla_v2i -- V2I traffic signal publisher for CARLA (AWSIM port)

Publishes the states of lanelet2-placed CARLA traffic lights as
`autoware_perception_msgs/TrafficLightGroupArray` (default topic
`/v2x/traffic_signals`, 10 Hz, QoS Reliable/Volatile/KeepLast(1)), so that
Autoware can obey signals without camera-based recognition via
`autoware_traffic_light_arbiter` (`external_traffic_signals` input).

Verified end-to-end on Odaiba against a pilot-auto.x2 stack: the arbiter's
judged output carries the published group states, and the ego stops on a
red hold and departs on green.

## Usage

    source <autoware_ws>/install/setup.bash   # rclpy + autoware_perception_msgs
    cd <repo>/PythonAPI/util
    python3 -m carla_v2i --osm-path /path/to/lanelet2_map.osm \
        --ros-args -p use_sim_time:=true

Group ids are lanelet2 regulatory-element relation ids by default
(`--id-mode way` switches to way ids, matching AWSIM's TrafficSignalId).

**`use_sim_time` is required whenever the consuming stack runs on `/clock`**
(any CARLA `--ros2` E2E setup): `traffic_light_arbiter` drops external
messages whose stamp differs from its own clock by more than
`external_delay_tolerance` (5 s by default), so wall-clock stamps never
ingest and the failure is silent. Only a stack running on wall clock may
omit it. Everything from `--ros-args` onward is passed to rclpy.

### Options

| option | default | meaning |
|---|---|---|
| `--osm-path` | (required) | lanelet2 map; source of regulatory-element relation ids |
| `--topic` | `/v2x/traffic_signals` | output topic. Check the arbiter's actual input with `ros2 node info <arbiter>` -- launch-file arg defaults may name a topic nothing subscribes to |
| `--rate-hz` | 10.0 | publish rate |
| `--radius-m` | 150.0 | ego-distance culling (AWSIM parity): only lights within this radius of the ego are published, so far-away groups stay unknown to Autoware. `<= 0` publishes all lights and needs no ego |
| `--id-mode` | `relation` | group id space: `relation` (Autoware convention) or `way` |
| `--ego-role-name` | `ego` | ego vehicle role_name (`hero` is also accepted) |
| `--carla-host` / `--carla-port` | localhost / 2000 | CARLA RPC endpoint |
| `--prediction-steps` | 6 | future state transitions to predict (0 disables) |

## Coexistence with scenario_simulator_v2 (ss2)

ss2's `V2ITrafficLights` publishes the same topics itself (both
`/v2x/traffic_signals` and the perception external topic), so:

- **Do NOT run together with a scenario that declares `<id> v2i` signals** --
  both publishers feed the arbiter and conflict.
- Running inside an ss2 stack is fine when the scenario declares **no** v2i
  signals: ss2 then registers no v2i lights, and this publisher is the only
  source on the arbiter input (verified on the pilot-auto.x2 stack).
- CARLA-standalone use (CARLA autonomous cycling or PythonAPI control +
  Autoware direct) works as before.

## Notes

- Read-only towards CARLA (never modifies world or actor state).
- Lights without a regulatory-element relation are not published
  (AWSIM parity).
- The traffic-light table is built at startup with a retry: a freshly
  connected client sees an empty actor list until the episode state reaches
  it, which matters when starting right as a synchronous-mode owner (e.g.
  the ss2 bridge) begins ticking.
