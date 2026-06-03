# moveit-arm-dora-node

MoveIt-based arm vendor adapter conforming to `SPEC-VENDOR-NODE-V1`. Wraps
[dora-moveit2](https://github.com/bobdingAI/dora-moveit2) and exposes ~12 verbs
covering motion, planning, gripper, and scene management.

## Quick start

```bash
# To actually drive an arm you need the runtime planning backend (dora-moveit2):
pip install -e ".[runtime]"
dora up
dora start examples/dataflow-standalone.yml
```

The base `pip install -e .` is enough for unit tests and Mode C type imports
(both use the in-process fakes); the `runtime` extra pulls in `dora-moveit2`,
the planning/IK/execution backend required for real motion.

See `examples/` for Mode B (pure dora) and Mode C (client library, no dora).

## License

Apache-2.0 — see LICENSE.
