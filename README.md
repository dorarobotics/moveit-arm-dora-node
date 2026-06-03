# moveit-arm-dora-node

MoveIt-based arm vendor adapter conforming to `SPEC-VENDOR-NODE-V1`. Wraps
[dora-moveit2](https://github.com/bobdingAI/dora-moveit2) and exposes ~12 verbs
covering motion, planning, gripper, and scene management.

## Quick start

```bash
pip install -e .
dora up
dora start examples/dataflow-standalone.yml
```

See `examples/` for Mode B (pure dora) and Mode C (client library, no dora).

## License

Apache-2.0 — see LICENSE.
