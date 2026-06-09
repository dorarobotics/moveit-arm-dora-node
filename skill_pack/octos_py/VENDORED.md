# Vendored: octos_py

This `octos_py/` package is **vendored** (copied in) from the octos agent SDK
tutorial — https://github.com/23garyd/octos-tutorial — so the optional LLM-agent
variant of the SO-101 / reBot demos works without an extra clone.

- Used only by `skill_pack/arm_agent.py` (the "drive it from a sentence" variant).
  The default **deterministic** demo (`skill_pickplace.py`) does not import it.
- Pure Python. The only runtime dependency is `openai` (the provider client, lazily
  imported in `provider.py`; the agent talks to a **local Ollama** OpenAI-compatible
  endpoint by default).
- `ros2_bridge.py` from upstream is intentionally **omitted** here (ROS2/rclpy
  integration, not used by the agent path).

To refresh against upstream, re-copy the package (minus `ros2_bridge.py`) from the
octos-tutorial repo. Attribution and license follow that upstream project.
