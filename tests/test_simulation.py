"""Verify actual MuJoCo actuator routing, not command echoes."""
import os
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/sharpa_teleop"))
from sharpa_teleop.sim_model import HandSimulation


@unittest.skipUnless(os.environ.get("SHARPA_MODELS"), "requires Sharpa model resources")
class SimulationTests(unittest.TestCase):
    def test_bilateral_targets_drive_the_correct_physical_joints(self):
        sim = HandSimulation(os.environ["SHARPA_MODELS"])
        before = {side: sim.feedback(side)[0] for side in ("left", "right")}
        for side, target in (("left", 0.6), ("right", 0.3)):
            joints = sim.joints[side]
            positions = [0.0] * len(joints.names)
            positions[joints.names.index(f"{side}_index_MCP_FE")] = target
            sim.command(side, joints.names, positions)
            self.assertEqual(sim.feedback(side)[0], before[side], "commands must not teleport qpos")
        for _ in range(1500):
            sim.step()
        for side, target in (("left", 0.6), ("right", 0.3)):
            qpos, _ = sim.feedback(side)
            index = sim.joints[side].names.index(f"{side}_index_MCP_FE")
            self.assertAlmostEqual(qpos[index], target, delta=0.08)
        self.assertEqual(sim.model.nq, 44, "untracked base joints must remain fixed")


if __name__ == "__main__":
    unittest.main()
