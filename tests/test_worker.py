"""Integration regression: never publish the optimizer's initial shared-memory zeros."""
import json
import math
import os
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
SDK = os.environ.get('SHARPA_MANUS_SDK', '')
PYTHON = Path(os.environ.get('RETARGET_PYTHON', ROOT / '.pixi/envs/retarget/bin/python'))


@unittest.skipUnless(SDK and PYTHON.is_file(), 'requires authorized SDK and pixi retarget environment')
class NativeWorkerTests(unittest.TestCase):
    def test_first_result_waits_for_computation(self):
        points = [[(finger - 2) * .018, .025 + joint * .022, .005, 1, 0, 0, 0]
                  for finger in range(5) for joint in range(5)]
        request = {'side': 'left', 'frame': 1, 'points': points}
        result = subprocess.run(
            [str(PYTHON), '-I', str(ROOT / 'src/sharpa_teleop/sharpa_teleop/retarget_worker.py'),
             '--sdk-root', SDK], input=json.dumps(request) + '\n', text=True,
            capture_output=True, timeout=30, check=True)
        messages = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual(messages[0], {'type': 'ready'})
        command = messages[1]
        self.assertNotIn('error', command, result.stderr)
        self.assertEqual(command['frame'], 1)
        self.assertEqual(command['side'], 'left')
        # This spread-finger fixture has a nonzero thumb MCP solution. Before
        # completion synchronization, frame 1 incorrectly returned initial zeros.
        thumb = command['names'].index('left_thumb_MCP_FE')
        self.assertGreater(command['positions'][thumb], 0.1)
        self.assertTrue(all(math.isfinite(value) for value in command['positions']))

    def test_compiled_solver_tracks_new_targets_and_rejects_invalid_inputs(self):
        # A changing analytic optimum detects C argument-layout errors and stale
        # results after the cold-start solve, without relying on SDK pose tuning.
        script = f"""
import sys
sys.path.insert(0, {str(ROOT / 'src/sharpa_teleop')!r})
import casadi as ca
import numpy as np
from sharpa_teleop.retarget_solver import CompiledSolver
x = ca.SX.sym('x', 22)
p = ca.SX.sym('p', 394)
objective = ca.sumsqr(x - p[:22]) + p[23] * ca.sumsqr(x)
native = ca.nlpsol('solver', 'ipopt', {{'x': x, 'p': p, 'f': objective}},
                  {{'print_time': False, 'ipopt.print_level': 0}})
solver = CompiledSolver(native)
parameter = np.zeros(394)
parameter[23] = .5
angles = np.zeros(22)
for direction in (1., -1., 2.):
    parameter[:22] = direction * np.linspace(.1, 1., 22)
    result = solver(x0=angles, p=parameter, lam_x0=np.zeros(22))
    angles = np.asarray(result['x']).ravel()
    np.testing.assert_allclose(angles, parameter[:22] / 1.5, atol=1e-7)
for extra in ({{'lbx': np.zeros(22)}}, {{'x0': np.full(22, np.nan)}}):
    arguments = {{'x0': angles, 'p': parameter, **extra}}
    try:
        solver(**arguments)
    except (ValueError, RuntimeError):
        pass
    else:
        raise AssertionError('Invalid inputs produced a command')
"""
        result = subprocess.run(
            [str(PYTHON), '-I', '-c', script], text=True,
            capture_output=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
