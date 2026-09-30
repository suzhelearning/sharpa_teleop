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



if __name__ == '__main__':
    unittest.main()
