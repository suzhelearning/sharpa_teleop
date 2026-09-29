import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

from ros2launch.api.api import parse_launch_arguments

spec = importlib.util.spec_from_file_location(
    'teleop_workspace_cli', Path(__file__).resolve().parents[1] / 'scripts/workspace.py')
workspace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(workspace)


class WorkspaceCliTests(unittest.TestCase):
    def test_hand_selection_is_accepted_by_real_ros_launch_parser(self):
        cases = (
            (['left'], {'left_serial': 'C55C9039C55F'}),
            (['right'], {'right_serial': 'CC549038CC57'}),
            ([], {'left_serial': 'C55C9039C55F', 'right_serial': 'CC549038CC57'}),
        )
        for selection, expected in cases:
            with self.subTest(selection=selection):
                with patch.object(sys, 'argv', ['workspace.py', 'real', *selection]), \
                        patch.object(workspace, 'ros_env', return_value={}), \
                        patch.object(workspace.os, 'execvpe') as launch:
                    workspace.main()
                # --show-args bypasses this parser, so exercise the actual launch boundary.
                options = dict(parse_launch_arguments(launch.call_args.args[1][4:]))
                selected = {key: value for key, value in options.items() if key.endswith('_serial')}
                self.assertEqual(selected, expected)


if __name__ == '__main__':
    unittest.main()
