from contextlib import redirect_stderr
import io
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from ros2launch.api.api import parse_launch_arguments

spec = importlib.util.spec_from_file_location(
    'teleop_workspace_cli', Path(__file__).resolve().parents[1] / 'scripts/workspace.py')
workspace = importlib.util.module_from_spec(spec)
spec.loader.exec_module(workspace)


class WorkspaceCliTests(unittest.TestCase):
    def test_manus_requires_operator_before_build_or_sdk_access(self):
        cases = (
            [],
            ["calibration_dir:=/some/calibration"],
            ["calibration_operator:="],
            ["calibration_operator:=   "],
            ["calibration_operator:=syz", "calibration_operator:="],
            ["with_client:=false"],
            ["--show-args"],
        )
        for arguments in cases:
            with self.subTest(arguments=arguments):
                with patch.object(sys, "argv", ["workspace.py", "manus", *arguments]), \
                        redirect_stderr(io.StringIO()), \
                        patch.object(workspace, "sdk_root", side_effect=AssertionError("SDK accessed without operator")), \
                        patch.object(workspace.subprocess, "run", side_effect=AssertionError("build before operator validation")):
                    with self.assertRaises(SystemExit) as error:
                        workspace.main()
                self.assertEqual(error.exception.code, 2)

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

    def test_operator_requires_its_own_complete_nonempty_pair(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for side in ("left", "right"):
                (root / f"Calibration_{side}.mcal").write_bytes(b"default calibration")
            operator = root / "syz"
            operator.mkdir()
            (operator / "syzLeftMetaglovePro.mcal").write_bytes(b"syz left")
            # Generic files must never stand in for a requested person's files.
            (operator / "Calibration_left.mcal").write_bytes(b"other left")
            (operator / "Calibration_right.mcal").write_bytes(b"other right")

            with self.assertRaises(ValueError):
                workspace._operator_calibration_dir("syz", root)
            (operator / "syzRightMetaglovePro.mcal").touch()
            with self.assertRaises(ValueError):
                workspace._operator_calibration_dir("syz", root)
            (operator / "syzRightMetaglovePro.mcal").write_bytes(b"syz right")
            self.assertEqual(workspace._operator_calibration_dir("syz", root), operator)

    def test_operator_cannot_select_parent_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary)
            root = parent / "calibration"
            root.mkdir()
            for side in ("Left", "Right"):
                (parent / f"..{side}MetaglovePro.mcal").write_bytes(b"parent calibration")
            with self.assertRaises(ValueError):
                workspace._operator_calibration_dir("..", root)

    def test_each_operator_uses_its_matching_filename_prefix(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name in ("syz", "zr", "sch", "dmp"):
                with self.subTest(operator=name):
                    directory = root / name
                    directory.mkdir()
                    for side in ("Left", "Right"):
                        (directory / f"{name}{side}MetaglovePro.mcal").write_bytes(name.encode())
                    self.assertEqual(workspace._operator_calibration_dir(name, root), directory)


if __name__ == '__main__':
    unittest.main()
