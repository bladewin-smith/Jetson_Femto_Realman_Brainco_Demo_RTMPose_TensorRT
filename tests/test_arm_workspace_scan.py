import ast
import ctypes
import pathlib
import types
import unittest

import numpy as np


SOURCE_PATH = (
    pathlib.Path(__file__).resolve().parents[1]
    / "scripts"
    / "camera_arm_rm65b_control_pose_hands.py"
)


def load_arm_workspace_methods():
    tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"), filename=str(SOURCE_PATH))
    arm_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "RM65PalmFollower"
    )
    wanted = {
        "_limit_per_axis_bounds",
        "_scan_axis_direction",
        "_sdk_check_joint_position_limit",
        "_update_effective_workspace_limits",
    }
    methods = [
        node
        for node in arm_class.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name in wanted
    ]
    harness = ast.ClassDef(
        name="WorkspaceHarness",
        bases=[],
        keywords=[],
        body=methods,
        decorator_list=[],
    )
    module = ast.fix_missing_locations(ast.Module(body=[harness], type_ignores=[]))
    namespace = {"ctypes": ctypes, "np": np}
    exec(compile(module, str(SOURCE_PATH), "exec"), namespace)
    return namespace["WorkspaceHarness"]


WorkspaceHarness = load_arm_workspace_methods()


class ArmWorkspaceScanTests(unittest.TestCase):
    def test_asymmetric_axis_limits_are_independent(self):
        result = WorkspaceHarness._limit_per_axis_bounds(
            [0.08, -0.08, 0.03],
            [0.02, 0.04, 0.01],
            [0.05, 0.06, 0.02],
        )
        np.testing.assert_allclose(result, [0.05, -0.04, 0.02])

    def test_scan_refines_first_unsafe_boundary_without_motion(self):
        harness = WorkspaceHarness()
        harness.workspace_scan_max_distance_m = np.array([0.20, 0.20, 0.20])
        harness.workspace_scan_step_m = 0.05
        harness.workspace_scan_refine_iterations = 8
        base_pose = [0.10, 0.20, 0.30, 0.0, 0.0, 0.0]
        boundary = 0.123

        def solve(_self, pose, seed_joints):
            distance = abs(float(pose[0]) - base_pose[0])
            if distance <= boundary:
                return True, list(seed_joints), "safe"
            return False, list(seed_joints), "joint_margin"

        harness._workspace_pose_solution = types.MethodType(solve, harness)
        distance, reason = harness._scan_axis_direction(
            base_pose,
            [0.0] * 6,
            axis_index=0,
            direction=1.0,
        )
        self.assertLessEqual(distance, boundary)
        self.assertLess(boundary - distance, 0.001)
        self.assertEqual(reason, "joint_margin")

    def test_configured_cap_is_applied_after_scanned_limits(self):
        harness = WorkspaceHarness()
        harness.max_arm_axis_offset_m = np.array([0.04, 0.08, 0.04])
        harness.workspace_scanned_negative_axis_offset_m = np.array(
            [0.03, 0.09, 0.02]
        )
        harness.workspace_scanned_positive_axis_offset_m = np.array(
            [0.05, 0.06, 0.03]
        )
        harness._update_effective_workspace_limits()
        np.testing.assert_allclose(
            harness.workspace_negative_axis_offset_m,
            [0.03, 0.08, 0.02],
        )
        np.testing.assert_allclose(
            harness.workspace_positive_axis_offset_m,
            [0.04, 0.06, 0.03],
        )

    def test_sdk_joint_limit_helper_falls_back_to_ctypes_pointer(self):
        class Logger:
            def info(self, _message):
                pass

        class PointerOnlyRobot:
            def __init__(self):
                self.list_calls = 0
                self.pointer_calls = 0

            def rm_algo_ikine_check_joint_position_limit(self, values):
                if isinstance(values, list):
                    self.list_calls += 1
                    raise TypeError("expected LP_c_float instance instead of list")
                self.pointer_calls += 1
                self.last_values = list(values)
                return 0

        harness = WorkspaceHarness()
        harness.robot = PointerOnlyRobot()
        harness.logger = Logger()
        harness.sdk_joint_limit_argument_mode = None
        harness._pad_joints_for_sdk = lambda joints: list(joints) + [0.0]

        self.assertEqual(
            harness._sdk_check_joint_position_limit([1, 2, 3, 4, 5, 6]),
            0,
        )
        self.assertEqual(harness.sdk_joint_limit_argument_mode, "ctypes")
        self.assertEqual(harness.robot.list_calls, 1)
        self.assertEqual(harness.robot.pointer_calls, 1)
        np.testing.assert_allclose(harness.robot.last_values, [1, 2, 3, 4, 5, 6, 0])

        self.assertEqual(
            harness._sdk_check_joint_position_limit([6, 5, 4, 3, 2, 1]),
            0,
        )
        self.assertEqual(harness.robot.list_calls, 1)
        self.assertEqual(harness.robot.pointer_calls, 2)


if __name__ == "__main__":
    unittest.main()
