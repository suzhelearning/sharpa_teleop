"""Load the vendor's dual-hand dynamics without modifying or copying its assets."""
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

from .safety import JointModel, load_joint_model

SIDES = ("left", "right")


class HandSimulation:
    def __init__(self, models_root: str):
        if not models_root:
            raise ValueError("models_root is required; set SHARPA_MODELS")
        wave = Path(models_root).expanduser().resolve() / "wave_01"
        scene_path = wave / "dual_sharpa_wave/dual_sharpa_wave.xml"
        tree = ET.parse(scene_path)
        scene = tree.getroot()
        self.joints = {
            side: load_joint_model(wave / f"{side}_sharpa_wave/{side}_sharpa_wave.urdf", side)
            for side in SIDES
        }
        controlled = {name for model in self.joints.values() for name in model.names}

        # The supplied dual XML uses wave_01-relative mesh paths, rather than
        # XML-file-relative paths. Resolve them in memory, keeping assets external.
        compiler = scene.find("compiler")
        if compiler is not None:
            compiler.set("meshdir", "")
        for mesh in scene.findall("asset/mesh"):
            mesh_path = wave / mesh.attrib["file"]
            if not mesh_path.is_file():
                raise FileNotFoundError(f"Missing vendor mesh: {mesh_path}")
            mesh.set("file", str(mesh_path))

        # Manus currently supplies finger poses only, not wrist/head tracking.
        # Fix the extra 18 base/head DoFs at the vendor's reference placement.
        for body in scene.iter("body"):
            for joint in list(body.findall("joint")):
                if joint.get("name") not in controlled:
                    body.remove(joint)
        actuators = scene.find("actuator")
        if actuators is None:
            raise ValueError("Vendor XML has no position actuators")
        for actuator in list(actuators):
            if actuator.get("joint") not in controlled:
                actuators.remove(actuator)
            elif actuator.tag != "position":
                raise ValueError("Simulation requires vendor position actuators")
        option = scene.find("option")
        if option is None:
            option = ET.SubElement(scene, "option")
        option.set("timestep", "0.002")
        option.set("integrator", "implicitfast")
        world = scene.find("worldbody")
        ET.SubElement(world, "light", pos="1 -1 2", dir="-1 1 -2", diffuse="0.8 0.8 0.8")
        ET.SubElement(world, "geom", name="sim_floor", type="plane", pos="0 0 -0.035",
                      size="1 1 0.01", rgba="0.18 0.21 0.25 1")
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        self.qpos_ids = {}
        self.dof_ids = {}
        self.actuator_ids = {}
        for side, joint_model in self.joints.items():
            joint_ids = np.array([self.model.joint(name).id for name in joint_model.names])
            actuator_ids = []
            for joint_id in joint_ids:
                matches = np.flatnonzero(
                    (self.model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT)
                    & (self.model.actuator_trnid[:, 0] == joint_id))
                if len(matches) != 1:
                    raise ValueError("Each finger joint must have exactly one position actuator")
                actuator_ids.append(int(matches[0]))
            self.qpos_ids[side] = self.model.jnt_qposadr[joint_ids]
            self.dof_ids[side] = self.model.jnt_dofadr[joint_ids]
            self.actuator_ids[side] = np.array(actuator_ids)
            # Enforce the intersection of URDF, XML joint and actuator limits.
            lower = np.maximum(joint_model.lower, self.model.jnt_range[joint_ids, 0])
            upper = np.minimum(joint_model.upper, self.model.jnt_range[joint_ids, 1])
            lower = np.maximum(lower, self.model.actuator_ctrlrange[actuator_ids, 0])
            upper = np.minimum(upper, self.model.actuator_ctrlrange[actuator_ids, 1])
            self.joints[side] = JointModel(joint_model.names, tuple(lower), tuple(upper))
            initial = np.clip(self.data.qpos[self.qpos_ids[side]], lower, upper)
            self.data.qpos[self.qpos_ids[side]] = initial
            self.data.ctrl[self.actuator_ids[side]] = initial
        mujoco.mj_forward(self.model, self.data)

    def command(self, side, names, positions):
        positions, reason = self.joints[side].validate(names, positions)
        if positions is None:
            raise ValueError(reason)
        self.data.ctrl[self.actuator_ids[side]] = positions

    def hold(self, side):
        joints = self.joints[side]
        self.data.ctrl[self.actuator_ids[side]] = np.clip(
            self.data.qpos[self.qpos_ids[side]], joints.lower, joints.upper)

    def step(self):
        previous_time = self.data.time
        mujoco.mj_step(self.model, self.data)
        if self.data.time <= previous_time or not np.isfinite(self.data.qpos).all():
            raise RuntimeError("MuJoCo state diverged or reset; stopping simulation")

    def feedback(self, side):
        return (self.data.qpos[self.qpos_ids[side]].tolist(),
                self.data.qvel[self.dof_ids[side]].tolist())

    @staticmethod
    def configure_camera(camera):
        camera.lookat[:] = [0.0, 0.0, 0.11]
        camera.distance = 0.75
        camera.azimuth = 0.0
        camera.elevation = -15.0
