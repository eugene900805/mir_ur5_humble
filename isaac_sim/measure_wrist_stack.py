"""Measure flange-to-fingertip length from exported URDF collision geometry.

Run after regenerating mir_isaac.urdf. Requires numpy and trimesh. Values include
fixed wrist offsets and gripper mimic joints, measured along ur_tool0 +Z.
"""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


MASTER_JOINT = "robotiq_85_left_knuckle_joint"


def origin(element):
    if element is None:
        return np.eye(4)
    transform = trimesh.transformations.euler_matrix(
        *map(float, element.get("rpy", "0 0 0").split())
    )
    transform[:3, 3] = list(map(float, element.get("xyz", "0 0 0").split()))
    return transform


def link_transforms(joints, opening):
    transforms = {"ur_tool0": np.eye(4)}
    by_name = {joint.get("name"): joint for joint in joints}

    def position(joint):
        if joint.get("name") == MASTER_JOINT:
            return opening
        mimic = joint.find("mimic")
        if mimic is None:
            return 0.0
        return (
            position(by_name[mimic.get("joint")]) * float(mimic.get("multiplier", "1"))
            + float(mimic.get("offset", "0"))
        )

    pending = list(joints)
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in transforms:
                continue
            transform = origin(joint.find("origin"))
            if joint.get("type") in ("revolute", "continuous"):
                axis = list(map(float, joint.find("axis").get("xyz").split()))
                transform = transform @ trimesh.transformations.rotation_matrix(position(joint), axis)
            transforms[child] = transforms[parent] @ transform
            pending.remove(joint)
            progressed = True
        if not progressed:
            break  # The rest of the mobile robot is upstream of ur_tool0.
    return transforms


def geometry_points(geometry):
    mesh = geometry.find("mesh")
    if mesh is not None:
        loaded = trimesh.load(mesh.get("filename"), force="mesh")
        scale = np.array(list(map(float, mesh.get("scale", "1 1 1").split())))
        return loaded.vertices * scale
    box = geometry.find("box")
    if box is not None:
        return trimesh.creation.box(extents=list(map(float, box.get("size").split()))).vertices
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        return trimesh.creation.cylinder(
            radius=float(cylinder.get("radius")),
            height=float(cylinder.get("length")), sections=64,
        ).vertices
    raise ValueError(f"Unsupported geometry: {ET.tostring(geometry)}")


def measure(urdf):
    robot = ET.parse(urdf).getroot()
    joints = robot.findall("joint")
    vertices = {}
    for link in robot.findall("link"):
        name = link.get("name")
        if not name.startswith(("robotiq_ft", "robotiq_wrist", "robotiq_85")):
            continue
        vertices[name] = [
            trimesh.transform_points(geometry_points(c.find("geometry")), origin(c.find("origin")))
            for c in link.findall("collision")
        ]
    reference = link_transforms(joints, 0.0)
    sensor_z = reference["robotiq_ft300_sensor"][2, 3]
    gripper_z = reference["robotiq_85_base_link"][2, 3]
    print(f"FT 300 + coupling: {sensor_z * 1000:.2f} mm")
    print(f"Camera added height: {(gripper_z - sensor_z) * 1000:.2f} mm")
    for opening, label in ((0.0, "open"), (0.7929, "closed")):
        transforms = link_transforms(joints, opening)
        candidates = [
            (trimesh.transform_points(points, transforms[name])[:, 2].max(), name)
            for name, parts in vertices.items() for points in parts
        ]
        length, name = max(candidates)
        print(f"{label}: total={length * 1000:.2f} mm; "
              f"gripper={(length - gripper_z) * 1000:.2f} mm; "
              f"master={opening} rad; furthest={name}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path,
                        default=Path(__file__).resolve().parent / "mir_isaac.urdf")
    measure(parser.parse_args().urdf)
