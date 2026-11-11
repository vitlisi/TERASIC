#!/usr/bin/env python3
"""Audit user-supplied rounded matrices and the saved SLX, without MATLAB/hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def supplied_matrices():
    # User message, 2026-10-09. Physical state mapping NOT yet verified.
    A = np.array([
        [0, 1, 0, 0, 0, 0],
        [255.5775, 0, 0, 728.9427, 0, 728.9427],
        [0, 0, 0, 1, 0, 0],
        [-255.5775, 0, 0, -872.6072, 0, -728.9427],
        [0, 0, 0, 0, 0, 1],
        [-255.5775, 0, 0, -728.9427, 0, -872.6072],
    ], dtype=float)
    B = np.array([[0, 0], [-445.2858, -445.2858], [0, 0],
                  [445.2858, 533.0454], [0, 0], [533.0454, 445.2858]])
    C = np.array([[0, 0, .5, 0, .5, 0], [0, -.0098, 0, 0, 0, 0],
                  [0, 0, -.1946, 0, .1946, 0], [0]*6,
                  [0, -1, 0, 0, 0, 0], [0]*6])
    D = np.zeros((6, 2))
    D[1] = .4618e-15
    return A, B, C, D


def slx_facts(path):
    facts = {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    with zipfile.ZipFile(path) as archive:
        root = ET.fromstring(archive.read("simulink/systems/system_root.xml"))
        plant = ET.fromstring(archive.read("simulink/systems/system_141.xml"))
        points = []
        for block in root.findall("Block"):
            for port in block.findall("./PortProperties/Port"):
                for field in port.findall(".//Field[@Name='Type']"):
                    points.append({"block": block.attrib["Name"],
                                   "port": port.attrib["Index"], "type": field.text})
        facts["saved_analysis_points"] = points
        joints, contacts = [], []
        for block in plant.findall("Block"):
            props = {p.attrib.get("Name"): p.text for p in block.iter("P")}
            typ = props.get("SourceType", "").replace("\n", " ")
            info = {"name": block.attrib["Name"], "sid": block.attrib["SID"],
                    "type": typ, "commented": props.get("Commented", "off")}
            if typ in ["Revolute Joint", "Prismatic Joint"]:
                info.update({k: props.get(k) for k in ["DampingCoefficient",
                            "DampingCoefficientUnits", "MotionActuationMode",
                            "TorqueActuationMode"]})
                joints.append(info)
            if typ == "Spatial Contact Force":
                contacts.append(info)
        facts["joints"] = joints
        facts["contacts"] = contacts
        facts["estimator_code"] = next(
            p.text for p in ET.fromstring(archive.read("simulink/stateflow/chart_31.xml")).iter("P")
            if p.attrib.get("Name") == "script")
    return facts


def audit(path, matrix_data=None):
    if matrix_data is None:
        A, B, C, D = supplied_matrices()
    else:
        A, B, C, D = (np.asarray(matrix_data[key], dtype=float) for key in ['A','B','C','D'])
    if A.shape != (6,6) or B.ndim != 2 or B.shape[0] != 6 or B.shape[1] not in [1,2]:
        raise ValueError('Expected A 6x6 and B 6x1 or 6x2')
    if C.shape != (6,6) or D.shape != (6,B.shape[1]):
        raise ValueError('Expected C 6x6 and matching D')
    if not all(np.isfinite(m).all() for m in [A,B,C,D]):
        raise ValueError('Matrices must be finite')
    # Merely an algebraic symmetry decomposition, not a physical-state assertion.
    T = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0],
                  [0, 0, .5, 0, .5, 0], [0, 0, 0, .5, 0, .5],
                  [0, 0, -.5, 0, .5, 0], [0, 0, 0, -.5, 0, .5]])
    Am = T @ A @ np.linalg.inv(T)
    # For a single input B ALREADY contains both wheel contributions.
    Bcommon = B @ np.ones((B.shape[1], 1))
    Bm = T @ Bcommon
    poles = np.linalg.eigvals(A)
    pbh = [{"pole_real": float(lam.real), "pole_imag": float(lam.imag),
            "rank_supplied_inputs": int(np.linalg.matrix_rank(np.column_stack([lam*np.eye(6)-A, B]))),
            "rank_common_input": int(np.linalg.matrix_rank(np.column_stack([lam*np.eye(6)-A, Bcommon])))}
           for lam in poles]
    result = {"matrix_source": (matrix_data or {}).get('matrix_source',
                                           "Rounded values supplied in user message, 2026-10-09"),
            "state_mapping_verified": False, "hardware_gains_generated": False,
            "sample_period_s": .01, "A": A.tolist(), "B": B.tolist(),
            "C": C.tolist(), "D": D.tolist(), "C_rank": int(np.linalg.matrix_rank(C)),
            "modal_state_labels": ["q0", "v0", "mean_q12", "mean_v12", "diff_q12", "diff_v12"],
            "T": T.tolist(), "A_modal": Am.tolist(), "B_common_modal": Bm.tolist(),
            "pbh": pbh, "poles_continuous": poles.tolist(),
            "poles_exact_zoh_10ms": np.exp(.01*poles).tolist()}
    if matrix_data is not None:
        result['input_description'] = matrix_data.get('input_description', 'Unspecified')
    if path is not None:
        result['slx'] = slx_facts(path)
        result['damping_SI_if_current_deg_units'] = .0126*180/np.pi
    else:
        result['slx'] = None
        result['note'] = 'Updated SLX not available; no mechanical-model verification performed.'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--matrices", type=Path, help='JSON ABCD; absent SLX is not replaced with an old one')
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    matrix_data = json.loads(args.matrices.read_text()) if args.matrices else None
    model = args.model
    if model is None and args.matrices is None:
        model = ROOT / 'maintenance/matlab/Terasic_Simscape_linmod.slx'
    output = args.output or ROOT/'logs/lqr/design'/(
        args.matrices.stem+'_audit.json' if args.matrices else 'model_audit.json')
    result = audit(model, matrix_data)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print("Continuous poles:", result["poles_continuous"])
    if result['slx'] is not None:
        print("Effective damping with saved deg/s unit:", result["damping_SI_if_current_deg_units"])
        print("Saved input analysis points:", [p for p in result["slx"]["saved_analysis_points"] if p["type"] == "in"])
    print("Common input leaves differential modes unactuated; no hardware gains generated.")
    print("Report:", output)


if __name__ == "__main__":
    main()
