"""Static checks for the calibrated Stonefish BlueROV2 Heavy model."""

import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
MODEL_PATH = REPO_ROOT / 'sim/world/data/robot/bluerov2_unphy.scn'
WORLD_PATH = REPO_ROOT / 'sim/world/scenario/waterlinked.scn'
TEXTURE_PATH = REPO_ROOT / 'sim/world/data/texture/br2.png'
TEXTURE_SHA256 = '13bccf98707b25e10a22933f25c1779a1003ef2b17a38b1995844c709abfe018'


def _model_root():
    return ET.parse(MODEL_PATH).getroot()


def test_visual_mesh_uses_its_unrotated_texture_atlas():
    model = _model_root()
    look = model.find("./looks/look[@name='br2']")

    assert look.attrib['texture'] == 'texture/br2.png'
    # The OBJ UVs and atlas orientation are a pair. A 90-degree atlas rotation
    # still loads successfully but produces black/cyan patches over the model.
    assert hashlib.sha256(TEXTURE_PATH.read_bytes()).hexdigest() == TEXTURE_SHA256


def test_waterlinked_proxy_has_no_visible_servo_placeholder():
    model = _model_root()

    assert model.find(".//link[@name='sonar_link']") is None
    assert model.find(".//actuator[@name='Servo']") is None
    assert model.find(".//sensor[@name='Dcam']/link").attrib['name'] == 'base_link'


def test_heavy_has_eight_balanced_t200_thrusters():
    actuators = _model_root().findall(".//actuator[@type='thruster']")
    assert len(actuators) == 8

    handedness = []
    for actuator in actuators:
        specs = actuator.find('specs')
        propeller = actuator.find('propeller')
        mesh_name = Path(propeller.find('mesh').attrib['filename']).stem
        is_right = propeller.attrib['right'] == 'true'
        is_inverted = specs.attrib['inverted_setpoint'] == 'true'
        handedness.append(is_right)

        assert float(propeller.attrib['diameter']) == pytest.approx(0.076)
        assert float(specs.attrib['max_setpoint']) == pytest.approx(
            3600.0 * 2.0 * math.pi / 60.0, rel=2e-6)
        # Inversion preserves one mixer force convention while handedness gives
        # CW/CCW propellers opposite reaction torques.
        assert is_right == (mesh_name == 'cw')
        assert is_inverted == (mesh_name == 'ccw')

    assert handedness.count(True) == 4
    assert handedness.count(False) == 4


def test_t200_coefficients_reproduce_published_16v_static_thrust():
    actuator = _model_root().find(".//actuator[@type='thruster']")
    specs = actuator.find('specs')
    propeller = actuator.find('propeller')
    coeff = actuator.find('thrust_model/thrust_coeff')
    world = ET.parse(WORLD_PATH).getroot()
    density = float(world.find('./environment/ocean/water').attrib['density'])

    omega = float(specs.attrib['max_setpoint'])
    diameter = float(propeller.attrib['diameter'])
    revolutions_per_second = omega / (2.0 * math.pi)
    scale = density * diameter ** 4 * revolutions_per_second ** 2

    assert scale * float(coeff.attrib['forward']) == pytest.approx(
        5.25 * 9.80665, rel=2e-4)
    assert scale * float(coeff.attrib['reverse']) == pytest.approx(
        4.10 * 9.80665, rel=2e-4)


def test_compound_vehicle_mass_and_materials_are_explicit():
    model = _model_root()
    base = model.find(".//base_link[@name='base_link']")
    parts = base.findall('./external_part') + base.findall('./internal_part')
    assert sum(float(part.find('mass').attrib['value']) for part in parts) \
        == pytest.approx(11.5)

    world_materials = {
        material.attrib['name']
        for material in ET.parse(WORLD_PATH).getroot().findall('./materials/material')
    }
    referenced_materials = {
        material.attrib['name']
        for material in model.findall('.//material')
    }
    assert referenced_materials <= world_materials
