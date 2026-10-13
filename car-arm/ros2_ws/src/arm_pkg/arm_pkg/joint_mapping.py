"""Validated mapping between STS encoder counts and URDF joint radians."""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


class MappingError(ValueError):
    pass


class JointMapping:
    def __init__(self, data: dict):
        if data.get('encoder_counts_per_revolution') != 4096:
            raise MappingError('expected 4096 encoder counts per revolution')
        self.counts_per_revolution = 4096
        joints = data.get('joints')
        if not isinstance(joints, list) or [j.get('id') for j in joints] != list(range(1, 7)):
            raise MappingError('expected joint IDs 1 through 6 in order')
        self.joints = {j['id']: j for j in joints}
        for servo_id, joint in self.joints.items():
            low, high = joint['software_count_limits']
            zero = joint['zero_count']
            sign = joint['positive_direction']
            if (type(zero) is not int or type(sign) is not int or sign not in (-1, 1) or
                    type(low) is not int or type(high) is not int or
                    not 0 <= low <= zero <= high <= 4095):
                raise MappingError(f'ID {servo_id} has invalid count mapping')
            angle_low, angle_high = joint['software_angle_limits_deg']
            if not angle_low <= 0 <= angle_high:
                raise MappingError(f'ID {servo_id} has invalid angle limits')
            endpoint_angles = [self.count_to_radians(servo_id, c) for c in (low, high)]
            if (abs(min(endpoint_angles) - math.radians(angle_low)) > 1e-6 or
                    abs(max(endpoint_angles) - math.radians(angle_high)) > 1e-6):
                raise MappingError(f'ID {servo_id} angle and count limits disagree')

    @classmethod
    def load(cls, path: str | Path) -> 'JointMapping':
        return cls(json.loads(Path(path).read_text(encoding='utf-8')))

    def joint(self, servo_id: int) -> dict:
        if type(servo_id) is not int or servo_id not in self.joints:
            raise MappingError(f'unknown servo ID {servo_id}')
        return self.joints[servo_id]

    def count_to_radians(self, servo_id: int, count: int) -> float:
        joint = self.joint(servo_id)
        if type(count) is not int or not 0 <= count <= 4095:
            raise MappingError(f'ID {servo_id} invalid encoder count {count}')
        return (count - joint['zero_count']) * joint['positive_direction'] * 2 * math.pi / self.counts_per_revolution

    def radians_to_count(self, servo_id: int, radians: float) -> int:
        joint = self.joint(servo_id)
        if isinstance(radians, bool) or not isinstance(radians, (int, float)) or not math.isfinite(radians):
            raise MappingError('target angle must be finite radians')
        low_deg, high_deg = joint['software_angle_limits_deg']
        if not math.radians(low_deg) - 1e-12 <= radians <= math.radians(high_deg) + 1e-12:
            raise MappingError(f'ID {servo_id} angle outside [{low_deg}, {high_deg}] degrees')
        delta = round(radians * self.counts_per_revolution / (2 * math.pi))
        count = joint['zero_count'] + joint['positive_direction'] * delta
        low, high = joint['software_count_limits']
        if not low <= count <= high:
            raise MappingError(f'ID {servo_id} target {count} outside software counts [{low}, {high}]')
        return count

    def validate_urdf(self, path: str | Path) -> None:
        root = ET.parse(path).getroot()
        urdf_joints = {j.attrib['name']: j for j in root.findall('joint')}
        for servo_id, joint in self.joints.items():
            name = joint['urdf_joint_name']
            if name not in urdf_joints:
                raise MappingError(f'ID {servo_id} URDF joint {name} missing')
            limit = urdf_joints[name].find('limit')
            lower = float(limit.attrib['lower'])
            upper = float(limit.attrib['upper'])
            if (abs(lower - math.radians(joint['software_angle_limits_deg'][0])) > 1e-6 or
                    abs(upper - math.radians(joint['software_angle_limits_deg'][1])) > 1e-6):
                raise MappingError(f'ID {servo_id} URDF and software angle limits differ')
