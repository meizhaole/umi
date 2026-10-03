import { describe, expect, it } from 'vitest';
import { Quaternion, Vector3 } from 'three';
import type { InertialDescription } from '../src/core/types';
import { getRapierMassProperties } from '../src/utils/urdfInertialProperties';

describe('URDF 惯性转 Rapier 质量属性', () => {
  it('保留质量、质心和非对角惯性张量的主惯量', () => {
    const inertial: InertialDescription = {
      origin: {
        position: [0.1, 0.2, 0.3],
        orientation: [0, 0, 0, 1],
      },
      mass: 2,
      inertia: { ixx: 2, ixy: 1, ixz: 0, iyy: 2, iyz: 0, izz: 4 },
    };
    const properties = getRapierMassProperties(inertial, new Quaternion());
    const expectedPrincipalFrame = new Quaternion().setFromAxisAngle(
      new Vector3(0, 0, 1),
      -Math.PI / 4,
    );

    expect(properties.mass).toBe(2);
    expect(properties.centerOfMass.toArray()).toEqual([0.1, 0.2, 0.3]);
    expect(properties.principalAngularInertia.toArray()).toEqual([1, 3, 4]);
    expect(Math.abs(properties.angularInertiaLocalFrame.dot(expectedPrincipalFrame))).toBeCloseTo(
      1,
    );
  });

  it('把 URDF 惯性原点变换到物理刚体坐标系', () => {
    const inertial: InertialDescription = {
      origin: {
        position: [1, 2, 3],
        orientation: [0, 0, Math.SQRT1_2, Math.SQRT1_2],
      },
      mass: 4,
      inertia: { ixx: 1, ixy: 0, ixz: 0, iyy: 2, iyz: 0, izz: 3 },
    };
    const linkToBodyRotation = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), Math.PI / 2);
    const properties = getRapierMassProperties(inertial, linkToBodyRotation);
    const expectedCenter = new Vector3(1, 2, 3).applyQuaternion(
      linkToBodyRotation.clone().invert(),
    );
    const expectedFrame = linkToBodyRotation
      .clone()
      .invert()
      .multiply(new Quaternion(...inertial.origin.orientation));

    expect(properties.centerOfMass.distanceTo(expectedCenter)).toBeLessThan(1e-12);
    expect(Math.abs(properties.angularInertiaLocalFrame.dot(expectedFrame))).toBeCloseTo(1);
  });
});
