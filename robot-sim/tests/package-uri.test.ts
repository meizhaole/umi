import { describe, expect, it } from 'vitest';
import { resolvePackageUri, resolvePackageUris } from '../src/utils/resolvePackageUri.mjs';

describe('package URI resolver', () => {
  const packageMappings = {
    rebotarm_bringup: '/robot',
    ur_description: '/robot/ur_description',
  };

  it.each([
    [
      'package://rebotarm_bringup/description/RS/meshes/base.STL',
      '/robot/description/RS/meshes/base.STL',
    ],
    [
      'package://ur_description/meshes/ur5/visual/base.dae',
      '/robot/ur_description/meshes/ur5/visual/base.dae',
    ],
  ])('解析 %s', (uri, expected) => {
    expect(resolvePackageUri(uri, packageMappings)).toBe(expected);
  });

  it('解析 URDF 中的所有 package URI', () => {
    const xml = '<mesh filename="package://ur_description/meshes/ur5/collision/base.stl" />';

    expect(resolvePackageUris(xml, packageMappings)).toBe(
      '<mesh filename="/robot/ur_description/meshes/ur5/collision/base.stl" />',
    );
  });

  it('拒绝未配置包和路径穿越', () => {
    expect(() => resolvePackageUri('package://missing/file.stl', packageMappings)).toThrow();
    expect(() =>
      resolvePackageUri('package://ur_description/../private/file.stl', packageMappings),
    ).toThrow();
  });
});
