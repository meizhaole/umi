import type {
  CollisionDescription,
  GeometryDescription,
  InertialDescription,
  JointDescription,
  JointType,
  LinkDescription,
  Pose,
  RobotDescription,
  Vec3,
  VisualDescription,
} from '../core/types';
import { IDENTITY_POSE, quaternionFromEuler } from './math';

const childrenNamed = (element: Element, name: string): Element[] =>
  Array.from(element.children).filter((child) => child.localName === name);

const childNamed = (element: Element, name: string): Element | undefined =>
  childrenNamed(element, name)[0];

const parseNumber = (value: string | null, fallback?: number): number => {
  if (value === null || value.trim() === '') {
    if (fallback !== undefined) return fallback;
    throw new Error('URDF 数值属性缺失');
  }
  const number = Number(value);
  if (!Number.isFinite(number)) throw new Error(`URDF 数值无效：${value}`);
  return number;
};

const parseVector = (value: string | null, fallback: Vec3 = [0, 0, 0]): Vec3 => {
  if (value === null || value.trim() === '') return [...fallback];
  const values = value.trim().split(/\s+/).map(Number);
  if (values.length !== 3 || values.some((number) => !Number.isFinite(number))) {
    throw new Error(`URDF 三维向量无效：${value}`);
  }
  return values as Vec3;
};

const parsePose = (element?: Element): Pose => {
  if (!element) return { ...IDENTITY_POSE, position: [...IDENTITY_POSE.position] };
  const position = parseVector(element.getAttribute('xyz'));
  const [roll, pitch, yaw] = parseVector(element.getAttribute('rpy'));
  return { position, orientation: quaternionFromEuler(roll, pitch, yaw) };
};

const parseGeometry = (element: Element): GeometryDescription => {
  const geometry = childNamed(element, 'geometry');
  const shape = geometry?.children[0];
  if (!shape) throw new Error('URDF visual/collision 缺少 geometry');

  switch (shape.localName) {
    case 'mesh': {
      const filename = shape.getAttribute('filename');
      if (!filename) throw new Error('URDF mesh 缺少 filename');
      return {
        type: 'mesh',
        filename,
        scale: parseVector(shape.getAttribute('scale'), [1, 1, 1]),
      };
    }
    case 'box':
      return { type: 'box', size: parseVector(shape.getAttribute('size')) };
    case 'cylinder':
      return {
        type: 'cylinder',
        radius: parseNumber(shape.getAttribute('radius')),
        length: parseNumber(shape.getAttribute('length')),
      };
    case 'sphere':
      return { type: 'sphere', radius: parseNumber(shape.getAttribute('radius')) };
    default:
      throw new Error(`不支持的 URDF geometry：${shape.localName}`);
  }
};

const parseVisual = (
  element: Element,
  materials: Map<string, [number, number, number, number]>,
): VisualDescription => {
  const material = childNamed(element, 'material');
  const materialName = material?.getAttribute('name') ?? undefined;
  const color = material ? childNamed(material, 'color')?.getAttribute('rgba') : null;
  const inlineColor = color ? color.trim().split(/\s+/).map(Number) : undefined;
  const resolvedColor =
    inlineColor?.length === 4 && inlineColor.every(Number.isFinite)
      ? (inlineColor as [number, number, number, number])
      : materialName
        ? materials.get(materialName)
        : undefined;

  return {
    name: element.getAttribute('name') ?? undefined,
    origin: parsePose(childNamed(element, 'origin')),
    geometry: parseGeometry(element),
    materialName,
    color: resolvedColor,
  };
};

const parseCollision = (element: Element): CollisionDescription => ({
  name: element.getAttribute('name') ?? undefined,
  origin: parsePose(childNamed(element, 'origin')),
  geometry: parseGeometry(element),
});

const parseInertial = (element: Element): InertialDescription => {
  const mass = childNamed(element, 'mass');
  const inertia = childNamed(element, 'inertia');
  if (!mass || !inertia) throw new Error('URDF inertial 缺少 mass 或 inertia');

  return {
    origin: parsePose(childNamed(element, 'origin')),
    mass: parseNumber(mass.getAttribute('value')),
    inertia: {
      ixx: parseNumber(inertia.getAttribute('ixx')),
      ixy: parseNumber(inertia.getAttribute('ixy')),
      ixz: parseNumber(inertia.getAttribute('ixz')),
      iyy: parseNumber(inertia.getAttribute('iyy')),
      iyz: parseNumber(inertia.getAttribute('iyz')),
      izz: parseNumber(inertia.getAttribute('izz')),
    },
  };
};

const parseJointType = (value: string | null): JointType => {
  const supportedTypes: JointType[] = [
    'fixed',
    'revolute',
    'continuous',
    'prismatic',
    'floating',
    'planar',
  ];
  if (!value || !supportedTypes.includes(value as JointType)) {
    throw new Error(`URDF joint 类型无效：${value ?? '缺失'}`);
  }
  return value as JointType;
};

const parseJoint = (element: Element): JointDescription => {
  const name = element.getAttribute('name');
  const parent = childNamed(element, 'parent')?.getAttribute('link');
  const child = childNamed(element, 'child')?.getAttribute('link');
  if (!name || !parent || !child) throw new Error('URDF joint 缺少 name、parent 或 child');

  const limitElement = childNamed(element, 'limit');
  const limit = limitElement
    ? {
        lower: limitElement.hasAttribute('lower')
          ? parseNumber(limitElement.getAttribute('lower'))
          : undefined,
        upper: limitElement.hasAttribute('upper')
          ? parseNumber(limitElement.getAttribute('upper'))
          : undefined,
        effort: limitElement.hasAttribute('effort')
          ? parseNumber(limitElement.getAttribute('effort'))
          : undefined,
        velocity: limitElement.hasAttribute('velocity')
          ? parseNumber(limitElement.getAttribute('velocity'))
          : undefined,
      }
    : undefined;

  const mimicElement = childNamed(element, 'mimic');
  const mimicJoint = mimicElement?.getAttribute('joint');
  const mimic =
    mimicElement && mimicJoint
      ? {
          joint: mimicJoint,
          multiplier: parseNumber(mimicElement.getAttribute('multiplier'), 1),
          offset: parseNumber(mimicElement.getAttribute('offset'), 0),
        }
      : undefined;

  return {
    name,
    type: parseJointType(element.getAttribute('type')),
    parent,
    child,
    origin: parsePose(childNamed(element, 'origin')),
    axis: parseVector(childNamed(element, 'axis')?.getAttribute('xyz') ?? null, [1, 0, 0]),
    limit,
    mimic,
  };
};

const findRootLink = (links: LinkDescription[], joints: JointDescription[]): string => {
  const children = new Set(joints.map((joint) => joint.child));
  const root = links.find((link) => !children.has(link.name));
  if (!root) throw new Error('URDF 没有根连杆');
  return root.name;
};

const findTipLink = (
  links: LinkDescription[],
  joints: JointDescription[],
  rootLink: string,
): string => {
  const parents = new Set(joints.map((joint) => joint.parent));
  const leaves = links.filter((link) => !parents.has(link.name)).map((link) => link.name);
  if (leaves.length === 1) return leaves[0];

  const parentJointByChild = new Map(joints.map((joint) => [joint.child, joint]));
  const getPath = (linkName: string): string[] => {
    const path = [linkName];
    let current = linkName;
    while (current !== rootLink) {
      const parentJoint = parentJointByChild.get(current);
      if (!parentJoint) break;
      current = parentJoint.parent;
      path.unshift(current);
    }
    return path;
  };

  const paths = leaves.map(getPath);
  const sharedPath: string[] = [];
  for (let index = 0; index < Math.min(...paths.map((path) => path.length)); index += 1) {
    const candidate = paths[0][index];
    if (!paths.every((path) => path[index] === candidate)) break;
    sharedPath.push(candidate);
  }
  return sharedPath.at(-1) && sharedPath.at(-1) !== rootLink
    ? (sharedPath.at(-1) as string)
    : (paths
        .reduce((longest, path) => (path.length > longest.length ? path : longest), [rootLink])
        .at(-1) as string);
};

export const parseUrdf = (xml: string, options: { tipLink?: string } = {}): RobotDescription => {
  if (typeof DOMParser === 'undefined') throw new Error('parseUrdf 需要浏览器 DOMParser');
  const document = new DOMParser().parseFromString(xml, 'application/xml');
  if (document.querySelector('parsererror')) throw new Error('URDF XML 格式无效');

  const robot = document.documentElement;
  if (robot.localName !== 'robot') throw new Error('URDF 根元素必须是 robot');

  const materials = new Map<string, [number, number, number, number]>();
  childrenNamed(robot, 'material').forEach((material) => {
    const name = material.getAttribute('name');
    const rgba = childNamed(material, 'color')?.getAttribute('rgba');
    if (!name || !rgba) return;
    const values = rgba.trim().split(/\s+/).map(Number);
    if (values.length === 4 && values.every(Number.isFinite)) {
      materials.set(name, values as [number, number, number, number]);
    }
  });

  const links = childrenNamed(robot, 'link').map((element): LinkDescription => ({
    name: element.getAttribute('name') ?? '',
    inertial: childNamed(element, 'inertial')
      ? parseInertial(childNamed(element, 'inertial') as Element)
      : undefined,
    visuals: childrenNamed(element, 'visual').map((visual) => parseVisual(visual, materials)),
    collisions: childrenNamed(element, 'collision').map(parseCollision),
  }));
  if (links.some((link) => !link.name)) throw new Error('URDF link 缺少 name');

  const joints = childrenNamed(robot, 'joint').map(parseJoint);
  const linkNames = new Set(links.map((link) => link.name));
  joints.forEach((joint) => {
    if (!linkNames.has(joint.parent) || !linkNames.has(joint.child)) {
      throw new Error(`URDF joint ${joint.name} 引用了不存在的 link`);
    }
  });
  const jointNames = new Set(joints.map((joint) => joint.name));
  joints.forEach((joint) => {
    if (joint.mimic && !jointNames.has(joint.mimic.joint)) {
      throw new Error(`URDF joint ${joint.name} mimic 引用了不存在的 joint`);
    }
  });

  const rootLink = findRootLink(links, joints);
  const tipLink = options.tipLink ?? findTipLink(links, joints, rootLink);
  if (!linkNames.has(tipLink)) throw new Error(`URDF tipLink 不存在：${tipLink}`);
  return {
    name: robot.getAttribute('name') ?? 'robot',
    rootLink,
    tipLink,
    links,
    joints,
  };
};
