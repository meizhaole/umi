import type { RobotConfig } from '../app/config';

export const configureRobotUrdf = (xml: string, config: RobotConfig): string => {
  if (config.tcpOffset === undefined && config.tcpParentLink === undefined) return xml;
  if (!config.tipLink || !config.tcpParentLink || config.tcpOffset === undefined) {
    throw new Error(`机器人 ${config.id} 的 TCP 固定变换配置不完整`);
  }
  if (!Number.isFinite(config.tcpOffset) || config.tipLink === config.tcpParentLink) {
    throw new Error(`机器人 ${config.id} 的 TCP 固定变换配置无效`);
  }

  const document = new DOMParser().parseFromString(xml, 'application/xml');
  if (document.querySelector('parsererror')) throw new Error('URDF XML 格式无效');
  const robot = document.documentElement;
  if (robot.localName !== 'robot') throw new Error('URDF 根元素必须是 robot');

  const links = Array.from(robot.children).filter((element) => element.localName === 'link');
  if (!links.some((link) => link.getAttribute('name') === config.tcpParentLink)) {
    throw new Error(`URDF 缺少 TCP 父 link：${config.tcpParentLink}`);
  }
  if (links.some((link) => link.getAttribute('name') === config.tipLink)) {
    throw new Error(`URDF 已存在配置的 TCP link：${config.tipLink}`);
  }

  const jointName = `${config.tcpParentLink}_to_${config.tipLink}`;
  if (
    Array.from(robot.children).some(
      (element) => element.localName === 'joint' && element.getAttribute('name') === jointName,
    )
  ) {
    throw new Error(`URDF 已存在 TCP joint：${jointName}`);
  }

  const link = document.createElement('link');
  link.setAttribute('name', config.tipLink);
  const joint = document.createElement('joint');
  joint.setAttribute('name', jointName);
  joint.setAttribute('type', 'fixed');
  const parent = document.createElement('parent');
  parent.setAttribute('link', config.tcpParentLink);
  const child = document.createElement('child');
  child.setAttribute('link', config.tipLink);
  const origin = document.createElement('origin');
  origin.setAttribute('xyz', `0 0 ${config.tcpOffset}`);
  joint.append(parent, child, origin);
  robot.append(link, joint);

  return new XMLSerializer().serializeToString(document);
};
