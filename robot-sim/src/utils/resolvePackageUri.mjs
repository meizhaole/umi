const PACKAGE_URI_PATTERN = /package:\/\/[^"'`\s<>]+/gu;
const PACKAGE_URI_PREFIX = /^package:\/\/([^/]+)\/(.+)$/u;

export const resolvePackageUri = (uri, packageMappings) => {
  const match = PACKAGE_URI_PREFIX.exec(uri);
  if (!match || !Object.hasOwn(packageMappings, match[1])) {
    throw new Error(`URDF 使用了未配置的资源 URI：${uri}`);
  }

  let decodedPath;
  try {
    decodedPath = decodeURIComponent(match[2]);
  } catch {
    throw new Error(`资源 URI 编码无效：${uri}`);
  }

  const segments = decodedPath.split('/');
  if (
    decodedPath.includes('\\') ||
    decodedPath.includes('\0') ||
    decodedPath.includes('?') ||
    decodedPath.includes('#') ||
    segments.some((segment) => segment === '' || segment === '.' || segment === '..')
  ) {
    throw new Error(`资源 URI 包含不安全的路径：${uri}`);
  }

  const root = packageMappings[match[1]].replace(/\/+$/u, '');
  return `${root}/${segments.join('/')}`;
};

export const resolvePackageUris = (urdfXml, packageMappings) =>
  urdfXml.replace(PACKAGE_URI_PATTERN, (uri) => resolvePackageUri(uri, packageMappings));
