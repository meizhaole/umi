// 同步 RS、DM URDF 实际引用的 STL 到 Vite 静态目录。
import { constants } from 'node:fs';
import { copyFile, lstat, mkdir, readFile, realpath, rename, rm } from 'node:fs/promises';
import { randomUUID } from 'node:crypto';
import { basename, extname, isAbsolute, relative, resolve, sep } from 'node:path';
import { fileURLToPath } from 'node:url';

const SCRIPT_DIRECTORY = fileURLToPath(new URL('.', import.meta.url));
const PROJECT_DIRECTORY = resolve(SCRIPT_DIRECTORY, '..');
const ROS_PACKAGE_DIRECTORY = resolve(
  PROJECT_DIRECTORY,
  '../reBotArmController_ROS2/src/rebotarm_bringup',
);
const ROBOT_ASSET_DIRECTORY = resolve(PROJECT_DIRECTORY, 'public/robot');
const URDF_MODELS = [
  { name: 'RS', file: 'description/RS/urdf/ReBot_Arm_RS.urdf' },
  { name: 'DM', file: 'description/DM/urdf/ReBot_Arm_DM.urdf' },
];
const PACKAGE_NAME = 'rebotarm_bringup';
const PACKAGE_URI_PATTERN = /package:\/\/[^"'`\s<>]+/gu;

function isPathInside(parent, candidate) {
  const pathFromParent = relative(parent, candidate);

  return (
    pathFromParent !== '' &&
    pathFromParent !== '..' &&
    !pathFromParent.startsWith(`..${sep}`) &&
    !isAbsolute(pathFromParent)
  );
}

function assetPathFromUri(uri, modelName) {
  const match = /^package:\/\/([^/]+)\/(.+)$/u.exec(uri);

  if (!match || match[1] !== PACKAGE_NAME) {
    throw new Error(`URDF 使用了不支持的资源 URI：${uri}`);
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

  if (segments[0] !== 'description' || segments[1] !== modelName) {
    throw new Error(`资源 URI 未指向 ${modelName} 描述目录：${uri}`);
  }

  if (extname(segments.at(-1)).toLowerCase() !== '.stl') {
    throw new Error(`当前资源同步仅支持 STL 网格：${uri}`);
  }

  return segments;
}

async function ensureDirectory(parent, segments) {
  let currentDirectory = parent;

  for (const segment of segments) {
    currentDirectory = resolve(currentDirectory, segment);
    if (!isPathInside(parent, currentDirectory)) {
      throw new Error(`目标目录超出允许范围：${currentDirectory}`);
    }

    try {
      const info = await lstat(currentDirectory);
      if (info.isSymbolicLink() || !info.isDirectory()) {
        throw new Error(`目标路径不是普通目录：${currentDirectory}`);
      }
    } catch (error) {
      if (error.code !== 'ENOENT') {
        throw error;
      }
      await mkdir(currentDirectory);
    }
  }

  return currentDirectory;
}

async function checkedSourcePath(segments) {
  const sourcePath = resolve(ROS_PACKAGE_DIRECTORY, ...segments);
  if (!isPathInside(ROS_PACKAGE_DIRECTORY, sourcePath)) {
    throw new Error(`源资源超出 ROS 包目录：${segments.join('/')}`);
  }

  let resolvedSource;
  try {
    resolvedSource = await realpath(sourcePath);
  } catch (error) {
    if (error.code === 'ENOENT') {
      throw new Error(`找不到 URDF 引用的资源：${sourcePath}`);
    }
    throw error;
  }

  const resolvedPackage = await realpath(ROS_PACKAGE_DIRECTORY);
  if (!isPathInside(resolvedPackage, resolvedSource)) {
    throw new Error(`源资源通过符号链接离开 ROS 包目录：${sourcePath}`);
  }

  const info = await lstat(resolvedSource);
  if (!info.isFile()) {
    throw new Error(`URDF 引用的资源不是普通文件：${sourcePath}`);
  }

  return resolvedSource;
}

async function copySafely(sourcePath, relativeDestination) {
  const segments = relativeDestination.split('/');
  const destinationName = segments.pop();
  const destinationDirectory = await ensureDirectory(ROBOT_ASSET_DIRECTORY, segments);
  const destinationPath = resolve(destinationDirectory, destinationName);

  if (!isPathInside(ROBOT_ASSET_DIRECTORY, destinationPath)) {
    throw new Error(`目标文件超出静态资源目录：${destinationPath}`);
  }

  try {
    const existing = await lstat(destinationPath);
    if (existing.isSymbolicLink() || !existing.isFile()) {
      throw new Error(`拒绝覆盖非普通文件：${destinationPath}`);
    }
  } catch (error) {
    if (error.code !== 'ENOENT') {
      throw error;
    }
  }

  const temporaryPath = resolve(
    destinationDirectory,
    `.${basename(destinationName)}.${randomUUID()}.tmp`,
  );

  try {
    await copyFile(sourcePath, temporaryPath, constants.COPYFILE_EXCL);
    await rename(temporaryPath, destinationPath);
  } catch (error) {
    await rm(temporaryPath, { force: true });
    throw error;
  }
}

async function buildCopyPlan() {
  const copyPlan = new Map();

  for (const model of URDF_MODELS) {
    const urdfSegments = model.file.split('/');
    const urdfSource = await checkedSourcePath(urdfSegments);
    const urdfContents = await readFile(urdfSource, 'utf8');
    copyPlan.set(model.file, urdfSource);

    for (const match of urdfContents.matchAll(PACKAGE_URI_PATTERN)) {
      const assetSegments = assetPathFromUri(match[0], model.name);
      const assetDestination = assetSegments.join('/');
      if (!copyPlan.has(assetDestination)) {
        copyPlan.set(assetDestination, await checkedSourcePath(assetSegments));
      }
    }
  }

  return copyPlan;
}

function color(text, code) {
  if (!process.stdout.isTTY || process.env.NO_COLOR) {
    return text;
  }
  return `\u001b[${code}m${text}\u001b[0m`;
}

async function main() {
  const copyPlan = await buildCopyPlan();
  await ensureDirectory(PROJECT_DIRECTORY, ['public', 'robot']);

  for (const [destination, source] of copyPlan) {
    await copySafely(source, destination);
  }

  const stlCount = [...copyPlan.keys()].filter(
    (file) => extname(file).toLowerCase() === '.stl',
  ).length;
  console.log(
    color(
      `模型资源同步完成：${URDF_MODELS.length} 份 URDF，${stlCount} 个唯一 STL → public/robot/description/`,
      32,
    ),
  );
}

main().catch((error) => {
  console.error(color(`模型资源同步失败：${error.message}`, 31));
  process.exitCode = 1;
});
