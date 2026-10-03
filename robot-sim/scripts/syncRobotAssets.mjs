// 同步 robot-sim 支持的模型 URDF 和其引用网格。
import { constants } from 'node:fs';
import { copyFile, lstat, mkdir, readFile, realpath, rename, rm } from 'node:fs/promises';
import { randomUUID } from 'node:crypto';
import { basename, extname, isAbsolute, relative, resolve, sep } from 'node:path';
import { fileURLToPath } from 'node:url';
import { resolvePackageUri } from '../src/utils/resolvePackageUri.mjs';

const SCRIPT_DIRECTORY = fileURLToPath(new URL('.', import.meta.url));
const PROJECT_DIRECTORY = resolve(SCRIPT_DIRECTORY, '..');
const ROS_PACKAGE_DIRECTORY = resolve(
  PROJECT_DIRECTORY,
  '../reBotArmController_ROS2/src/rebotarm_bringup',
);
const UR_DESCRIPTION_DIRECTORY = resolve(PROJECT_DIRECTORY, 'assets/ur_description');
const ROBOT_ASSET_DIRECTORY = resolve(PROJECT_DIRECTORY, 'public/robot');
const SOURCE_PACKAGE_ROOTS = {
  rebotarm_bringup: ROS_PACKAGE_DIRECTORY,
  ur_description: UR_DESCRIPTION_DIRECTORY,
};
const DESTINATION_PACKAGE_ROOTS = {
  rebotarm_bringup: ROBOT_ASSET_DIRECTORY,
  ur_description: resolve(ROBOT_ASSET_DIRECTORY, 'ur_description'),
};
const ROBOT_URDFS = [
  {
    model: 'RS',
    package: 'rebotarm_bringup',
    file: 'description/RS/urdf/ReBot_Arm_RS.urdf',
  },
  {
    model: 'DM',
    package: 'rebotarm_bringup',
    file: 'description/DM/urdf/ReBot_Arm_DM.urdf',
  },
  {
    model: 'UR5',
    package: 'ur_description',
    file: 'urdf/ur5.urdf',
  },
];
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
      if (error.code !== 'ENOENT') throw error;
      await mkdir(currentDirectory);
    }
  }

  return currentDirectory;
}

async function checkedSourcePath(packageName, sourcePath) {
  const packageRoot = SOURCE_PACKAGE_ROOTS[packageName];
  if (!packageRoot || !isPathInside(packageRoot, sourcePath)) {
    throw new Error(`源资源超出 ${packageName} 包目录：${sourcePath}`);
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

  const resolvedPackage = await realpath(packageRoot);
  if (!isPathInside(resolvedPackage, resolvedSource)) {
    throw new Error(`源资源通过符号链接离开 ${packageName} 包目录：${sourcePath}`);
  }

  const info = await lstat(resolvedSource);
  if (!info.isFile()) throw new Error(`URDF 引用的资源不是普通文件：${sourcePath}`);
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
    if (error.code !== 'ENOENT') throw error;
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

async function addToCopyPlan(copyPlan, uri, packageName) {
  const sourcePath = resolvePackageUri(uri, SOURCE_PACKAGE_ROOTS);
  const destinationPath = resolvePackageUri(uri, DESTINATION_PACKAGE_ROOTS);
  if (!isPathInside(ROBOT_ASSET_DIRECTORY, destinationPath)) {
    throw new Error(`静态资源目标超出 robot 目录：${destinationPath}`);
  }

  const source = await checkedSourcePath(packageName, sourcePath);
  const destination = relative(ROBOT_ASSET_DIRECTORY, destinationPath);
  if (!copyPlan.has(destination)) copyPlan.set(destination, source);
}

async function buildCopyPlan() {
  const copyPlan = new Map();

  for (const robot of ROBOT_URDFS) {
    const urdfUri = `package://${robot.package}/${robot.file}`;
    const urdfSource = await checkedSourcePath(
      robot.package,
      resolvePackageUri(urdfUri, SOURCE_PACKAGE_ROOTS),
    );
    await addToCopyPlan(copyPlan, urdfUri, robot.package);
    const urdfContents = await readFile(urdfSource, 'utf8');

    for (const match of urdfContents.matchAll(PACKAGE_URI_PATTERN)) {
      const uri = match[0];
      const packageName = /^package:\/\/([^/]+)\//u.exec(uri)?.[1];
      if (!packageName) throw new Error(`URDF 资源 URI 无效：${uri}`);
      await addToCopyPlan(copyPlan, uri, packageName);
    }
  }

  return copyPlan;
}

function color(text, code) {
  if (!process.stdout.isTTY || process.env.NO_COLOR) return text;
  return `\u001b[${code}m${text}\u001b[0m`;
}

async function main() {
  const copyPlan = await buildCopyPlan();
  await ensureDirectory(PROJECT_DIRECTORY, ['public', 'robot']);

  for (const [destination, source] of copyPlan) {
    await copySafely(source, destination);
  }

  const meshCount = [...copyPlan.keys()].filter((file) =>
    ['.dae', '.stl'].includes(extname(file).toLowerCase()),
  ).length;
  console.log(
    color(
      `模型资源同步完成：${ROBOT_URDFS.length} 份 URDF，${meshCount} 个唯一网格 → public/robot/`,
      32,
    ),
  );
}

main().catch((error) => {
  console.error(color(`模型资源同步失败：${error.message}`, 31));
  process.exitCode = 1;
});
