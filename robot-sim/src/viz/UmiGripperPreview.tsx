import { useEffect, useState } from 'react';
import { Canvas, useThree } from '@react-three/fiber';
import { Box3, LoadingManager, Vector3 } from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { URDFRobot } from 'urdf-loader';
import URDFLoader from 'urdf-loader';
import { publishDebugEvent } from '../app/debugBus';

const URDF_URL = '/robot/umi-gripper/umi-gripper.urdf';
const ASSET_DIRECTORY = '/robot/umi-gripper/';

interface PreviewMetrics {
  meshFileCount: number;
  size: [number, number, number];
}

const PreviewControls = () => {
  const { camera, gl } = useThree();

  useEffect(() => {
    const controls = new OrbitControls(camera, gl.domElement);
    controls.target.set(1, 0.1, -0.04);
    controls.minDistance = 0.12;
    controls.maxDistance = 2.5;
    controls.update();
    return () => controls.dispose();
  }, [camera, gl]);

  return null;
};

export const UmiGripperPreview = () => {
  const [robot, setRobot] = useState<URDFRobot | null>(null);
  const [status, setStatus] = useState('正在加载 URDF 与网格');
  const [error, setError] = useState('');
  const [metrics, setMetrics] = useState<PreviewMetrics | null>(null);
  const [fingerPosition, setFingerPosition] = useState(0);

  useEffect(() => {
    let active = true;
    let loadedRobot: URDFRobot | null = null;
    let meshFileCount = 0;
    const failedMeshUrls = new Set<string>();
    const manager = new LoadingManager();
    manager.onLoad = () => {
      if (!active || !loadedRobot) return;

      const bounds = new Box3().setFromObject(loadedRobot);
      const size = bounds.getSize(new Vector3()).toArray() as [number, number, number];
      setMetrics({ meshFileCount, size });
      setStatus(failedMeshUrls.size === 0 ? '网格加载完成' : '部分网格加载失败');
      publishDebugEvent('umi_gripper_preview:assets_loaded', {
        urdf: URDF_URL,
        meshFileCount,
        failedMeshUrls: [...failedMeshUrls],
        sizeMeters: size,
      });
    };
    manager.onError = (url) => {
      if (!active) return;
      failedMeshUrls.add(url);
      setError(`网格加载失败：${url}`);
      setStatus('网格加载失败');
      publishDebugEvent('umi_gripper_preview:asset_error', { url });
    };

    const load = async () => {
      publishDebugEvent('umi_gripper_preview:loading', { urdf: URDF_URL });
      try {
        const response = await fetch(URDF_URL);
        if (!response.ok) throw new Error(`URDF 加载失败：HTTP ${response.status}`);
        const source = await response.text();
        const document = new DOMParser().parseFromString(source, 'application/xml');
        meshFileCount = document.querySelectorAll('link > visual mesh[filename]').length;
        const loader = new URDFLoader(manager);
        loader.workingPath = ASSET_DIRECTORY;
        loader.parseCollision = false;
        loadedRobot = loader.parse(source);
        loadedRobot.rotation.set(-Math.PI / 2, 0, 0);
        setRobot(loadedRobot);
      } catch (loadError) {
        if (!active) return;
        const message = loadError instanceof Error ? loadError.message : String(loadError);
        setError(message);
        setStatus('URDF 加载失败');
        publishDebugEvent('umi_gripper_preview:error', { message });
      }
    };

    void load();
    return () => {
      active = false;
    };
  }, []);

  useEffect(() => {
    if (!robot) return;
    robot.setJointValues({
      left_finger_joint: fingerPosition,
      right_finger_joint: fingerPosition,
    });
    robot.updateWorldMatrix(true, true);
    const leftFingerPosition = robot.links.left_finger_holder
      .getWorldPosition(new Vector3())
      .toArray();
    const rightFingerPosition = robot.links.right_finger_holder
      .getWorldPosition(new Vector3())
      .toArray();
    publishDebugEvent('umi_gripper_preview:finger_joints', {
      left_finger_joint: fingerPosition,
      right_finger_joint: fingerPosition,
      fingerSpanX: leftFingerPosition[0] - rightFingerPosition[0],
    });
  }, [fingerPosition, robot]);

  return (
    <main style={{ position: 'fixed', inset: 0, background: '#10151d', color: '#e8eef5' }}>
      <Canvas camera={{ position: [1.28, 0.3, 0.42], fov: 38 }}>
        <color attach="background" args={['#10151d']} />
        <ambientLight intensity={1.15} />
        <directionalLight position={[1.8, 2.5, 1.2]} intensity={2.1} />
        <gridHelper args={[1, 20, '#315056', '#1d2c35']} position={[1, -0.12, 0]} />
        <axesHelper args={[0.12]} position={[1, 0, 0]} />
        <group position={[1, 0, 0]}>{robot ? <primitive object={robot} /> : null}</group>
        <PreviewControls />
      </Canvas>
      <section
        aria-label="UMI gripper debug preview"
        style={{
          position: 'absolute',
          top: 18,
          left: 18,
          width: 310,
          padding: 16,
          border: '1px solid #273341',
          borderRadius: 12,
          background: 'rgba(16, 23, 32, 0.92)',
          font: '13px/1.6 system-ui, sans-serif',
        }}
      >
        <strong>UMI gripper 独立预览</strong>
        <div style={{ color: error ? '#fc8e95' : '#65dfc8', marginTop: 5 }}>{status}</div>
        <div style={{ color: '#9aabb8', overflowWrap: 'anywhere' }}>{URDF_URL}</div>
        {error ? <div style={{ color: '#fc8e95' }}>{error}</div> : null}
        {metrics ? (
          <div style={{ color: '#9aabb8' }}>
            STL mesh：{metrics.meshFileCount} 个；尺寸：
            {metrics.size.map((value) => `${(value * 100).toFixed(1)} cm`).join(' × ')}
          </div>
        ) : null}
        <label htmlFor="umi-gripper-finger-position" style={{ display: 'block', marginTop: 10 }}>
          左右 finger joint 同步：{(fingerPosition * 1000).toFixed(0)} mm
        </label>
        <input
          id="umi-gripper-finger-position"
          aria-label="finger joint position"
          max="0.05"
          min="0"
          onChange={(event) => setFingerPosition(Number(event.target.value))}
          step="0.001"
          style={{ width: '100%', accentColor: '#65dfc8' }}
          type="range"
          value={fingerPosition}
        />
      </section>
    </main>
  );
};
