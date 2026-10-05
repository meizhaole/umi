import { useEffect, useState } from 'react';
import { LoadingManager, Quaternion, Vector3 } from 'three';
import type { URDFRobot } from 'urdf-loader';
import URDFLoader from 'urdf-loader';
import type { JointValues } from '../core/types';
import type { RobotDescription, Pose } from '../core/types';
import { publishDebugEvent } from '../app/debugBus';
import { WristCameraCapture, type SimCameraFrame } from '../sim/sensors/WristCameraCapture';
import type { CupBodyRef } from '../sim/tasks/CupArrangementScene';

interface URDFViewerProps {
  urdfXml: string;
  packageMappings: Readonly<Record<string, string>>;
  jointValues: JointValues;
  onError: (message: string) => void;
  onLoaded: () => void;
  description: RobotDescription;
  cameraEnabled: boolean;
  cupBodyRef: CupBodyRef;
  inferenceActive: boolean;
  tcpPose: Pose;
  onCameraFrame: (frame: SimCameraFrame) => void;
}

export const URDFViewer = ({
  urdfXml,
  packageMappings,
  jointValues,
  onError,
  onLoaded,
  description,
  cameraEnabled,
  cupBodyRef,
  inferenceActive,
  tcpPose,
  onCameraFrame,
}: URDFViewerProps) => {
  const [robot, setRobot] = useState<URDFRobot | null>(null);

  useEffect(() => {
    let active = true;
    setRobot(null);
    const manager = new LoadingManager();
    manager.onLoad = () => {
      if (active) {
        publishDebugEvent('robot:visual_assets_loaded', { name: description.name });
        onLoaded();
      }
    };
    manager.onError = (url) => {
      if (active) onError(`URDF 网格加载失败：${url}`);
    };
    const loader = new URDFLoader(manager);
    loader.packages = packageMappings;
    loader.parseCollision = false;
    try {
      const loadedRobot = loader.parse(urdfXml);
      loadedRobot.rotation.set(-Math.PI / 2, 0, 0);
      setRobot(loadedRobot);
    } catch (error) {
      if (active) onError(error instanceof Error ? error.message : String(error));
    }
    return () => {
      active = false;
    };
  }, [description.name, onError, onLoaded, packageMappings, urdfXml]);

  useEffect(() => {
    if (!robot) return;
    robot.setJointValues(jointValues);
    robot.updateMatrixWorld(true);
    const linkPoses = Object.fromEntries(
      Object.entries(robot.links).map(([name, link]) => [
        name,
        {
          position: link.getWorldPosition(new Vector3()).toArray(),
          orientation: link.getWorldQuaternion(new Quaternion()).toArray(),
        },
      ]),
    );
    publishDebugEvent('robot:visual_state', { jointValues, linkPoses });
  }, [jointValues, robot]);

  return robot ? (
    <>
      <primitive object={robot} />
      {cameraEnabled ? (
        <WristCameraCapture
          description={description}
          enabled={cameraEnabled}
          cupBodyRef={cupBodyRef}
          inferenceActive={inferenceActive}
          jointValues={jointValues}
          onCapture={onCameraFrame}
          eefPose={tcpPose}
          robot={robot}
        />
      ) : null}
    </>
  ) : null;
};
