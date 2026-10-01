import { useEffect, useState } from 'react';
import type { URDFRobot } from 'urdf-loader';
import URDFLoader from 'urdf-loader';
import type { JointValues } from '../core/types';
import { SIMULATION_CONFIG } from '../app/config';
import type { RobotDescription, Pose } from '../core/types';
import { WristCameraCapture, type SimCameraFrame } from '../sim/sensors/WristCameraCapture';
import type { CupBodyRef } from '../sim/tasks/CupArrangementScene';

interface URDFViewerProps {
  modelId: string;
  urdfFile: string;
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
  modelId,
  urdfFile,
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
    const loader = new URDFLoader();
    loader.packages = { rebotarm_bringup: '/robot/' };
    loader.parseCollision = true;
    loader.load(
      `${SIMULATION_CONFIG.robotAssetRoot}/${modelId}/urdf/${urdfFile}`,
      (loadedRobot) => {
        if (!active) return;
        loadedRobot.rotation.set(-Math.PI / 2, 0, 0);
        Object.values(loadedRobot.colliders).forEach((collider) => {
          collider.visible = false;
        });
        setRobot(loadedRobot);
        onLoaded();
      },
      undefined,
      (error) => {
        if (active) onError(error instanceof Error ? error.message : String(error));
      },
    );
    return () => {
      active = false;
    };
  }, [modelId, onError, onLoaded, urdfFile]);

  useEffect(() => {
    if (!robot) return;
    robot.setJointValues(jointValues);
  }, [jointValues, robot]);

  return robot ? (
    <>
      <primitive object={robot} />
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
    </>
  ) : null;
};
