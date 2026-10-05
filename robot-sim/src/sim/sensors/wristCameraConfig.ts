// STL 镜头外表面的几何代理不代表真实光心，需用实物标定。
// 固定四元数将相机 -Z/+Y 映射到 GoPro 局部 -Y/+Z。
export const WRIST_CAMERA_CONFIG = {
  // 软件渲染源尺寸，按 16:9 中心裁剪；不代表 HDMI 的物理采集分辨率。
  captureWidth: 448,
  captureHeight: 252,
  // 沿用旧预览的 60 度，GoPro clean HDMI 的实际 FOV 仍待实机校准。
  fieldOfViewDegrees: 60,
  near: 0.01,
  far: 5,
  goproVisualOrigin: [-0.01265, -0.0223, 0.08185] as const,
  goproVisualRotation: [0, 0, 0] as const,
  goproCameraPosition: [0.0199, -0.0296, 0.00915] as const,
  goproCameraQuaternion: [0, Math.SQRT1_2, Math.SQRT1_2, 0] as const,
  legacyMountPosition: [0, 0, 0.055] as const,
  legacyMountRotation: [0, 1.355 - Math.PI / 2, 0] as const,
} as const;
