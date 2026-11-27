import type { Frame } from "./index.js";

/** Only the Babylon.js camera fields Opsis uses (FreeCamera, UniversalCamera), so no Babylon import is needed. */
export interface BabylonCamera {
  position: { set(x: number, y: number, z: number): unknown };
  rotationQuaternion: { set(x: number, y: number, z: number, w: number): unknown } | null;
  fov: number;
  getScene(): { useRightHandedSystem: boolean };
}

/**
 * Moves `camera` to the frame's pose and matches its field of view to a viewport of `aspect`. Returns false while there is no pose yet.
 * Needs `scene.useRightHandedSystem = true` (Opsis is right-handed) and a `camera.rotationQuaternion`.
 */
export function updateCamera(camera: BabylonCamera, frame: Frame, aspect: number) {
  if (!camera.getScene().useRightHandedSystem || !camera.rotationQuaternion) throw new Error("opsis/babylon: needs a right-handed scene and camera.rotationQuaternion");
  if (!frame.pose) return false;
  const [x, y, z] = frame.pose.position, q = frame.pose.quaternion;
  camera.position.set(x, y, z);
  camera.rotationQuaternion.set(q[0], q[1], q[2], q[3]);
  camera.fov = frame.fovY(aspect);
  return true;
}
