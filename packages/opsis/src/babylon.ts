import type { Frame } from "./index.js";

/** Structural subset of a Babylon.js camera (FreeCamera, UniversalCamera): no Babylon import. */
export interface BabylonCamera {
  position: { set(x: number, y: number, z: number): unknown };
  rotationQuaternion: { set(x: number, y: number, z: number, w: number): unknown } | null;
  fov: number;
  getScene(): { useRightHandedSystem: boolean };
}

/**
 * Drives `camera` from `frame` for a viewport of `aspect`. Returns false while there is no pose.
 * The scene must be right-handed (`scene.useRightHandedSystem = true`), as Opsis world coordinates are, and the
 * camera needs a `rotationQuaternion` (`BABYLON.Quaternion.Identity()`).
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
