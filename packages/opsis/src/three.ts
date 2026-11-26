import type { Frame } from "./index.js";

/** Structural subset of `THREE.PerspectiveCamera`: no three.js import, any version. */
export interface ThreeCamera {
  position: { fromArray(a: number[]): unknown };
  quaternion: { fromArray(a: number[]): unknown };
  fov: number;
  aspect: number;
  near: number;
  far: number;
  updateProjectionMatrix(): void;
}

/** Drives `camera` from `frame` for a viewport of `aspect`. Returns false while there is no pose. */
export function updateCamera(camera: ThreeCamera, frame: Frame, aspect: number) {
  if (!frame.pose) return false;
  camera.position.fromArray(frame.pose.position);
  camera.quaternion.fromArray(frame.pose.quaternion);
  camera.fov = (frame.fovY(aspect) * 180) / Math.PI;
  camera.aspect = aspect;
  camera.updateProjectionMatrix();
  return true;
}
