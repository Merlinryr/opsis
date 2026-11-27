import type { Frame } from "./index.js";

/** Only the `THREE.PerspectiveCamera` fields Opsis uses, so no three.js import is needed and any version works. */
export interface ThreeCamera {
  position: { fromArray(a: number[]): unknown };
  quaternion: { fromArray(a: number[]): unknown };
  fov: number;
  aspect: number;
  near: number;
  far: number;
  updateProjectionMatrix(): void;
}

/** Moves `camera` to the frame's pose and matches its field of view to a viewport of `aspect`. Returns false while there is no pose yet. */
export function updateCamera(camera: ThreeCamera, frame: Frame, aspect: number) {
  if (!frame.pose) return false;
  camera.position.fromArray(frame.pose.position);
  camera.quaternion.fromArray(frame.pose.quaternion);
  camera.fov = (frame.fovY(aspect) * 180) / Math.PI;
  camera.aspect = aspect;
  camera.updateProjectionMatrix();
  return true;
}
