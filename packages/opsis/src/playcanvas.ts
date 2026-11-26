import type { Frame } from "./index.js";

/** Structural subset of a PlayCanvas camera entity: no PlayCanvas import, any version. */
export interface PlayCanvasCameraEntity {
  setPosition(x: number, y: number, z: number): unknown;
  setRotation(x: number, y: number, z: number, w: number): unknown;
  camera: { fov: number; horizontalFov?: boolean } | null | undefined;
}

/** Drives a camera entity from `frame` for a viewport of `aspect`. Returns false while there is no pose. */
export function updateCamera(entity: PlayCanvasCameraEntity, frame: Frame, aspect: number) {
  if (!entity.camera) throw new Error("opsis/playcanvas: entity has no camera component");
  if (!frame.pose) return false;
  const [x, y, z] = frame.pose.position, q = frame.pose.quaternion;
  entity.setPosition(x, y, z);
  entity.setRotation(q[0], q[1], q[2], q[3]);
  entity.camera.horizontalFov = false;
  entity.camera.fov = (frame.fovY(aspect) * 180) / Math.PI;
  return true;
}
