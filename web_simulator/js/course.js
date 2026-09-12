import * as THREE from 'three';

// Ground texture: the course layout image the user provided directly
// (web_simulator/png/shihou_cource_unity.png), used as-is instead of the
// earlier hand-traced procedural reconstruction (which didn't match well
// enough). Real-world scale is not modeled yet -- per instruction, ignored
// for now -- so the plane this is mapped onto is just sized to a reasonable
// scale using the image's own aspect ratio. Revisit COURSE_WIDTH_M once
// real dimensions/orientation are worked out.
const COURSE_IMAGE_URL = 'png/shihou_cource_unity.png'; // relative to index.html
const COURSE_IMAGE_ASPECT = 1024 / 819; // width / height, from the source PNG

export const COURSE_WIDTH_M = 100;
export const COURSE_DEPTH_M = COURSE_WIDTH_M / COURSE_IMAGE_ASPECT;

const textureLoader = new THREE.TextureLoader();

export function createCourseTexture() {
  const texture = textureLoader.load(COURSE_IMAGE_URL, undefined, undefined, (err) =>
    console.error(`Failed to load course image ${COURSE_IMAGE_URL}`, err)
  );
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 4;
  return texture;
}
