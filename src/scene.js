// The arm in an orthographic, isometric view drawn like a drafting sheet:
// flat paper-coloured surfaces, ink feature edges, dash-dot joint axes, and
// drop lines from the flange (and the reach target) to the floor so heights
// read without perspective. The reach target's handle is a page element on
// top of the canvas (main.js), so it stays visible inside the flange.
import * as THREE from 'three';
import { JOINTS, linkMatrices } from './kinematics.js';

const ISO_ELEVATION = Math.atan(1 / Math.SQRT2); // 35.26°, true isometric
const VIEW_HEIGHT_MM = 1450;
const CENTER = new THREE.Vector3(-120, -230, 470);
const EDGE_ANGLE = 28; // degrees between faces before an edge is inked
const OUTLINE_MM = 2.2; // silhouette: back faces pushed out this far, drawn in ink

export function palette() {
  const asked = document.documentElement.dataset.theme;
  const dark = asked ? asked === 'dark' : window.matchMedia?.('(prefers-color-scheme: dark)').matches;
  return dark
    ? { paper: '#1c1c1c', surface: '#57534d', ink: '#ece6dc', rule: '#5a5753', hot: '#e8693f' }
    : { paper: '#e9e1d2', surface: '#f4efe6', ink: '#2a2418', rule: '#b9ab92', hot: '#e05a2b' };
}

export function createScene(canvas, links) {
  const colors = palette();
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(colors.paper);

  const scene = new THREE.Scene();
  const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, -10000, 10000);
  camera.up.set(0, 0, 1);
  const view = { azimuth: -Math.PI / 4, elevation: ISO_ELEVATION, zoom: 1 };

  scene.add(new THREE.HemisphereLight(0xffffff, 0x888070, 2.2));
  const sun = new THREE.DirectionalLight(0xffffff, 1.4);
  scene.add(sun);

  const ink = new THREE.LineBasicMaterial({ color: colors.ink });
  const rule = new THREE.LineBasicMaterial({ color: colors.rule });
  const axisMat = new THREE.LineDashedMaterial({ color: colors.rule, dashSize: 14, gapSize: 6 });
  const hot = new THREE.LineBasicMaterial({ color: colors.hot });
  const surface = new THREE.Color(colors.surface);
  // Silhouettes the crease edges miss (the smooth housings): the mesh's back
  // faces, pushed out along their normals and drawn flat in ink, peek out
  // around the front faces as an outline.
  const outline = new THREE.MeshBasicMaterial({ color: colors.ink, side: THREE.BackSide });
  outline.onBeforeCompile = shader => {
    shader.vertexShader = shader.vertexShader.replace('#include <begin_vertex>',
      `#include <begin_vertex>\n  transformed += normalize(normal) * ${OUTLINE_MM.toFixed(2)};`);
  };
  const hotColor = new THREE.Color(colors.hot);

  // Base and links 1..7, each holding its own zero-pose geometry; the link's
  // world matrix comes straight from kinematics.linkMatrices.
  const groups = links.map(({ positions, indices }, i) => {
    const indexed = new THREE.BufferGeometry();
    indexed.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    indexed.setIndex(new THREE.BufferAttribute(indices, 1));
    const flat = indexed.toNonIndexed();
    flat.computeVertexNormals();
    const material = new THREE.MeshLambertMaterial({
      color: surface.clone(),
      side: THREE.DoubleSide,
      polygonOffset: true, polygonOffsetFactor: 1, polygonOffsetUnits: 1,
    });
    const group = new THREE.Group();
    group.matrixAutoUpdate = false;
    group.add(new THREE.Mesh(flat, material));
    const hull = indexed.clone();
    hull.computeVertexNormals();
    group.add(new THREE.Mesh(hull, outline));
    group.add(new THREE.LineSegments(new THREE.EdgesGeometry(indexed, EDGE_ANGLE), ink));
    scene.add(group);
    return { group, material, index: i };
  });

  // Joint i's axis is fixed in the part below it (the base for joint 1).
  JOINTS.forEach((j, i) => {
    const c = new THREE.Vector3(0, 0, j.at);
    const d = new THREE.Vector3(...j.axis).multiplyScalar(j.axis[2] ? 70 : 150);
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints([c.clone().sub(d), c.clone().add(d)]), axisMat);
    line.computeLineDistances();
    groups[i].group.add(line);
  });

  // Floor: a 1.4 m square and centre lines.
  const floor = [];
  const h = 700;
  floor.push([-h, -h], [h, -h], [h, -h], [h, h], [h, h], [-h, h], [-h, h], [-h, -h]);
  const floorLines = new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints(floor.map(([x, y]) => new THREE.Vector3(x, y, 0))), rule);
  scene.add(floorLines);
  const centre = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(-h, 0, 0), new THREE.Vector3(h, 0, 0), new THREE.Vector3(0, -h, 0), new THREE.Vector3(0, h, 0),
  ]), axisMat);
  centre.computeLineDistances();
  scene.add(centre);

  // Flange path for the current run: the whole path in rule, the part
  // already travelled in the accent colour.
  const pathAll = new THREE.Line(new THREE.BufferGeometry(), rule);
  const pathDone = new THREE.Line(new THREE.BufferGeometry(), hot);
  pathAll.visible = pathDone.visible = false;
  scene.add(pathAll, pathDone);

  // Drop lines and markers.
  const flangeDrop = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), axisMat);
  scene.add(flangeDrop);
  const ring = r => {
    const pts = [];
    for (let k = 0; k <= 48; k++) pts.push(new THREE.Vector3(Math.cos((k / 48) * Math.PI * 2) * r, Math.sin((k / 48) * Math.PI * 2) * r, 0));
    return new THREE.BufferGeometry().setFromPoints(pts);
  };
  const targetDrop = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), hot);
  const footprint = new THREE.Line(ring(30), hot);
  targetDrop.add(footprint);
  targetDrop.visible = false;
  scene.add(targetDrop);

  const matrix = new THREE.Matrix4();
  let dirty = true;

  function frame() {
    const { clientWidth: w, clientHeight: hgt } = canvas;
    if (!w || !hgt) return;
    renderer.setSize(w, hgt, false);
    const halfH = VIEW_HEIGHT_MM / 2 / view.zoom, halfW = halfH * (w / hgt);
    Object.assign(camera, { left: -halfW, right: halfW, top: halfH, bottom: -halfH });
    camera.updateProjectionMatrix();
    const dir = new THREE.Vector3(
      Math.cos(view.elevation) * Math.cos(view.azimuth),
      Math.cos(view.elevation) * Math.sin(view.azimuth),
      Math.sin(view.elevation));
    camera.position.copy(CENTER).addScaledVector(dir, 4000);
    camera.lookAt(CENTER);
    sun.position.copy(dir).applyAxisAngle(new THREE.Vector3(0, 0, 1), 0.9).add(new THREE.Vector3(0, 0, 0.6));
  }

  function render() {
    if (!dirty) return;
    dirty = false;
    frame();
    renderer.render(scene, camera);
  }

  return {
    view,
    invalidate() { dirty = true; },
    render,
    /** Pose the arm; `load` (0..1 per link 1..7) tints each link toward the accent. */
    setPose(q, load = null) {
      linkMatrices(q).forEach((m, i) => {
        groups[i].group.matrix.fromArray(m);
        groups[i].group.matrixWorldNeedsUpdate = true;
        // Below a tenth of the peak a link stays plain, so only real effort shows.
        const k = load ? Math.max(0, Math.min(1, (load[i - 1] - 0.1) / 0.9)) : 0;
        if (i > 0) groups[i].material.color.copy(surface).lerp(hotColor, k * 0.9);
      });
      dirty = true;
    },
    setFlange(p) {
      flangeDrop.geometry.setFromPoints([new THREE.Vector3(...p), new THREE.Vector3(p[0], p[1], 0)]);
      flangeDrop.computeLineDistances();
      dirty = true;
    },
    /** Full flange path (Float32Array xyz…) or null to hide it. */
    setPath(points) {
      pathAll.visible = pathDone.visible = !!points;
      if (points) {
        pathAll.geometry.setAttribute('position', new THREE.BufferAttribute(points, 3));
        pathDone.geometry.setAttribute('position', new THREE.BufferAttribute(points, 3));
        pathAll.geometry.computeBoundingSphere();
        pathDone.geometry.computeBoundingSphere();
      }
      dirty = true;
    },
    setPathProgress(count) { pathDone.geometry.setDrawRange(0, count); dirty = true; },
    setTarget(p) {
      targetDrop.visible = !!p;
      if (p) {
        targetDrop.geometry.setFromPoints([new THREE.Vector3(...p), new THREE.Vector3(p[0], p[1], 0)]);
        footprint.position.set(p[0], p[1], 0);
      }
      dirty = true;
    },
    /** Screen point (CSS px in the canvas) of a world point. */
    toScreen(p) {
      frame();
      const v = new THREE.Vector3(...p).project(camera);
      return [(v.x + 1) / 2 * canvas.clientWidth, (1 - v.y) / 2 * canvas.clientHeight];
    },
    /** Where the pointer ray meets the horizontal plane at height z (mm). */
    onPlane(x, y, z) {
      frame();
      const ndc = new THREE.Vector2((x / canvas.clientWidth) * 2 - 1, 1 - (y / canvas.clientHeight) * 2);
      const ray = new THREE.Raycaster();
      ray.setFromCamera(ndc, camera);
      const hit = new THREE.Vector3();
      return ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), -z), hit) ? [hit.x, hit.y] : null;
    },
    resetView() { Object.assign(view, { azimuth: -Math.PI / 4, elevation: ISO_ELEVATION, zoom: 1 }); dirty = true; },
  };
}
