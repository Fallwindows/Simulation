import * as THREE from 'three';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/controls/OrbitControls.js';

const status = document.getElementById('status');
const metrics = document.getElementById('metrics');
const health = document.getElementById('health');

function createPointViewer(canvasId, color) {
  const canvas = document.getElementById(canvasId);
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(devicePixelRatio);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x030712);
  const camera = new THREE.PerspectiveCamera(60, 1, 0.01, 10000);
  camera.position.set(0, 0, 8);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  const geometry = new THREE.BufferGeometry();
  const material = new THREE.PointsMaterial({ color, size: 0.035 });
  const cloud = new THREE.Points(geometry, material);
  scene.add(cloud);
  let hasData = false;
  let lastRadius = 0;

  function resize() {
    renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
    camera.aspect = canvas.clientWidth / canvas.clientHeight;
    camera.updateProjectionMatrix();
  }

  function frameCloud() {
    geometry.computeBoundingSphere();
    const sphere = geometry.boundingSphere;
    if (!sphere || sphere.radius <= 0) return;
    const direction = new THREE.Vector3(1, -1, 0.7).normalize();
    const distance = Math.max(sphere.radius * 2.8, 1.0);
    controls.target.copy(sphere.center);
    camera.position.copy(sphere.center).addScaledVector(direction, distance);
    camera.near = Math.max(0.01, distance / 1000.0);
    camera.far = Math.max(1000.0, distance * 12.0);
    camera.updateProjectionMatrix();
    controls.update();
    lastRadius = sphere.radius;
  }

  function setPoints(nextPoints) {
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(nextPoints.flat(), 3));
    geometry.computeBoundingSphere();
    const radius = geometry.boundingSphere?.radius || 0;
    if (!hasData || (lastRadius > 0 && radius > lastRadius * 1.2)) {
      frameCloud();
    }
    hasData = nextPoints.length > 0;
  }

  function resetView() {
    camera.position.set(0, 0, 8);
    camera.near = 0.01;
    camera.far = 10000;
    camera.updateProjectionMatrix();
    controls.target.set(0, 0, 0);
    controls.update();
    if (hasData) frameCloud();
  }

  function render() {
    controls.update();
    renderer.render(scene, camera);
  }

  window.addEventListener('resize', resize);
  resize();
  return { setPoints, resetView, render };
}

const lidarViewer = createPointViewer('lidar-cloud', 0x67e8f9);
const mapViewer = createPointViewer('map-cloud', 0xfbbf24);
document.getElementById('reset-lidar').onclick = () => lidarViewer.resetView();
document.getElementById('reset-map').onclick = () => mapViewer.resetView();

async function poll() {
  try {
    const [s, m] = await Promise.all([fetch('/api/status'), fetch('/api/metrics')]);
    status.textContent = JSON.stringify(await s.json(), null, 2);
    metrics.textContent = JSON.stringify(await m.json(), null, 2);
    health.textContent = 'dashboard online';
  } catch (error) {
    health.textContent = 'error: ' + error;
  }
}
setInterval(poll, 1000);
poll();

function connect(streamName, viewer) {
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  const socket = new WebSocket(scheme + '://' + location.host + '/ws/' + streamName);
  socket.onmessage = event => {
    const payload = JSON.parse(event.data);
    viewer.setPoints(payload.points || []);
  };
  socket.onclose = () => setTimeout(() => connect(streamName, viewer), 1000);
  socket.onerror = () => socket.close();
}

connect('lidar', lidarViewer);
connect('map', mapViewer);

function renderLoop() {
  lidarViewer.render();
  mapViewer.render();
  requestAnimationFrame(renderLoop);
}
renderLoop();
