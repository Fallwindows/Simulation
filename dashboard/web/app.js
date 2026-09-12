import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js';

const status = document.getElementById('status');
const metrics = document.getElementById('metrics');
const health = document.getElementById('health');

function createPointViewer(canvasId, color) {
  const canvas = document.getElementById(canvasId);
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(devicePixelRatio);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x030712);
  const camera = new THREE.PerspectiveCamera(60, 1, 0.01, 1000);
  camera.position.set(0, 0, 8);
  const geometry = new THREE.BufferGeometry();
  const material = new THREE.PointsMaterial({ color, size: 0.035 });
  const cloud = new THREE.Points(geometry, material);
  scene.add(cloud);
  let points = [];

  function resize() {
    renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);
    camera.aspect = canvas.clientWidth / canvas.clientHeight;
    camera.updateProjectionMatrix();
  }

  function setPoints(nextPoints) {
    points = nextPoints;
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(points.flat(), 3));
    geometry.computeBoundingSphere();
  }

  function reset() { setPoints([]); }
  function render() { renderer.render(scene, camera); }

  window.addEventListener('resize', resize);
  resize();
  return { setPoints, reset, render };
}

const lidarViewer = createPointViewer('lidar-cloud', 0x67e8f9);
const mapViewer = createPointViewer('map-cloud', 0xfbbf24);
document.getElementById('reset-lidar').onclick = () => lidarViewer.reset();
document.getElementById('reset-map').onclick = () => mapViewer.reset();

async function poll() {
  try {
    const [s, m] = await Promise.all([fetch('/api/status'), fetch('/api/metrics')]);
    status.textContent = JSON.stringify(await s.json(), null, 2);
    metrics.textContent = JSON.stringify(await m.json(), null, 2);
    health.textContent = 'dashboard online';
  } catch (error) {
    health.textContent = `error: ${error}`;
  }
}
setInterval(poll, 1000);
poll();

function connect(streamName, viewer) {
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  const socket = new WebSocket(`${scheme}://${location.host}/ws/${streamName}`);
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
