import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js';

const status = document.getElementById('status');
const metrics = document.getElementById('metrics');
const health = document.getElementById('health');
const canvas = document.getElementById('cloud');
let points = [];

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(devicePixelRatio);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x030712);
const camera = new THREE.PerspectiveCamera(60, 1, 0.01, 1000);
camera.position.set(0, 0, 8);
const cloudGeometry = new THREE.BufferGeometry();
const cloudMaterial = new THREE.PointsMaterial({ color: 0x67e8f9, size: 0.035 });
const cloud = new THREE.Points(cloudGeometry, cloudMaterial);
scene.add(cloud);

function resize() { renderer.setSize(canvas.clientWidth, canvas.clientHeight, false); camera.aspect = canvas.clientWidth / canvas.clientHeight; camera.updateProjectionMatrix(); }
window.addEventListener('resize', resize); resize();
document.getElementById('reset').onclick = () => { points = []; draw(); };

function draw() {
  cloudGeometry.setAttribute('position', new THREE.Float32BufferAttribute(points.flat(), 3));
  cloudGeometry.computeBoundingSphere();
  renderer.render(scene, camera);
}

async function poll() {
  try {
    const [s, m] = await Promise.all([fetch('/api/status'), fetch('/api/metrics')]);
    status.textContent = JSON.stringify(await s.json(), null, 2);
    metrics.textContent = JSON.stringify(await m.json(), null, 2);
    health.textContent = 'dashboard online';
  } catch (error) { health.textContent = `error: ${error}`; }
}
setInterval(poll, 1000); poll();

function connect() {
  const socket = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws/lidar`);
  socket.onmessage = event => { points = JSON.parse(event.data).points || []; draw(); };
  socket.onclose = () => setTimeout(connect, 1000);
}
connect();
function renderLoop() { renderer.render(scene, camera); requestAnimationFrame(renderLoop); }
renderLoop();
