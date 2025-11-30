const canvas = document.getElementById('map');
const ctx = (canvas instanceof HTMLCanvasElement) ? canvas.getContext('2d') : null;
const meta = document.getElementById('meta');
const legend = document.getElementById('legend');
if (canvas instanceof HTMLCanvasElement) {
  canvas.width = 720;
  canvas.height = 500;
}
const tileColors = {
  0: '#5c2a2d',
  1: '#0e1c23',
  2: '#0c8a16',
  3: '#c62828',
  4: '#ff8f00',
  5: '#593a78',
  6: '#ff0000'
};
function tileColor(type) {
  return Object.prototype.hasOwnProperty.call(tileColors, type)
    ? tileColors[type]
    : '#0e1c23';
}
function renderLegend() {
  const entries = [
    { label: 'Walkable', color: tileColors[1] },
    { label: 'Low priority', color: tileColors[2] },
    { label: 'Non-walkable', color: tileColors[0] },
    { label: 'Thickened wall', color: tileColors[6] },
    { label: 'Teleport over', color: tileColors[5] },
    { label: 'Objects', color: '#ffeb3b' },
    { label: 'Monsters', color: '#ff7043', shape: 'circle' },
    { label: 'Navigation path', color: '#00bcd4', shape: 'line' },
    { label: 'Player', color: '#00e5ff', shape: 'circle' }
  ];
  if (legend) {
    legend.innerHTML = entries
      .map((entry) => {
        const shape = entry.shape || 'square';
        return `<div class="legend-item"><span class="legend-swatch ${shape}" style="--legend-color:${entry.color};"></span>${entry.label}</div>`;
      })
      .join('');
  }
}
renderLegend();


function asArray(value) {
  if (!value) {
    return [];
  }
  return Array.isArray(value) ? value : [];
}

function draw(payload) {
  if (!canvas || !ctx) return;
  ctx.fillStyle = '#050607';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  const scale = typeof payload.scale === 'number' ? payload.scale : 3;
  const originX = canvas.width / 2;
  const originY = canvas.height / 2;

  asArray(payload.tiles).forEach((tile) => {
    ctx.fillStyle = tileColor(tile.type);
    const size = tile.type === 0 || tile.type === 5 || tile.type === 6
      ? Math.max(scale, 3)
      : Math.max(1, scale - 0.5);
    ctx.fillRect(originX + tile.x * scale - size / 2, originY + tile.y * scale - size / 2, size, size);
  });

  const pathPoints = asArray(payload.path);
  if (pathPoints.length > 1) {
    ctx.strokeStyle = '#00bcd4';
    ctx.lineWidth = 2;
    ctx.beginPath();
    pathPoints.forEach((pt, idx) => {
      const px = originX + pt.x * scale;
      const py = originY + pt.y * scale;
      if (idx === 0) {
        ctx.moveTo(px, py);
      } else {
        ctx.lineTo(px, py);
      }
    });
    ctx.stroke();
  }

  ctx.fillStyle = '#ffeb3b';
  asArray(payload.objects).forEach((obj) => {
    ctx.fillRect(originX + obj.x * scale - 2, originY + obj.y * scale - 2, 4, 4);
  });

  ctx.fillStyle = '#ff7043';
  asArray(payload.monsters).forEach((mon) => {
    ctx.beginPath();
    ctx.arc(originX + mon.x * scale, originY + mon.y * scale, mon.size || 3, 0, Math.PI * 2);
    ctx.fill();
  });

  ctx.fillStyle = '#00e5ff';
  // Always draw player at center
  ctx.beginPath();
  ctx.arc(originX, originY, 4, 0, Math.PI * 2);
  ctx.fill();

  if (payload.meta && meta) {
    meta.textContent = ' ' + payload.meta;
  }
}

let latestPayload = null;
let ticking = false;

function update(payload) {
  latestPayload = payload || {};
  if (!ticking) {
    ticking = true;
    requestAnimationFrame(() => {
      draw(latestPayload);
      ticking = false;
    });
  }
}

function poll() {
  fetch('/data', { cache: 'no-store' })
    .then((res) => res.json())
    .then(update)
    .catch((err) => {
      if (meta) meta.textContent = ' Error: ' + err.message;
    });

  setTimeout(poll, 100);
}

poll();
