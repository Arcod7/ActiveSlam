'use strict';

const S = {
  runs: [],
  seeds: [],
  run: null,
  compare: false,
  overlay: false,
  panes: { A: null, B: null },   // A is the pinned pane, B follows navigation
};

const el = (id) => document.getElementById(id);
const statusLine = el('status-line');

function say(message) { statusLine.textContent = message || ''; }

async function api(path, options) {
  const response = await fetch(path, options);
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || response.statusText);
  return payload;
}

function whenPlotly(callback) {
  if (window.Plotly) callback();
  else setTimeout(() => whenPlotly(callback), 60);
}

// --- theme -----------------------------------------------------------------

function theme() {
  const style = getComputedStyle(document.documentElement);
  const read = (name) => style.getPropertyValue(name).trim();
  return {
    surface: read('--surface-1'),
    text: read('--text-primary'),
    textSecondary: read('--text-secondary'),
    muted: read('--text-muted'),
    grid: read('--grid'),
    axis: read('--axis'),
    band: read('--band'),
    s1: read('--series-1'),
    s2: read('--series-2'),
    s3: read('--series-3'),
    mapLow: read('--map-low'),
    mapHigh: read('--map-high'),
  };
}

// --- data helpers ----------------------------------------------------------

function col(table, name) {
  if (!table || !table[name]) return null;
  const values = table[name];
  return values.some((v) => v !== null && v !== undefined) ? values : null;
}

function rel(times, t0) {
  if (!times) return null;
  return times.map((v) => (v === null || v === undefined ? null : v - t0));
}

function planner(data, names) {
  for (const name of names) {
    if (data.planner && data.planner[name]) return data.planner[name];
  }
  return null;
}

function fmt(value, digits) {
  if (value === null || value === undefined) return '--';
  if (digits === 0) return String(Math.round(value));
  return Number(value).toFixed(digits === undefined ? 3 : digits);
}

function statusClass(status) {
  if (status === 'ok') return 'ok';
  if (!status) return '';
  if (status === 'no_data' || status === 'motionless') return 'bad';
  return 'warn';
}

// --- chart construction ----------------------------------------------------

const BASE_LINE = { type: 'scatter', mode: 'lines', line: { width: 2 } };

function baseLayout(title, t, extra) {
  return Object.assign({
    title: { text: title, font: { size: 13, color: t.text }, x: 0, xanchor: 'left', y: 0.97 },
    paper_bgcolor: t.surface,
    plot_bgcolor: t.surface,
    font: { family: 'system-ui, -apple-system, "Segoe UI", sans-serif', size: 11, color: t.muted },
    margin: { l: 56, r: 16, t: 34, b: 40 },
    xaxis: { gridcolor: t.grid, zerolinecolor: t.axis, linecolor: t.axis,
             autorange: true, title: { text: 'time (s)' } },
    yaxis: { gridcolor: t.grid, zerolinecolor: t.axis, linecolor: t.axis, autorange: true },
    hovermode: 'x unified',
    showlegend: false,
    legend: { orientation: 'h', y: -0.22, font: { color: t.textSecondary } },
  }, extra || {});
}

function trajectoryChart(data, overlay, t) {
  const traces = [];
  const map = data.map;
  if (map && map.kind === 'grid') {
    traces.push({
      type: 'heatmap', z: map.z,
      x0: map.x0, dx: map.resolution, y0: map.y0, dy: map.resolution,
      colorscale: [[0, t.mapLow], [1, t.mapHigh]],
      showscale: false, hoverinfo: 'skip', opacity: 0.5, showlegend: false,
    });
  }

  const lines = [
    ['gt', 'Ground truth', t.s3],
    ['slam', 'SLAM estimate', t.s1],
    ['odom', 'Dead reckoning', t.s2],
  ];
  for (const [key, label, color] of lines) {
    const track = data.traj[key];
    if (!track) continue;
    traces.push(Object.assign({}, BASE_LINE, {
      x: track.x, y: track.y, name: label,
      line: { width: 2, color },
      hovertemplate: `${label}<br>x %{x:.2f} m, y %{y:.2f} m<extra></extra>`,
    }));
  }

  if (data.goals) {
    traces.push({
      type: 'scatter', mode: 'markers', x: data.goals.x, y: data.goals.y,
      name: 'Frontier goals',
      marker: { symbol: 'x', size: 8, color: t.muted, opacity: 0.75 },
      hovertemplate: 'goal<br>x %{x:.2f} m, y %{y:.2f} m<extra></extra>',
    });
  }
  if (data.revisit_targets) {
    traces.push({
      type: 'scatter', mode: 'markers',
      x: data.revisit_targets.x, y: data.revisit_targets.y, name: 'Revisit targets',
      marker: { symbol: 'diamond-open', size: 10, color: t.textSecondary, line: { width: 2 } },
      hovertemplate: 'revisit target<br>x %{x:.2f} m, y %{y:.2f} m<extra></extra>',
    });
  }

  if (overlay) {
    for (const [key, label] of [['gt', 'Pinned ground truth'], ['slam', 'Pinned SLAM']]) {
      const track = overlay.traj[key];
      if (!track) continue;
      traces.push(Object.assign({}, BASE_LINE, {
        x: track.x, y: track.y, name: label,
        line: { width: 2, color: t.textSecondary, dash: 'dot' },
        hovertemplate: `${label}<br>x %{x:.2f} m, y %{y:.2f} m<extra></extra>`,
      }));
    }
  }

  return {
    key: 'traj', tall: true, traces, time: false,
    layout: baseLayout('Trajectory (x-y, world frame)', t, {
      showlegend: true,
      hovermode: 'closest',
      margin: { l: 56, r: 16, t: 34, b: 46 },
      xaxis: { gridcolor: t.grid, zerolinecolor: t.axis, linecolor: t.axis, title: { text: 'x (m)' } },
      yaxis: {
        gridcolor: t.grid, zerolinecolor: t.axis, linecolor: t.axis,
        title: { text: 'y (m)' }, scaleanchor: 'x', scaleratio: 1,
      },
    }),
  };
}

function timeChart(key, title, yTitle, series, t, options) {
  const traces = series.filter((s) => s.y).map((s) => Object.assign({}, BASE_LINE, {
    x: s.x, y: s.y, name: s.name,
    line: { width: 2, color: s.color },
    connectgaps: false,
    hovertemplate: `${s.name} %{y:.4g}<extra></extra>`,
  }));
  if (!traces.length) return null;
  const legend = traces.length > 1;
  const layout = baseLayout(title, t, {
    showlegend: legend,
    margin: { l: 56, r: 16, t: 34, b: legend ? 64 : 40 },
    legend: { orientation: 'h', y: -0.36, font: { color: t.textSecondary } },
    yaxis: { gridcolor: t.grid, zerolinecolor: t.axis, linecolor: t.axis, title: { text: yTitle } },
  });
  if (options && options.markers) traces.push(options.markers);
  return { key, tall: false, traces, time: true, layout };
}

function buildCharts(data, overlay) {
  const t = theme();
  const t0 = data.t0 || 0;
  const metrics = data.metrics;
  const mapMetrics = data.map_metrics;
  const charts = [trajectoryChart(data, overlay, t)];

  const mt = metrics ? rel(metrics.t, t0) : null;
  if (mt) {
    charts.push(timeChart('error', 'Position error (unaligned)', 'error (m)', [
      { x: mt, y: col(metrics, 'abs_error'), name: 'Absolute error', color: t.s1 },
      { x: mt, y: col(metrics, 'ate'), name: 'Cumulative ATE', color: t.s2 },
    ], t));
    charts.push(timeChart('rpe_trans', 'Relative pose error (translation)', 'RPE (m)', [
      { x: mt, y: col(metrics, 'rpe_trans'), name: 'RPE translation', color: t.s1 },
    ], t));
    charts.push(timeChart('rpe_rot', 'Relative pose error (rotation)', 'RPE (deg)', [
      { x: mt, y: col(metrics, 'rpe_rot_deg'), name: 'RPE rotation', color: t.s1 },
    ], t));
    charts.push(timeChart('dopt', 'Pose uncertainty (D-optimality)', 'det(cov)^(1/3)', [
      { x: mt, y: col(metrics, 'dopt'), name: 'D-optimality', color: t.s1 },
    ], t));
    charts.push(timeChart('uratio', 'Revisit trigger ratio', 'u_ratio', [
      { x: mt, y: col(metrics, 'u_ratio'), name: 'Uncertainty ratio', color: t.s1 },
    ], t));
    charts.push(timeChart('counts', 'Loop closures and revisits', 'count', [
      { x: mt, y: col(metrics, 'lc_count'), name: 'Loop closures', color: t.s1 },
      { x: mt, y: col(metrics, 'revisit_count'), name: 'Revisits', color: t.s2 },
      { x: mt, y: col(metrics, 'rebuild_count'), name: 'Rebuilds', color: t.s3 },
    ], t));
  }

  if (mapMetrics) {
    const gt = rel(mapMetrics.t, t0);
    charts.push(timeChart('coverage', 'Map coverage vs ground truth', 'coverage', [
      { x: gt, y: col(mapMetrics, 'coverage'), name: 'Coverage', color: t.s1 },
    ], t));
    if (col(mapMetrics, 'iou_occ')) {
      charts.push(timeChart('iou', 'Occupied-cell agreement', 'IoU', [
        { x: gt, y: col(mapMetrics, 'iou_occ'), name: 'IoU', color: t.s1 },
      ], t));
    }
    if (col(mapMetrics, 'chamfer')) {
      charts.push(timeChart('chamfer', 'Surface distance', 'chamfer (m)', [
        { x: gt, y: col(mapMetrics, 'chamfer'), name: 'Chamfer', color: t.s1 },
      ], t));
    }
  }

  const extractor = planner(data, ['extractor']);
  if (extractor) {
    const pt = rel(extractor.columns.t_ros, t0);
    let markers = null;
    if (extractor.events && extractor.events.length && extractor.events.length < 400) {
      markers = {
        type: 'scatter', mode: 'markers',
        x: extractor.events.map((e) => e.t - t0),
        y: extractor.events.map(() => 0),
        text: extractor.events.map((e) => e.event),
        name: 'Planner events',
        marker: { symbol: 'triangle-up', size: 9, color: t.textSecondary },
        hovertemplate: '%{text}<extra></extra>',
      };
    }
    charts.push(timeChart('goal_dist', 'Distance to frontier goal', 'distance (m)', [
      { x: pt, y: col(extractor.columns, 'dist_m'), name: 'Goal distance', color: t.s1 },
    ], t, { markers }));
    charts.push(timeChart('clusters', 'Frontier clusters', 'count', [
      { x: pt, y: col(extractor.columns, 'clusters'), name: 'Clusters', color: t.s1 },
      { x: pt, y: col(extractor.columns, 'blacklist_n'), name: 'Blacklisted', color: t.s2 },
    ], t));
  }

  const control = planner(data, ['wall_oriented', 'controller', 'wall_looking']);
  if (control) {
    const ct = rel(control.columns.t_ros, t0);
    charts.push(timeChart('velocity', 'Body velocity commands', 'command', [
      { x: ct, y: col(control.columns, 'surge'), name: 'Surge', color: t.s1 },
      { x: ct, y: col(control.columns, 'sway'), name: 'Sway', color: t.s2 },
      { x: ct, y: col(control.columns, 'heave'), name: 'Heave', color: t.s3 },
    ], t));
    charts.push(timeChart('yaw', 'Yaw command', 'yaw command', [
      { x: ct, y: col(control.columns, 'yaw_cmd'), name: 'Yaw command', color: t.s1 },
    ], t));
    charts.push(timeChart('heading', 'Heading error', 'error (deg)', [
      { x: ct, y: col(control.columns, 'hdg_err_deg'), name: 'Heading error', color: t.s1 },
    ], t));
  }

  const bands = bandShapesFor(data, t0, t);
  let labelled = false;
  return charts.filter(Boolean).map((chart) => {
    if (chart.time && bands.shapes.length) {
      chart.layout.shapes = bands.shapes;
      if (!labelled) {                       // one key per pane, not per chart
        chart.layout.annotations = bands.annotations;
        labelled = true;
      }
    }
    return chart;
  });
}

/* Shaded spans mark the revisit state machine; each carries its own label so
   the shading never has to be decoded from colour alone. */
function bandShapesFor(data, t0, t) {
  const bands = data.revisit_bands || [];
  return {
    shapes: bands.map((band) => ({
      type: 'rect', xref: 'x', yref: 'paper',
      x0: band.t0 - t0, x1: band.t1 - t0, y0: 0, y1: 1,
      fillcolor: t.band, line: { width: 0 }, layer: 'below',
    })),
    annotations: bands.map((band) => ({
      x: (band.t0 + band.t1) / 2 - t0, y: 0.99, xref: 'x', yref: 'paper',
      text: band.state, showarrow: false, yanchor: 'top',
      font: { size: 10, color: t.muted },
    })),
  };
}

// --- rendering -------------------------------------------------------------

const PLOT_CONFIG = { responsive: true, displaylogo: false,
  modeBarButtonsToRemove: ['select2d', 'lasso2d', 'autoScale2d'] };

function renderTiles(container, data) {
  container.innerHTML = '';
  for (const tile of data.tiles || []) {
    const node = document.createElement('div');
    node.className = 'tile';
    node.innerHTML =
      `<div class="label"></div><div class="value"></div><div class="note"></div>`;
    node.querySelector('.label').textContent = tile.label;
    node.querySelector('.value').innerHTML =
      `${fmt(tile.value, tile.digits)}${tile.unit ? ` <span class="unit">${tile.unit}</span>` : ''}`;
    node.querySelector('.note').textContent = tile.note || '';
    container.appendChild(node);
  }
  if (!container.children.length) {
    container.innerHTML = '<div class="empty">No metrics recorded for this seed.</div>';
  }
}

/* The trajectory repaints on the first frame and the rest follow over the next
   few, so a seed step reads as immediate instead of one long blocking redraw.
   The token drops stale work when the arrow keys are held down. */
let stageToken = 0;
function reactInStages(pane, specs) {
  const token = ++stageToken;
  pane._stageToken = token;
  Plotly.react(pane._chartNodes[0], specs[0].traces, specs[0].layout, PLOT_CONFIG);
  let index = 1;
  const step = () => {
    if (pane._stageToken !== token) return;
    for (const limit = Math.min(index + 4, specs.length); index < limit; index += 1) {
      Plotly.react(pane._chartNodes[index], specs[index].traces, specs[index].layout, PLOT_CONFIG);
    }
    if (index < specs.length) requestAnimationFrame(step);
  };
  if (specs.length > 1) requestAnimationFrame(step);
}

/* Seed changes reuse the existing plots via Plotly.react -- tearing the charts
   down and rebuilding them is what made stepping through seeds feel slow. */
function renderPane(paneId, data, overlay) {
  const pane = el(`pane-${paneId}`);
  const label = pane.querySelector('.pane-label');
  const charts = pane.querySelector('.charts');
  const argsBox = pane.querySelector('.args');
  if (!data) {
    label.textContent = '';
    charts.innerHTML = '<div class="empty">Nothing selected.</div>';
    pane.querySelector('.tiles').innerHTML = '';
    argsBox.innerHTML = '';
    pane._chartKeys = null;
    return;
  }
  label.textContent = `${data.run} / ${data.seed_dir}`;
  renderTiles(pane.querySelector('.tiles'), data);

  const specs = buildCharts(data, overlay);
  const signature = specs.map((s) => s.key).join(',');
  if (pane._chartKeys === signature && pane._chartNodes) {
    reactInStages(pane, specs);
  } else {
    charts.innerHTML = '';
    const nodes = [];
    const plotted = [];
    for (const spec of specs) {
      const node = document.createElement('div');
      node.className = `chart${spec.tall ? ' tall' : ''}`;
      charts.appendChild(node);
      Plotly.newPlot(node, spec.traces, spec.layout, PLOT_CONFIG);
      nodes.push(node);
      if (spec.time) plotted.push(node);
    }
    pane._chartNodes = nodes;
    pane._chartKeys = signature;
    syncTimeAxes(plotted);
    if (!specs.length) charts.innerHTML = '<div class="empty">No plottable data in this seed.</div>';
  }

  const args = data.args || {};
  const keys = Object.keys(args).sort();
  argsBox.innerHTML = keys.length
    ? keys.map((k) => `<code>${k}=${args[k]}</code>`).join('') +
      (data.manifest && data.manifest.git_sha
        ? ` <span>commit ${String(data.manifest.git_sha).slice(0, 10)}</span>` : '')
    : '';
}

function syncTimeAxes(nodes) {
  let syncing = false;
  for (const node of nodes) {
    node.on('plotly_relayout', (event) => {
      if (syncing) return;
      const range = ('xaxis.range[0]' in event)
        ? [event['xaxis.range[0]'], event['xaxis.range[1]']]
        : (event['xaxis.autorange'] ? null : undefined);
      if (range === undefined) return;
      syncing = true;
      for (const other of nodes) {
        if (other === node) continue;
        Plotly.relayout(other, range ? { 'xaxis.range': range } : { 'xaxis.autorange': true });
      }
      syncing = false;
    });
  }
}

function renderAll() {
  whenPlotly(() => {
    el('panes').classList.toggle('compare', S.compare);
    el('pane-A').classList.toggle('hidden', !S.compare);
    if (S.compare) renderPane('A', S.panes.A, null);
    renderPane('B', S.panes.B, S.compare && S.overlay ? S.panes.A : null);
  });
}

// --- navigation ------------------------------------------------------------

function seedIndex() {
  const current = S.panes.B && S.panes.B.seed_dir;
  return S.seeds.findIndex((s) => s.dir === current);
}

/* Bundles are cached as promises so a prefetch and a navigation asking for the
   same seed share one request. */
const MAX_CACHED_SEEDS = 40;
const bundles = new Map();

function fetchSeed(run, dir) {
  const key = `${run}/${dir}`;
  let pending = bundles.get(key);
  if (pending) {
    bundles.delete(key);            // refresh recency
  } else {
    pending = api(`/api/runs/${encodeURIComponent(run)}/${encodeURIComponent(dir)}`)
      .catch((error) => { bundles.delete(key); throw error; });
  }
  bundles.set(key, pending);
  while (bundles.size > MAX_CACHED_SEEDS) bundles.delete(bundles.keys().next().value);
  return pending;
}

/* Warm every seed of the run in the background, nearest-first, so stepping is
   instant. Superseded as soon as another run is selected. */
let prefetchToken = 0;
async function prefetchRun(run, seeds, from) {
  const token = ++prefetchToken;
  const order = seeds.map((_, i) => i)
    .sort((a, b) => Math.abs(a - from) - Math.abs(b - from));
  let done = 0;
  const queue = order.slice();
  const worker = async () => {
    while (queue.length) {
      if (token !== prefetchToken) return;
      const index = queue.shift();
      try { await fetchSeed(run, seeds[index].dir); } catch (error) { /* skip bad seed */ }
      done += 1;
      if (token === prefetchToken && done < seeds.length) {
        el('cache-state').textContent = `caching ${done}/${seeds.length}`;
      }
    }
  };
  await Promise.all([worker(), worker(), worker()]);
  if (token === prefetchToken) el('cache-state').textContent = `${seeds.length} seeds cached`;
}

let navToken = 0;
async function loadSeed(dir) {
  const run = S.run;
  const token = ++navToken;
  const key = `${run}/${dir}`;
  if (!bundles.has(key)) say(`Loading ${run} / ${dir} ...`);
  el('seed-select').value = dir;
  const seed = S.seeds.find((s) => s.dir === dir) || {};
  const chip = el('seed-status');
  chip.textContent = seed.status || 'unknown';
  chip.className = `chip ${statusClass(seed.status)}`;

  let data;
  try {
    data = await fetchSeed(run, dir);
  } catch (error) {
    if (token === navToken) say(`Could not load ${dir}: ${error.message}`);
    return;
  }
  if (token !== navToken || run !== S.run) return;   // a later key press won
  S.panes.B = data;
  el('replay-start').disabled = !data.has_bag;
  el('replay-start').title = data.has_bag
    ? 'Play this seed\'s rosbag in RViz' : 'No rosbag was recorded for this seed';
  say(`${run} / ${dir}${data.has_bag ? '' : ' -- no rosbag recorded'}`);
  renderAll();
}

function stepSeed(delta) {
  if (!S.seeds.length) return;
  const index = seedIndex();
  const next = ((index < 0 ? 0 : index + delta) + S.seeds.length) % S.seeds.length;
  loadSeed(S.seeds[next].dir);
}

async function loadRun(name) {
  S.run = name;
  say(`Loading ${name} ...`);
  const info = await api(`/api/runs/${encodeURIComponent(name)}`);
  S.seeds = info.seeds || [];
  const select = el('seed-select');
  select.innerHTML = '';
  for (const seed of S.seeds) {
    const option = document.createElement('option');
    option.value = seed.dir;
    option.textContent = seed.dir + (seed.has_bag ? '  (bag)' : '');
    select.appendChild(option);
  }
  el('run-badge').classList.toggle('hidden', !(info.progress && info.progress.state === 'running'));
  el('cache-state').textContent = '';
  window.location.hash = `run=${name}`;
  if (!S.seeds.length) {
    prefetchToken += 1;
    S.panes.B = null;
    say(`${name} has no seed directories.`);
    renderAll();
    return;
  }
  await loadSeed(S.seeds[0].dir);
  prefetchRun(name, S.seeds, 0);
}

// --- replay ----------------------------------------------------------------

function applyReplayStatus(status) {
  const running = status.state === 'running';
  el('replay-stop').disabled = !running;
  const chip = el('replay-state');
  chip.textContent = running ? `playing ${status.seed} at ${status.rate}x` : '';
  chip.className = `chip${running ? ' ok' : ''}`;
}

async function startReplay() {
  if (!S.panes.B) return;
  try {
    applyReplayStatus(await api('/api/replay/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        run: S.panes.B.run,
        seed: S.panes.B.seed_dir,
        rate: Number(el('replay-rate').value),
        loop: el('replay-loop').checked,
      }),
    }));
    say('RViz replay started -- it opens in its own window.');
  } catch (error) {
    say(`Replay failed: ${error.message}`);
  }
}

async function stopReplay() {
  try {
    applyReplayStatus(await api('/api/replay/stop', { method: 'POST' }));
    say('Replay stopped.');
  } catch (error) {
    say(`Could not stop the replay: ${error.message}`);
  }
}

// --- wiring ----------------------------------------------------------------

async function init() {
  const info = await api('/api/runs');
  S.runs = info.runs || [];
  const select = el('run-select');
  for (const run of S.runs) {
    const option = document.createElement('option');
    option.value = run.name;
    option.textContent = run.name + (run.running ? '  (running)' : '');
    select.appendChild(option);
  }
  const wanted = decodeURIComponent((window.location.hash.match(/run=([^&]+)/) || [])[1] || '');
  const initial = S.runs.find((r) => r.name === wanted) || S.runs[0];
  if (!initial) { say(`No runs found under ${info.root}`); return; }
  select.value = initial.name;
  await loadRun(initial.name);
  applyReplayStatus(await api('/api/replay'));
}

el('run-select').addEventListener('change', (e) => loadRun(e.target.value));
el('seed-select').addEventListener('change', (e) => loadSeed(e.target.value));
el('seed-prev').addEventListener('click', () => stepSeed(-1));
el('seed-next').addEventListener('click', () => stepSeed(1));

el('compare-toggle').addEventListener('change', (e) => {
  S.compare = e.target.checked;
  el('pin-btn').classList.toggle('hidden', !S.compare);
  el('overlay-wrap').classList.toggle('hidden', !S.compare);
  if (S.compare && !S.panes.A) S.panes.A = S.panes.B;
  renderAll();
  // The pane columns change width without a window resize, so nudge Plotly.
  setTimeout(() => {
    for (const id of ['pane-A', 'pane-B']) {
      for (const node of el(id)._chartNodes || []) Plotly.Plots.resize(node);
    }
  }, 0);
});

el('pin-btn').addEventListener('click', () => { S.panes.A = S.panes.B; renderAll(); });
el('overlay-toggle').addEventListener('change', (e) => { S.overlay = e.target.checked; renderAll(); });
el('replay-start').addEventListener('click', startReplay);
el('replay-stop').addEventListener('click', stopReplay);

el('theme-btn').addEventListener('click', () => {
  const dark = document.documentElement.dataset.theme === 'dark';
  document.documentElement.dataset.theme = dark ? 'light' : 'dark';
  el('theme-btn').textContent = dark ? 'Dark' : 'Light';
  renderAll();
});

document.addEventListener('keydown', (event) => {
  if (event.target.matches('input, select, textarea')) return;
  if (event.key === 'ArrowRight') { event.preventDefault(); stepSeed(1); }
  if (event.key === 'ArrowLeft') { event.preventDefault(); stepSeed(-1); }
});

setInterval(async () => {
  try { applyReplayStatus(await api('/api/replay')); } catch (error) { /* server gone */ }
}, 4000);

init().catch((error) => say(`Startup failed: ${error.message}`));
