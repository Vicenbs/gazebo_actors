/**
 * Gazebo Actors — Gestor & Secuenciador de Actores (JavaScript)
 * Maneja la telemetría en tiempo real, renderizado de canvas 2D y llamadas REST API.
 */

// Estado global de la aplicación
let appState = {
  selectedActor: "",
  actors: {},
  robot: null,
  simTime: 0.0,
  terrain: {
    x_min: -15.0,
    x_max: 15.0,
    y_min: -15.0,
    y_max: 15.0,
    rows_y: [],
    slope_deg: 0.0
  },
  activeTab: "tab-actors"
};


// Canvas y contexto
let canvas = null;
let ctx = null;
let canvasRect = null;
let teleopInterval = null;

// Inicialización al cargar la ventana
window.addEventListener("DOMContentLoaded", () => {
  if (typeof applyLanguage === "function" && typeof detectSystemLanguage === "function") {
    applyLanguage(detectSystemLanguage());
  }

  initTabs();
  initCanvas();
  initKeyboardListeners();
  
  // Obtener configuración del terreno y arrancar bucle de telemetría (5 Hz)
  fetchTerrain();
  fetchStatus();
  setInterval(fetchStatus, 200);
});

function onLanguageChanged(lang) {
  updateHeaderUI();
  updateActorDropdowns();
  updateActorsCardList();
  updateSequencerTable();
  updateKinematicsPanel();
  drawMap();
}

/* ==========================================================================
   NAVEGACIÓN ENTRE PESTAÑAS
   ========================================================================== */
function initTabs() {
  const tabs = document.querySelectorAll(".nav-tab");
  tabs.forEach(tab => {
    tab.addEventListener("click", () => {
      const targetId = tab.getAttribute("data-tab");
      appState.activeTab = targetId;

      // Actualizar botones
      tabs.forEach(t => t.classList.remove("active"));
      tab.classList.add("active");

      // Actualizar paneles
      document.querySelectorAll(".tab-pane").forEach(pane => pane.classList.remove("active"));
      const activePane = document.getElementById(targetId);
      if (activePane) activePane.classList.add("active");

      // Redibujar mapa si se abre el secuenciador
      if (targetId === "tab-sequencer") {
        requestAnimationFrame(() => {
          resizeCanvas();
          drawMap();
        });
        setTimeout(() => {
          resizeCanvas();
          drawMap();
        }, 50);
      }
    });
  });
}

function switchToTab(tabId, actorName = null) {
  if (actorName) {
    appState.selectedActor = actorName;
    const select1 = document.getElementById("sequencer-actor-select");
    const select2 = document.getElementById("teleop-actor-select");
    if (select1) select1.value = actorName;
    if (select2) select2.value = actorName;
    onSelectedActorChanged();
  }
  const btn = document.querySelector(`.nav-tab[data-tab="${tabId}"]`);
  if (btn) btn.click();
}

/* ==========================================================================
   MAPA 2D INTERACTIVO (CANVAS)
   ========================================================================== */
function resizeCanvas() {
  if (!canvas) return false;
  const container = canvas.parentElement || document.getElementById("canvas-container");
  if (!container) return false;

  const w = container.clientWidth;
  const h = container.clientHeight;

  if (w <= 0 || h <= 0) return false;

  const dpr = window.devicePixelRatio || 1;
  const targetWidth = Math.round(w * dpr);
  const targetHeight = Math.round(h * dpr);

  if (canvas.width !== targetWidth || canvas.height !== targetHeight) {
    canvas.width = targetWidth;
    canvas.height = targetHeight;
  }

  if (ctx) {
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  return true;
}

function initCanvas() {
  canvas = document.getElementById("vineyard-map-canvas");
  if (!canvas) return;
  ctx = canvas.getContext("2d");

  // Ajustar resolución interna al DPI si el contenedor ya tiene tamaño
  resizeCanvas();

  // Escuchar cambio de tamaño de ventana para ajustar el mapa reactivamente
  window.addEventListener("resize", () => {
    if (appState.activeTab === "tab-sequencer") {
      resizeCanvas();
      drawMap();
    }
  });

  // Evento mousemove para mostrar coordenadas
  canvas.addEventListener("mousemove", (e) => {
    const coords = canvasToWorld(e.offsetX, e.offsetY);
    const badge = document.getElementById("canvas-coord-badge");
    if (badge && coords) {
      const z = computeElevation(coords.x, coords.y);
      badge.innerText = `X: ${coords.x.toFixed(2)}m | Y: ${coords.y.toFixed(2)}m | Z: ${z.toFixed(2)}m`;
    }
  });

  // Evento click: Añadir Waypoint interactivo
  canvas.addEventListener("click", (e) => {
    const coords = canvasToWorld(e.offsetX, e.offsetY);
    if (!coords) return;

    // Redondear a múltiplos de 0.25 m
    const x = Math.round(coords.x * 4) / 4;
    const y = Math.round(coords.y * 4) / 4;

    const waitInput = document.getElementById("quick-wp-wait");
    const actSelect = document.getElementById("quick-wp-action");
    const wait = waitInput ? (parseFloat(waitInput.value) || 4.0) : 4.0;
    const act = actSelect ? actSelect.value : "idle";

    addWaypointDirect(x, y, wait, act);
  });
}

// Conversión de coordenadas Mundo (m) <-> Canvas (px)
function worldToCanvas(wx, wy) {
  if (!canvas) return { x: 0, y: 0 };
  const w = canvas.clientWidth > 0 ? canvas.clientWidth : (canvas.width > 0 ? canvas.width / (window.devicePixelRatio || 1) : 800);
  const h = canvas.clientHeight > 0 ? canvas.clientHeight : (canvas.height > 0 ? canvas.height / (window.devicePixelRatio || 1) : 480);
  const padX = 45;
  const padY = 30;
  const usableW = Math.max(10, w - padX * 2);
  const usableH = Math.max(10, h - padY * 2);

  const t = appState.terrain || {};
  const xMin = (t.x_min !== undefined) ? t.x_min : -15.0;
  const xMax = (t.x_max !== undefined) ? t.x_max : 15.0;
  const yMin = (t.y_min !== undefined) ? t.y_min : -15.0;
  const yMax = (t.y_max !== undefined) ? t.y_max : 15.0;

  const spanX = Math.max(0.1, xMax - xMin);
  const spanY = Math.max(0.1, yMax - yMin);

  const cx = padX + ((wx - xMin) / spanX) * usableW;
  const cy = padY + ((yMax - wy) / spanY) * usableH;
  return { x: cx, y: cy };
}

function canvasToWorld(cx, cy) {
  if (!canvas) return null;
  const w = canvas.clientWidth > 0 ? canvas.clientWidth : (canvas.width > 0 ? canvas.width / (window.devicePixelRatio || 1) : 800);
  const h = canvas.clientHeight > 0 ? canvas.clientHeight : (canvas.height > 0 ? canvas.height / (window.devicePixelRatio || 1) : 480);
  const padX = 45;
  const padY = 30;
  const usableW = Math.max(10, w - padX * 2);
  const usableH = Math.max(10, h - padY * 2);

  const t = appState.terrain || {};
  const xMin = (t.x_min !== undefined) ? t.x_min : -15.0;
  const xMax = (t.x_max !== undefined) ? t.x_max : 15.0;
  const yMin = (t.y_min !== undefined) ? t.y_min : -15.0;
  const yMax = (t.y_max !== undefined) ? t.y_max : 15.0;

  const spanX = Math.max(0.1, xMax - xMin);
  const spanY = Math.max(0.1, yMax - yMin);

  const wx = xMin + ((cx - padX) / usableW) * spanX;
  const wy = yMax - ((cy - padY) / usableH) * spanY;

  // Limitar al área válida del entorno
  const clampedX = Math.max(xMin, Math.min(xMax, wx));
  const clampedY = Math.max(yMin, Math.min(yMax, wy));
  return { x: clampedX, y: clampedY };
}

function computeElevation(x, y) {
  if (appState.terrain.terrain_type === "flat" || !appState.terrain.slope_deg || Math.abs(appState.terrain.slope_deg) < 0.1) {
    return 0.0;
  }
  const normX = Math.min(1.0, Math.max(0.0, x / 30.0));
  const maxH = 30.0 * Math.tan((appState.terrain.slope_deg * Math.PI) / 180.0);
  return maxH * Math.pow(normX, 1.8);
}

function drawMap() {
  if (!canvas || !ctx) return;
  if (appState.activeTab !== "tab-sequencer") return;

  resizeCanvas();

  const w = canvas.clientWidth > 0 ? canvas.clientWidth : (canvas.width / (window.devicePixelRatio || 1));
  const h = canvas.clientHeight > 0 ? canvas.clientHeight : (canvas.height / (window.devicePixelRatio || 1));

  if (w <= 0 || h <= 0) return;

  // Fondo
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#151d28";
  ctx.fillRect(0, 0, w, h);

  const t = appState.terrain;

  // 1. Grilla y límites del espacio de trabajo
  const p00 = worldToCanvas(t.x_min, t.y_min);
  const pMax = worldToCanvas(t.x_max, t.y_max);

  ctx.strokeStyle = "#334155";
  ctx.lineWidth = 1;
  ctx.strokeRect(p00.x, pMax.y, pMax.x - p00.x, p00.y - pMax.y);

  const isSlope = (t.terrain_type === "slope" && t.slope_deg > 0.1 && t.rows_y && t.rows_y.length > 0);

  if (isSlope) {
    // 2. Curvas de nivel Z de la pendiente (líneas verticales tenues)
    ctx.strokeStyle = "rgba(100, 116, 139, 0.25)";
    ctx.fillStyle = "rgba(148, 163, 184, 0.4)";
    ctx.font = "9px monospace";
    for (let x = 5.0; x <= 30.0; x += 5.0) {
      const p1 = worldToCanvas(x, t.y_max);
      const p2 = worldToCanvas(x, t.y_min);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();

      const z = computeElevation(x, 0.0);
      ctx.fillText(`Z≈${z.toFixed(1)}m`, p1.x - 12, p1.y - 6);
    }

    // 3. Guías longitudinales del terreno
    ctx.strokeStyle = "#3a5a40";

    ctx.setLineDash([8, 6]);
    ctx.lineWidth = 2;
    t.rows_y.forEach(ry => {
      const rStart = worldToCanvas(t.x_min + 1.0, ry);
      const rEnd = worldToCanvas(t.x_max - 1.0, ry);
      ctx.beginPath();
      ctx.moveTo(rStart.x, rStart.y);
      ctx.lineTo(rEnd.x, rEnd.y);
      ctx.stroke();

      // Pequeños iconos o puntos de cepas de vid
      ctx.fillStyle = "#2d6a4f";
      for (let vx = 3.0; vx < t.x_max - 2.0; vx += 2.5) {
        const vp = worldToCanvas(vx, ry);
        ctx.beginPath();
        ctx.arc(vp.x, vp.y, 2.5, 0, Math.PI * 2);
        ctx.fill();
      }
    });
    ctx.setLineDash([]);
  } else {
    // Cuadrícula cartesiana estándar para simulaciones genéricas
    const step = 5.0;
    ctx.font = "9px monospace";

    for (let gx = Math.ceil(t.x_min / step) * step; gx <= t.x_max; gx += step) {
      const p1 = worldToCanvas(gx, t.y_max);
      const p2 = worldToCanvas(gx, t.y_min);
      ctx.beginPath();
      ctx.strokeStyle = (Math.abs(gx) < 0.001) ? "rgba(56, 189, 248, 0.5)" : "rgba(51, 65, 85, 0.35)";
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
      ctx.fillStyle = "rgba(148, 163, 184, 0.4)";
      ctx.fillText(`${gx.toFixed(0)}m`, p1.x - 8, p1.y - 4);
    }

    for (let gy = Math.ceil(t.y_min / step) * step; gy <= t.y_max; gy += step) {
      const p1 = worldToCanvas(t.x_min, gy);
      const p2 = worldToCanvas(t.x_max, gy);
      ctx.beginPath();
      ctx.strokeStyle = (Math.abs(gy) < 0.001) ? "rgba(56, 189, 248, 0.5)" : "rgba(51, 65, 85, 0.35)";
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
      ctx.fillStyle = "rgba(148, 163, 184, 0.4)";
      ctx.fillText(`${gy.toFixed(0)}m`, p1.x + 4, p1.y + 3);
    }
  }

  // 4. Dibujar waypoints y ruta del actor seleccionado
  const selected = appState.actors[appState.selectedActor];
  if (selected && selected.waypoints && selected.waypoints.length > 0) {
    const wps = selected.waypoints;

    // Línea de ruta que conecta los waypoints
    ctx.strokeStyle = "rgba(245, 158, 11, 0.6)";
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    for (let i = 0; i < wps.length; i++) {
      const pt = worldToCanvas(wps[i].x, wps[i].y);
      if (i === 0) ctx.moveTo(pt.x, pt.y);
      else ctx.lineTo(pt.x, pt.y);
    }
    // Si está en modo bucle, cerrar el camino
    if (selected.mode === "loop" && wps.length > 1) {
      const pt0 = worldToCanvas(wps[0].x, wps[0].y);
      ctx.lineTo(pt0.x, pt0.y);
    }
    ctx.stroke();
    ctx.setLineDash([]);

    // Marcadores numerados de Waypoint
    wps.forEach((wp, idx) => {
      const pt = worldToCanvas(wp.x, wp.y);
      const isTarget = (idx === selected.current_wp_idx);

      // Círculo exterior resaltado si es el objetivo actual
      ctx.beginPath();
      ctx.arc(pt.x, pt.y, isTarget ? 13 : 10, 0, Math.PI * 2);
      ctx.fillStyle = isTarget ? "#f59e0b" : "#475569";
      ctx.fill();
      ctx.strokeStyle = isTarget ? "#fff" : "#94a3b8";
      ctx.lineWidth = 1.5;
      ctx.stroke();

      // Número
      ctx.fillStyle = "#fff";
      ctx.font = "bold 10px sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(`${idx + 1}`, pt.x, pt.y);

      // Etiqueta
      ctx.fillStyle = isTarget ? "#fbbf24" : "#94a3b8";
      ctx.font = "9px sans-serif";
      let wpInfo = `(${wp.x}, ${wp.y})`;
      if (wp.wait_time > 0) {
        let actionStr = "";
        if (wp.action === "dance") actionStr = " [dance]";
        else if (wp.action === "talk") actionStr = " [talk]";
        wpInfo += ` [${wp.wait_time}s${actionStr}]`;
      }
      ctx.fillText(wpInfo, pt.x, pt.y + 16);
    });
  }

  // 5. Dibujar todos los actores presentes
  Object.values(appState.actors).forEach(act => {
    const pt = worldToCanvas(act.x, act.y);
    const isSel = (act.name === appState.selectedActor);

    // Círculo de cuerpo del actor
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, isSel ? 10 : 8, 0, Math.PI * 2);
    ctx.fillStyle = act.color || "#10b981";
    ctx.fill();
    ctx.strokeStyle = isSel ? "#ffffff" : "#0f172a";
    ctx.lineWidth = 2;
    ctx.stroke();

    // Flecha de orientación (rumbo yaw)
    const arrowLen = 14;
    const ax = pt.x + arrowLen * Math.cos(act.yaw);
    const ay = pt.y - arrowLen * Math.sin(act.yaw); // Eje Y invertido en pantalla
    ctx.beginPath();
    ctx.moveTo(pt.x, pt.y);
    ctx.lineTo(ax, ay);
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.stroke();

    // Etiqueta con el nombre y acción en vivo
    ctx.fillStyle = isSel ? "#ffffff" : "#cbd5e1";
    ctx.font = isSel ? "bold 10px sans-serif" : "9px sans-serif";
    ctx.textAlign = "center";
    let actBadge = "";
    if (act.state === "WAITING") {
      if (act.current_action === "dance") actBadge = " (dance)";
      else if (act.current_action === "talk") actBadge = " (talk)";
      else actBadge = " (wait)";
    }
    ctx.fillText(`${act.name}${actBadge}`, pt.x, pt.y - 12);
  });

  // 6. Dibujar el robot (si se recibe odometría)
  if (appState.robot) {

    const rpt = worldToCanvas(appState.robot.x, appState.robot.y);
    
    // Chasis rectangular del robot
    ctx.save();
    ctx.translate(rpt.x, rpt.y);
    ctx.rotate(-appState.robot.yaw);

    ctx.fillStyle = "#2563eb";
    ctx.fillRect(-10, -7, 20, 14);
    ctx.strokeStyle = "#93c5fd";
    ctx.lineWidth = 1.5;
    ctx.strokeRect(-10, -7, 20, 14);

    // Indicador frontal (morro del robot)
    ctx.fillStyle = "#60a5fa";
    ctx.beginPath();
    ctx.moveTo(10, -4);
    ctx.lineTo(15, 0);
    ctx.lineTo(10, 4);
    ctx.fill();

    ctx.restore();

    // Etiqueta robot
    ctx.fillStyle = "#60a5fa";
    ctx.font = "bold 9px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("Robot", rpt.x, rpt.y - 12);
  }

  // 7. Indicador de Orientación / Eje Principal
  const indX = (t.x_min !== undefined) ? t.x_min + 2.0 : 2.0;
  const indY = (t.y_min !== undefined) ? t.y_min + 1.5 : -5.0;
  const slopeIndicatorPt = worldToCanvas(indX, indY);
  ctx.fillStyle = "rgba(148, 163, 184, 0.7)";
  ctx.font = "10px sans-serif";
  ctx.textAlign = "left";
  if (t.terrain_type === "slope" && (t.slope_deg || 0) > 0.1) {
    ctx.fillText("Cuesta Arriba (+X)", slopeIndicatorPt.x, slopeIndicatorPt.y);
  } else {
    ctx.fillText("Eje +X (Avance)", slopeIndicatorPt.x, slopeIndicatorPt.y);
  }
}


/* ==========================================================================
   TELEMETRÍA Y ACTUALIZACIÓN EN VIVO (POLLING 5 HZ)
   ========================================================================== */
async function fetchTerrain() {
  try {
    const res = await fetch("/api/terrain");
    if (res.ok) {
      appState.terrain = await res.json();
      drawMap();
    }
  } catch (err) {
    console.error("Error al obtener terreno:", err);
  }
}

async function fetchStatus() {
  try {
    const res = await fetch("/api/status");
    if (!res.ok) throw new Error("HTTP error " + res.status);
    const data = await res.json();

    // Actualizar estado global
    appState.simTime = data.sim_time;
    appState.robot = data.robot;

    // Indexar actores por nombre
    const actorsMap = {};
    (data.actors || []).forEach(a => {
      actorsMap[a.name] = a;
    });
    appState.actors = actorsMap;

    // Actualizar interfaz
    updateHeaderUI();
    updateActorDropdowns();
    updateActorsCardList();
    updateSequencerTable();
    updateKinematicsPanel();


    // Redibujar canvas 2D
    drawMap();

    // Marcar conexión exitosa
    const pill = document.getElementById("connection-pill");
    const ptext = document.getElementById("connection-text");
    if (pill) pill.className = "status-pill connected";
    if (ptext) ptext.innerText = (typeof t === "function") ? t("sim_status_active") : "Simulación Activa";

  } catch (err) {
    const pill = document.getElementById("connection-pill");
    const ptext = document.getElementById("connection-text");
    if (pill) pill.className = "status-pill";
    if (ptext) ptext.innerText = (typeof t === "function") ? t("sim_status_disconnected") : "Desconectado";
  }
}

function updateHeaderUI() {
  const timeEl = document.getElementById("sim-time-val");
  const countEl = document.getElementById("actors-count-val");
  if (timeEl) timeEl.innerText = `${appState.simTime.toFixed(1)}s`;
  if (countEl) countEl.innerText = `${Object.keys(appState.actors).length}`;
}

function updateActorDropdowns() {
  const sel1 = document.getElementById("sequencer-actor-select");
  const sel2 = document.getElementById("teleop-actor-select");
  const names = Object.keys(appState.actors);

  // Garantizar que selectedActor nunca quede vacío si hay actores presentes
  if (!appState.selectedActor && names.length > 0) {
    appState.selectedActor = names[0];
  } else if (appState.selectedActor && !names.includes(appState.selectedActor)) {
    appState.selectedActor = names.length > 0 ? names[0] : "";
  }

  const noActorsText = (typeof t === "function") ? t("no_actors_option") : "-- Sin actores en simulación --";
  const defaultBadgeText = (typeof t === "function") ? t("actor_badge_default") : "Predeterminado";

  [sel1, sel2].forEach(select => {
    if (!select) return;
    const currentVal = select.value || appState.selectedActor;
    
    // Comparar opciones para no recrear DOM si no cambiaron y no cambió el idioma
    const existingOptions = Array.from(select.options).map(o => o.value);
    const langChanged = select.getAttribute("data-last-lang") !== (typeof currentLanguage !== "undefined" ? currentLanguage : "es");

    if (langChanged || JSON.stringify(existingOptions) !== JSON.stringify(names)) {
      if (typeof currentLanguage !== "undefined") {
        select.setAttribute("data-last-lang", currentLanguage);
      }
      select.innerHTML = "";
      if (names.length === 0) {
        const opt = document.createElement("option");
        opt.value = "";
        opt.innerText = noActorsText;
        select.appendChild(opt);
        appState.selectedActor = "";
      } else {
        names.forEach(name => {
          const opt = document.createElement("option");
          opt.value = name;
          const act = appState.actors[name];
          const disp = act ? (act.display_name || act.name) : name;
          opt.innerText = (act && act.is_default) ? `${disp} (${defaultBadgeText})` : disp;
          select.appendChild(opt);
        });
        if (names.includes(currentVal)) {
          select.value = currentVal;
        } else if (names.length > 0) {
          select.value = names[0];
          appState.selectedActor = names[0];
        }
      }
    } else if (appState.selectedActor && select.value !== appState.selectedActor && names.includes(appState.selectedActor)) {
      select.value = appState.selectedActor;
    }
  });
}

function onSelectedActorChanged() {
  const sel = document.getElementById("sequencer-actor-select");
  if (sel) {
    appState.selectedActor = sel.value;
    const sel2 = document.getElementById("teleop-actor-select");
    if (sel2) sel2.value = sel.value;
    updateSequencerTable();
    drawMap();
  }
}

/* ==========================================================================
   ACTUALIZACIÓN DE COMPONENTES DOM
   ========================================================================== */
function updateActorsCardList() {
  const grid = document.getElementById("actors-card-grid");
  if (!grid) return;

  const actors = Object.values(appState.actors);
  const curLang = typeof currentLanguage !== "undefined" ? currentLanguage : "es";

  // 1. Caso: Sin actores en la simulación
  if (actors.length === 0) {
    const ph = document.getElementById("empty-actors-placeholder");
    if (!ph || ph.getAttribute("data-ph-lang") !== curLang) {
      grid.innerHTML = `
        <div id="empty-actors-placeholder" data-ph-lang="${curLang}" style="grid-column: 1 / -1; text-align: center; padding: 48px 24px; color: #94a3b8; background: rgba(30, 41, 59, 0.45); border-radius: 12px; border: 1px dashed rgba(255,255,255,0.15);">
          <div style="display:flex; justify-content:center; margin-bottom: 12px; color: #64748b;">
            <svg viewBox="0 0 24 24" width="36" height="36" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round">
              <path d="M16 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2"></path>
              <circle cx="12" cy="7" r="4"></circle>
            </svg>
          </div>
          <div style="font-weight: 600; font-size: 16px; margin-bottom: 8px; color: #f1f5f9;">${typeof t === "function" ? t("empty_actors_title") : "No hay actores en la simulación"}</div>
          <div style="font-size: 13px; max-width: 500px; margin: 0 auto; line-height: 1.6; color: #94a3b8;">
            ${typeof t === "function" ? t("empty_actors_desc") : "La simulación está lista. Usa el formulario de la izquierda \"Añadir Nuevo Actor\" para generar un trabajador con colisión física y animación dinámica en Gazebo Sim."}
          </div>
        </div>
      `;
    }
    return;
  }

  // Quitar placeholder vacío si existe
  const placeholder = document.getElementById("empty-actors-placeholder");
  if (placeholder) {
    placeholder.remove();
  }

  const currentNames = new Set(actors.map(a => a.name));

  // 2. Eliminar tarjetas de actores eliminados (solo las que ya no están)
  const existingCards = grid.querySelectorAll(".actor-card");
  existingCards.forEach(card => {
    const aName = card.getAttribute("data-actor-name");
    if (!currentNames.has(aName)) {
      card.remove();
    }
  });

  // 3. Crear o actualizar en sitio las tarjetas existentes (cero destrucción de DOM ni botones)
  actors.forEach(act => {
    let stateClass = "badge-state-idle";
    let stateLabel = act.state;

    if (act.state === "NAVIGATING") {
      stateClass = "badge-state-navigating";
      stateLabel = typeof t === "function" ? t("actor_state_walking") : "Caminando";
    } else if (act.state === "WAITING") {
      stateClass = "badge-state-waiting";
      stateLabel = typeof t === "function" ? t("actor_state_waiting", { time: act.wait_remaining }) : `Inspección (${act.wait_remaining}s)`;
    } else if (act.state === "MANUAL") {
      stateClass = "badge-state-manual";
      stateLabel = typeof t === "function" ? t("actor_state_manual") : "Manual";
    } else if (act.state === "PAUSED") {
      stateClass = "badge-state-paused";
      stateLabel = typeof t === "function" ? t("actor_state_paused") : "Pausado";
    }

    let card = document.getElementById(`actor-card-${act.name}`);

    const strSeq = typeof t === "function" ? t("actor_btn_sequence") : "Secuenciar";
    const strTel = typeof t === "function" ? t("actor_btn_teleop") : "Teleoperar";
    const strDel = typeof t === "function" ? t("actor_btn_delete") : "Eliminar";
    const strDefault = typeof t === "function" ? t("actor_badge_default") : "Predeterminado";
    const strDynamic = typeof t === "function" ? t("actor_badge_dynamic") : "Actor Dinámico";
    const strAlt = typeof t === "function" ? t("actor_card_alt") : "Cota";
    const strHeading = typeof t === "function" ? t("actor_card_heading") : "Rumbo";
    const strSpeed = typeof t === "function" ? t("actor_card_speed") : "Velocidad";
    const strDistRobot = typeof t === "function" ? t("actor_card_dist_robot") : "Dist. Robot";
    const strWaypoints = typeof t === "function" ? t("actor_card_waypoints") : "Waypoints";
    const curLang = typeof currentLanguage !== "undefined" ? currentLanguage : "es";

    if (card && (!card.querySelector(".lbl-heading") || card.getAttribute("data-card-lang") !== curLang)) {
      card.remove();
      card = null;
    }

    if (!card) {
      // Crear tarjeta por primera vez
      card = document.createElement("div");
      card.id = `actor-card-${act.name}`;
      card.className = "actor-card";
      card.setAttribute("data-actor-name", act.name);
      card.setAttribute("data-card-lang", curLang);

      card.innerHTML = `
        <div class="actor-card-header">
          <div class="actor-avatar-badge">
            <div class="actor-circle" style="background:${act.color || '#10b981'}">
              ${act.name.charAt(0).toUpperCase()}
            </div>
            <div>
              <div class="actor-name-title">${act.display_name || act.name}</div>
              <small class="actor-sub-text" style="color:#94a3b8"></small>
            </div>
          </div>
          <span class="badge ${stateClass}">${stateLabel}</span>
        </div>

        <div class="actor-card-body">
          <div><strong>X:</strong> <span class="val-x">${act.x.toFixed(2)}</span> m</div>
          <div><strong>Y:</strong> <span class="val-y">${act.y.toFixed(2)}</span> m</div>
          <div><strong>Z (<span class="lbl-alt">${strAlt}</span>):</strong> <span class="val-z">${act.z.toFixed(2)}</span> m</div>
          <div><strong class="lbl-heading">${strHeading}:</strong> <span class="val-yaw">${(act.yaw * 180 / Math.PI).toFixed(0)}</span>°</div>
          <div><strong class="lbl-speed">${strSpeed}:</strong> <span class="val-vx">${act.vx.toFixed(2)}</span> m/s</div>
          <div class="val-extra"></div>
        </div>

        <div class="actor-card-actions">
          <button class="btn btn-sm btn-outline btn-act-seq" onclick="switchToTab('tab-sequencer', '${act.name}')">${strSeq}</button>
          <button class="btn btn-sm btn-outline btn-act-teleop" onclick="switchToTab('tab-teleop', '${act.name}')">${strTel}</button>
          <button class="btn btn-sm btn-danger-outline btn-act-del" onclick="deleteActor('${act.name}', this)">${strDel}</button>
        </div>
      `;
      grid.appendChild(card);
    }

    // Actualización selectiva de valores dinámicos sin recrear la tarjeta ni los botones
    const badge = card.querySelector(".actor-card-header .badge");
    if (badge) {
      const currentClass = `badge ${stateClass}`;
      if (badge.className !== currentClass) badge.className = currentClass;
      if (badge.innerText !== stateLabel) badge.innerText = stateLabel;
    }

    const subText = card.querySelector(".actor-sub-text");
    if (subText) {
      const gzBadge = act.is_spawned_in_gz ? ' &bull; <span style="color:#10b981">Gazebo OK</span>' : ' &bull; <span style="color:#eab308">Gazebo Sync</span>';
      const newSub = `${act.name !== (act.display_name || act.name) ? act.name + ' &bull; ' : ''}${act.is_default ? strDefault : strDynamic}${gzBadge}`;
      if (subText.innerHTML !== newSub) subText.innerHTML = newSub;
    }

    const btnSeq = card.querySelector(".btn-act-seq");
    if (btnSeq && btnSeq.innerText !== strSeq) btnSeq.innerText = strSeq;
    const btnTel = card.querySelector(".btn-act-teleop");
    if (btnTel && btnTel.innerText !== strTel) btnTel.innerText = strTel;
    const btnDel = card.querySelector(".btn-act-del");
    if (btnDel && !btnDel.disabled && btnDel.innerText !== strDel) btnDel.innerText = strDel;

    const lblAlt = card.querySelector(".lbl-alt");
    if (lblAlt && lblAlt.innerText !== strAlt) lblAlt.innerText = strAlt;
    const lblHeading = card.querySelector(".lbl-heading");
    if (lblHeading && lblHeading.innerText !== `${strHeading}:`) lblHeading.innerText = `${strHeading}:`;
    const lblSpeed = card.querySelector(".lbl-speed");
    if (lblSpeed && lblSpeed.innerText !== `${strSpeed}:`) lblSpeed.innerText = `${strSpeed}:`;

    const elX = card.querySelector(".val-x");
    const strX = act.x.toFixed(2);
    if (elX && elX.innerText !== strX) elX.innerText = strX;

    const elY = card.querySelector(".val-y");
    const strY = act.y.toFixed(2);
    if (elY && elY.innerText !== strY) elY.innerText = strY;

    const elZ = card.querySelector(".val-z");
    const strZ = act.z.toFixed(2);
    if (elZ && elZ.innerText !== strZ) elZ.innerText = strZ;

    const elYaw = card.querySelector(".val-yaw");
    const strYaw = (act.yaw * 180 / Math.PI).toFixed(0);
    if (elYaw && elYaw.innerText !== strYaw) elYaw.innerText = strYaw;

    const elVx = card.querySelector(".val-vx");
    const strVx = act.vx.toFixed(2);
    if (elVx && elVx.innerText !== strVx) elVx.innerText = strVx;

    const elExtra = card.querySelector(".val-extra");
    if (elExtra) {
      const extraHtml = act.distance_to_robot !== null
        ? `<strong>${strDistRobot}:</strong> ${act.distance_to_robot.toFixed(2)} m`
        : `<strong>${strWaypoints}:</strong> ${act.waypoints ? act.waypoints.length : 0}`;
      if (elExtra.innerHTML !== extraHtml) elExtra.innerHTML = extraHtml;
    }
  });
}

function updateSequencerTable() {
  const actor = appState.actors[appState.selectedActor];
  const tbody = document.getElementById("wp-table-body");
  const countText = document.getElementById("wp-count-text");
  const modeBadge = document.getElementById("seq-mode-badge");
  const stateBadge = document.getElementById("seq-state-badge");
  const curLang = typeof currentLanguage !== "undefined" ? currentLanguage : "es";
  const strModePrefix = typeof t === "function" ? t("seq_mode_prefix") : "Modo: ";
  const strModeNone = typeof t === "function" ? t("seq_mode_none") : "Ninguno";

  if (!actor) {
    const emptySeq = document.getElementById("empty-seq-placeholder");
    const noActorTitle = typeof t === "function" ? t("seq_no_actor_title") : "No hay actor seleccionado.";
    const noActorDesc = typeof t === "function" ? t("seq_no_actor_desc") : "Añade o selecciona un actor en la pestaña 'Gestión de Actores' para asignarle waypoints.";
    if (tbody && (!emptySeq || emptySeq.getAttribute("data-ph-lang") !== curLang)) {
      tbody.innerHTML = `<tr id="empty-seq-placeholder" data-ph-lang="${curLang}"><td colspan="7" style="text-align:center;padding:2rem;color:#94a3b8">${noActorTitle}<br><span style="font-size:12px;color:#64748b">${noActorDesc}</span></td></tr>`;
    }
    if (modeBadge) modeBadge.innerText = `${strModePrefix}${strModeNone}`;
    if (stateBadge) {
      const strNoActors = typeof t === "function" ? t("seq_state_no_actors") : "Sin actores";
      stateBadge.innerText = `${typeof t === 'function' ? t('seq_state_prefix') : 'Estado: '}${strNoActors}`;
      stateBadge.className = "badge badge-state-idle";
    }
    if (countText) countText.innerText = "0";
    return;
  }

  // Actualizar badges superiores
  if (modeBadge) {
    let modeText = typeof t === "function" ? t("btn_mode_loop") : "Bucle";
    if (actor.mode === "ping_pong") modeText = typeof t === "function" ? t("btn_mode_pingpong") : "Ida y Vuelta";
    else if (actor.mode === "once") modeText = typeof t === "function" ? t("btn_mode_once") : "Paso Único";
    const fullMode = `${strModePrefix}${modeText}`;
    if (modeBadge.innerText !== fullMode) modeBadge.innerText = fullMode;
  }

  if (stateBadge) {
    let actDesc = "";
    if (actor.state === "WAITING") {
      let icon = typeof t === "function" ? t("action_idle") : "Parado";
      if (actor.current_action === "dance") icon = typeof t === "function" ? t("action_dance") : "Bailando";
      else if (actor.current_action === "talk") icon = typeof t === "function" ? t("action_talk") : "Hablando";
      actDesc = ` (${icon} ${actor.wait_remaining > 0 ? actor.wait_remaining.toFixed(1) + 's' : ''})`;
    }
    const fullState = `${typeof t === 'function' ? t('seq_state_prefix') : 'Estado: '}${actor.state}${actDesc}`;
    const stateClass = `badge badge-state-${actor.state.toLowerCase()}`;
    if (stateBadge.innerText !== fullState) stateBadge.innerText = fullState;
    if (stateBadge.className !== stateClass) stateBadge.className = stateClass;
  }

  // Resaltar botones de modo
  ["loop", "pingpong", "once"].forEach(m => {
    const btn = document.getElementById(`btn-mode-${m}`);
    if (!btn) return;
    const targetMode = m === "pingpong" ? "ping_pong" : m;
    if (actor.mode === targetMode) {
      if (!btn.classList.contains("active")) btn.classList.add("active");
    } else {
      if (btn.classList.contains("active")) btn.classList.remove("active");
    }
  });

  const wpLen = actor.waypoints ? actor.waypoints.length : 0;
  if (countText && countText.innerText !== `${wpLen}`) {
    countText.innerText = `${wpLen}`;
  }

  if (!tbody) return;

  if (wpLen === 0) {
    const noWpPh = document.getElementById("no-wp-placeholder");
    const strNoWp = typeof t === "function" ? t("seq_no_waypoints") : "No hay waypoints asignados.<br>Haz clic en el mapa 2D para añadir uno.";
    if (!noWpPh || noWpPh.getAttribute("data-ph-lang") !== curLang) {
      tbody.innerHTML = `<tr id="no-wp-placeholder" data-ph-lang="${curLang}"><td colspan="8" style="text-align:center;padding:1.5rem;color:#64748b">${strNoWp}</td></tr>`;
    }
    return;
  }

  const noWpPh = document.getElementById("no-wp-placeholder");
  if (noWpPh) noWpPh.remove();
  const emptySeqPh = document.getElementById("empty-seq-placeholder");
  if (emptySeqPh) emptySeqPh.remove();

  const currentWpIds = new Set(actor.waypoints.map(w => w.id));

  // Eliminar filas que ya no existen
  tbody.querySelectorAll("tr[data-wp-id]").forEach(tr => {
    const id = parseInt(tr.getAttribute("data-wp-id"));
    if (!currentWpIds.has(id)) {
      tr.remove();
    }
  });

  const actIdleText = typeof t === "function" ? t("action_idle") : "Parado";
  const actDanceText = typeof t === "function" ? t("action_dance") : "Bailar";
  const actTalkText = typeof t === "function" ? t("action_talk") : "Hablar";

  // Crear o actualizar en sitio cada fila sin destruir botones
  actor.waypoints.forEach((wp, idx) => {
    const isActive = (idx === actor.current_wp_idx && actor.state !== "IDLE");
    let tr = tbody.querySelector(`tr[data-wp-id="${wp.id}"]`);

    if (!tr) {
      tr = document.createElement("tr");
      tr.setAttribute("data-wp-id", wp.id);
      if (isActive) tr.className = "active-wp-row";

      tr.innerHTML = `
        <td style="text-align:center"><strong class="wp-idx">${idx + 1}</strong></td>
        <td class="wp-x">${wp.x.toFixed(2)}</td>
        <td class="wp-y">${wp.y.toFixed(2)}</td>
        <td class="wp-z">${wp.z.toFixed(2)}</td>
        <td style="text-align:center">
          <input type="number" step="0.5" min="0" max="300" value="${wp.wait_time}" class="wp-wait-input" style="width:50px;text-align:center" onchange="updateWaypointProp(${wp.id}, 'wait_time', this.value)"> s
        </td>
        <td>
          <select class="wp-action-select" style="width:105px" onchange="updateWaypointProp(${wp.id}, 'action', this.value)">
            <option value="idle" ${wp.action === 'idle' || !wp.action ? 'selected' : ''}>${actIdleText}</option>
            <option value="dance" ${wp.action === 'dance' ? 'selected' : ''}>${actDanceText}</option>
            <option value="talk" ${wp.action === 'talk' ? 'selected' : ''}>${actTalkText}</option>
          </select>
        </td>
        <td class="wp-speed" style="text-align:center">${wp.speed.toFixed(2)}</td>
        <td style="text-align:center">
          <button class="btn btn-sm btn-danger-outline" style="padding:0.15rem 0.4rem" onclick="deleteWaypoint(${wp.id})">&times;</button>
        </td>
      `;
      tbody.appendChild(tr);
    } else {
      if (isActive && !tr.classList.contains("active-wp-row")) {
        tr.classList.add("active-wp-row");
      } else if (!isActive && tr.classList.contains("active-wp-row")) {
        tr.classList.remove("active-wp-row");
      }

      const elIdx = tr.querySelector(".wp-idx");
      if (elIdx && elIdx.innerText !== `${idx + 1}`) elIdx.innerText = `${idx + 1}`;

      const elX = tr.querySelector(".wp-x");
      const strX = wp.x.toFixed(2);
      if (elX && elX.innerText !== strX) elX.innerText = strX;

      const elY = tr.querySelector(".wp-y");
      const strY = wp.y.toFixed(2);
      if (elY && elY.innerText !== strY) elY.innerText = strY;

      const elZ = tr.querySelector(".wp-z");
      const strZ = wp.z.toFixed(2);
      if (elZ && elZ.innerText !== strZ) elZ.innerText = strZ;

      const elWaitInput = tr.querySelector(".wp-wait-input");
      if (elWaitInput && document.activeElement !== elWaitInput) {
        if (parseFloat(elWaitInput.value) !== wp.wait_time) {
          elWaitInput.value = wp.wait_time;
        }
      }

      const elActionSelect = tr.querySelector(".wp-action-select");
      if (elActionSelect && document.activeElement !== elActionSelect) {
        const curAction = wp.action || "idle";
        if (elActionSelect.value !== curAction) {
          elActionSelect.value = curAction;
        }
      }

      const elSpeed = tr.querySelector(".wp-speed");
      const strSpeed = wp.speed.toFixed(2);
      if (elSpeed && elSpeed.innerText !== strSpeed) elSpeed.innerText = strSpeed;
    }
  });
}

function updateKinematicsPanel() {
  const posEl = document.getElementById("teleop-pos-val");
  const posSub = document.getElementById("teleop-pos-sub");
  const yawEl = document.getElementById("teleop-yaw-val");
  const yawSub = document.getElementById("teleop-yaw-sub");
  const velEl = document.getElementById("teleop-vel-val");
  const angVelSub = document.getElementById("teleop-angvel-sub");
  const stateEl = document.getElementById("teleop-state-val");
  const robotSub = document.getElementById("teleop-robot-sub");
  const zBadge = document.getElementById("teleop-z-badge");
  const animBadge = document.getElementById("teleop-anim-live-badge");
  const idleSelect = document.getElementById("teleop-idle-action-select");

  const actor = appState.actors[appState.selectedActor];
  if (!actor) {
    if (posEl) posEl.innerText = "-- m";
    if (posSub) posSub.innerText = typeof t === "function" ? t("metric_no_actor") : "Sin actor seleccionado";
    if (yawEl) yawEl.innerText = "--°";
    if (yawSub) yawSub.innerText = "-- rad";
    if (velEl) velEl.innerText = "0.00 m/s";
    if (angVelSub) angVelSub.innerText = `${typeof t === 'function' ? t('metric_vel_rot') : 'Rotación: '}0.00 rad/s`;
    if (stateEl) stateEl.innerText = "IDLE";
    if (robotSub) robotSub.innerText = typeof t === "function" ? t("metric_no_robot") : "Robot: Sin odometría activa";
    if (zBadge) zBadge.innerText = "-- m";
    if (animBadge) {
      animBadge.innerText = `${typeof t === 'function' ? t('teleop_anim_badge_prefix') : 'Animación: '}--`;
      animBadge.className = "teleop-anim-live-badge";
    }
    return;
  }

  if (zBadge) zBadge.innerText = `${actor.z.toFixed(2)} m`;
  if (posEl) posEl.innerText = `X: ${actor.x.toFixed(2)} | Y: ${actor.y.toFixed(2)} | Z: ${actor.z.toFixed(2)} m`;
  if (posSub) {
    posSub.innerText = typeof t === "function"
      ? t("teleop_frame_map", { z: actor.z.toFixed(2) })
      : `Marco: map (Cota Z: ${actor.z.toFixed(2)}m)`;
  }

  const yawDeg = (actor.yaw * 180 / Math.PI);
  if (yawEl) yawEl.innerText = `${yawDeg.toFixed(1)}°`;
  if (yawSub) yawSub.innerText = `${actor.yaw.toFixed(2)} rad`;

  if (velEl) velEl.innerText = `${actor.vx.toFixed(2)} m/s`;
  if (angVelSub) angVelSub.innerText = `${typeof t === 'function' ? t('metric_vel_rot') : 'Rotación: '}${(actor.wz || 0.0).toFixed(2)} rad/s`;

  const isMoving = Math.abs(actor.vx) > 0.05 || Math.abs(actor.wz || 0.0) > 0.05;

  if (stateEl) {
    if (actor.state === "MANUAL") {
      const strWalk = typeof t === "function" ? t("actor_state_walking") : "Caminando";
      const strStand = typeof t === "function" ? t("actor_state_standing") : "Parado";
      stateEl.innerText = isMoving ? `MANUAL (${strWalk})` : `MANUAL (${strStand})`;
    } else {
      stateEl.innerText = actor.state;
    }
  }

  if (robotSub) {
    if (actor.distance_to_robot !== null) {
      robotSub.innerText = `${typeof t === 'function' ? t('metric_dist_robot') : 'Distancia al Robot: '}${actor.distance_to_robot.toFixed(2)} m`;
    } else {
      robotSub.innerText = typeof t === "function" ? t("metric_no_robot") : "Robot: Sin odometría activa";
    }
  }

  // Sincronizar selector de animación en reposo
  const activeIdle = (actor.teleop_idle_action || "idle").toLowerCase();
  if (idleSelect && document.activeElement !== idleSelect) {
    if (idleSelect.value !== activeIdle) {
      idleSelect.value = activeIdle;
    }
  }

  // Sincronizar botones rápidos de animación en reposo
  document.querySelectorAll(".teleop-quick-anim-btn").forEach(btn => {
    if (btn.getAttribute("data-anim") === activeIdle) {
      btn.classList.add("btn-primary");
      btn.classList.remove("btn-ghost");
    } else {
      btn.classList.remove("btn-primary");
      btn.classList.add("btn-ghost");
    }
  });

  // Actualizar badge en vivo de animación activa
  if (animBadge) {
    const badgePrefix = typeof t === "function" ? t("teleop_anim_badge_prefix") : "Animación: ";
    if (isMoving) {
      animBadge.innerText = `${badgePrefix}${typeof t === 'function' ? t('teleop_anim_walking') : 'Caminando'}`;
      animBadge.className = "teleop-anim-live-badge moving";
    } else {
      let icon = typeof t === "function" ? t("action_idle") : "Parado";
      const act = actor.current_action || actor.teleop_idle_action || "idle";
      if (act === "dance") icon = typeof t === "function" ? t("action_dance") : "Bailar";
      else if (act === "talk") icon = typeof t === "function" ? t("action_talk") : "Hablar";
      animBadge.innerText = `${badgePrefix}${icon}`;
      animBadge.className = "teleop-anim-live-badge";
    }
  }
}


/* ==========================================================================
   LLAMADAS A LA REST API
   ========================================================================== */
function onCustomColorPick(val) {
  document.querySelectorAll('input[name="actor-color"]').forEach(r => r.checked = false);
}

async function addActorSubmit() {
  const nameInput = document.getElementById("new-actor-name");
  const xInput = document.getElementById("new-actor-x");
  const yInput = document.getElementById("new-actor-y");
  const zInput = document.getElementById("new-actor-z");
  const speedInput = document.getElementById("new-actor-speed");
  const colorInput = document.querySelector('input[name="actor-color"]:checked');
  const customColorInput = document.getElementById("actor-custom-color");
  const selectedColor = colorInput ? colorInput.value : (customColorInput ? customColorInput.value : "#10b981");

  const parsedZ = (zInput && zInput.value.trim() !== "" && !isNaN(parseFloat(zInput.value))) ? parseFloat(zInput.value) : null;

  const payload = {
    name: nameInput.value.trim(),
    x: parseFloat(xInput.value),
    y: parseFloat(yInput.value),
    z: parsedZ,
    speed: parseFloat(speedInput.value),
    color: selectedColor
  };

  try {
    const res = await fetch("/api/actors", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload)
    });
    const data = await res.json();
    if (data.success) {
      nameInput.value = "";
      await fetchStatus();
      switchToTab("tab-sequencer", data.actor ? data.actor.name : payload.name);
    } else {
      alert("Error: " + (data.error || "No se pudo spawnear el actor"));
    }
  } catch (err) {
    alert("Error de conexión al spawnear actor: " + err);
  }
}

async function deleteActor(name, btn = null) {
  if (!btn) {
    const card = document.getElementById(`actor-card-${name}`);
    if (card) btn = card.querySelector(".btn-act-del");
  }

  const confirmMsg = (typeof t === "function") 
    ? t("actor_delete_confirm", { name: name }) 
    : `¿Estás seguro de eliminar el actor '${name}' de la simulación Gazebo?`;
  if (!confirm(confirmMsg)) {
    return;
  }

  if (btn) {
    btn.disabled = true;
    btn.innerHTML = (typeof t === "function") ? t("actor_deleting") : "⏳ Eliminando...";
  }

  // Actualización visual optimista inmediata
  if (appState.actors && appState.actors[name]) {
    delete appState.actors[name];
    if (appState.selectedActor === name) {
      const remaining = Object.keys(appState.actors);
      appState.selectedActor = remaining.length > 0 ? remaining[0] : "";
    }
    updateActorDropdowns();
    updateActorsCardList();
    updateSequencerTable();
    updateKinematicsPanel();
    drawMap();
  }

  try {
    const res = await fetch(`/api/actors/${encodeURIComponent(name)}`, { method: "DELETE" });
    const data = await res.json();
    if (!data.success) {
      alert("Error: " + (data.error || "No se pudo eliminar el actor"));
    }
  } catch (err) {
    alert("Error al eliminar actor: " + err);
  } finally {
    await fetchStatus();
  }
}

async function respawnDefaultActor() {
  try {
    const res = await fetch("/api/actors/reset_default", { method: "POST" });
    const data = await res.json();
    if (data.success) {
      await fetchStatus();
      switchToTab("tab-sequencer", "human_worker");
    } else {
      alert("Error: " + (data.error || "No se pudo regenerar el actor"));
    }
  } catch (err) {
    alert("Error de conexión al regenerar actor: " + err);
  }
}

async function addWaypointDirect(x, y, waitTime = 4.0, action = "idle") {
  if (!appState.selectedActor || !appState.actors[appState.selectedActor]) {
    alert("No hay ningún actor seleccionado. Primero crea o añade un actor en la pestaña 'Gestión de Actores' para poder asignarle waypoints.");
    switchToTab("tab-actors");
    return;
  }

  const actor = appState.actors[appState.selectedActor];
  const speed = actor ? actor.default_speed : 0.65;

  const payload = {
    action: "add",
    x: x,
    y: y,
    z: (appState.terrain.slope_deg && Math.abs(appState.terrain.slope_deg) > 0.1) ? computeElevation(x, y) : (actor ? actor.z : 0.0),
    wait_time: waitTime,
    speed: speed,
    wp_action: action
  };

  try {
    const res = await fetch(`/api/actors/${appState.selectedActor}/waypoints`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload)
    });
    if (res.ok) {
      await fetchStatus();
    }
  } catch (err) {
    console.error("Error al añadir waypoint:", err);
  }
}

async function addManualWaypoint() {
  if (!appState.selectedActor || !appState.actors[appState.selectedActor]) {
    alert("No hay ningún actor seleccionado. Primero añade un actor en la pestaña 'Gestión de Actores'.");
    switchToTab("tab-actors");
    return;
  }

  const x = parseFloat(document.getElementById("quick-wp-x").value);
  const y = parseFloat(document.getElementById("quick-wp-y").value);
  const zInput = document.getElementById("quick-wp-z");
  const rawZ = zInput ? zInput.value.trim() : "";
  const parsedZ = (rawZ !== "" && !isNaN(parseFloat(rawZ))) ? parseFloat(rawZ) : null;
  const wait = parseFloat(document.getElementById("quick-wp-wait").value);
  const actSelect = document.getElementById("quick-wp-action");
  const act = actSelect ? actSelect.value : "idle";
  const speed = parseFloat(document.getElementById("quick-wp-speed").value);

  if (isNaN(x) || isNaN(y)) {
    alert("Coordenadas inválidas");
    return;
  }

  const payload = {
    action: "add",
    x: x,
    y: y,
    z: parsedZ,
    wait_time: isNaN(wait) ? 0.0 : wait,
    speed: isNaN(speed) ? 0.65 : speed,
    wp_action: act
  };

  try {
    await fetch(`/api/actors/${appState.selectedActor}/waypoints`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload)
    });
    await fetchStatus();
  } catch (err) {
    alert("Error al añadir waypoint: " + err);
  }
}

async function updateWaypointProp(wpId, field, value) {
  if (!appState.selectedActor) return;
  const payload = {
    action: "update",
    id: wpId
  };
  if (field === "wait_time") {
    payload.wait_time = parseFloat(value) || 0.0;
  } else if (field === "action") {
    payload.wp_action = value;
  } else if (field === "speed") {
    payload.speed = parseFloat(value) || 0.65;
  }

  try {
    await fetch(`/api/actors/${appState.selectedActor}/waypoints`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload)
    });
    await fetchStatus();
  } catch (err) {
    console.error("Error al actualizar waypoint:", err);
  }
}

async function deleteWaypoint(wpId) {
  try {
    await fetch(`/api/actors/${appState.selectedActor}/waypoints/${wpId}`, {
      method: "DELETE"
    });
    await fetchStatus();
  } catch (err) {
    console.error("Error al eliminar waypoint:", err);
  }
}

async function clearAllWaypoints() {
  const confirmMsg = (typeof t === "function") 
    ? t("confirm_clear_all_wp") 
    : "¿Deseas vaciar todos los waypoints del actor actual?";
  if (!confirm(confirmMsg)) return;
  try {
    await fetch(`/api/actors/${appState.selectedActor}/waypoints`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: "clear" })
    });
    await fetchStatus();
  } catch (err) {
    alert("Error al vaciar waypoints: " + err);
  }
}

async function controlSequence(command) {
  try {
    await fetch(`/api/actors/${appState.selectedActor}/sequence`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command: command })
    });
    await fetchStatus();
  } catch (err) {
    console.error("Error en control de secuencia:", err);
  }
}

async function setMode(mode) {
  if (!appState.selectedActor || !appState.actors[appState.selectedActor]) return;
  try {
    await fetch(`/api/actors/${appState.selectedActor}/sequence`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command: "set_mode", mode: mode })
    });
    await fetchStatus();
  } catch (err) {
    console.error("Error al cambiar modo:", err);
  }
}

async function loadPreset(presetName) {
  if (!appState.selectedActor || !appState.actors[appState.selectedActor]) {
    alert("No hay ningún actor seleccionado. Primero añade un actor en la pestaña 'Gestión de Actores'.");
    switchToTab("tab-actors");
    return;
  }
  try {
    await fetch(`/api/actors/${appState.selectedActor}/sequence`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command: "load_preset", preset: presetName })
    });
    await fetchStatus();
  } catch (err) {
    alert("Error al cargar preset: " + err);
  }
}


/* ==========================================================================
   TELEOPERACIÓN MANUAL 3D (WASD + QE)
   ========================================================================== */
let currentTeleop = { lin: 0.0, ang: 0.0, vert: 0.0 };
let activeTeleopKeys = new Set();
let isDpadActive = false;

function sendTeleopCmd(lin = null, ang = null, vert = null) {
  if (!appState.selectedActor || !appState.actors[appState.selectedActor]) return;
  if (lin !== null) currentTeleop.lin = lin;
  if (ang !== null) currentTeleop.ang = ang;
  if (vert !== null) currentTeleop.vert = vert;

  const idleSelect = document.getElementById("teleop-idle-action-select");
  const idleAction = idleSelect ? idleSelect.value : "idle";

  fetch(`/api/actors/${encodeURIComponent(appState.selectedActor)}/teleop`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      linear: currentTeleop.lin,
      angular: currentTeleop.ang,
      vertical: currentTeleop.vert,
      idle_action: idleAction
    })
  }).catch(() => {});
}

function selectTeleopAnim(anim) {
  const sel = document.getElementById("teleop-idle-action-select");
  if (sel) sel.value = anim;
  onTeleopIdleActionChanged(anim);
}

async function onTeleopIdleActionChanged(val) {
  if (!appState.selectedActor) return;
  const actor = appState.actors[appState.selectedActor];
  if (actor) {
    actor.teleop_idle_action = val;
    if (Math.abs(actor.vx) <= 0.05 && Math.abs(actor.wz || 0.0) <= 0.05) {
      actor.current_action = val;
    }
    updateKinematicsPanel();
  }

  // Desenfocar select para que las teclas de control WASD no queden bloqueadas
  const sel = document.getElementById("teleop-idle-action-select");
  if (sel) sel.blur();

  // Actualizar botones rápidos de animación
  document.querySelectorAll(".teleop-quick-anim-btn").forEach(btn => {
    if (btn.getAttribute("data-anim") === val) {
      btn.classList.add("btn-primary");
      btn.classList.remove("btn-ghost");
    } else {
      btn.classList.remove("btn-primary");
      btn.classList.add("btn-ghost");
    }
  });

  try {
    await fetch(`/api/actors/${encodeURIComponent(appState.selectedActor)}/teleop_idle_action`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: val })
    });
  } catch (err) {
    console.error("Error al actualizar animación en reposo:", err);
  }
}

function updateTeleopFromActiveKeys() {
  const speedSlider = document.getElementById("teleop-speed-slider");
  const v = speedSlider ? parseFloat(speedSlider.value) : 0.70;

  let lin = 0.0;
  let ang = 0.0;
  let vert = 0.0;

  if (activeTeleopKeys.has("w") || activeTeleopKeys.has("arrowup")) lin += v;
  if (activeTeleopKeys.has("s") || activeTeleopKeys.has("arrowdown")) lin -= (v * 0.7);

  if (activeTeleopKeys.has("a") || activeTeleopKeys.has("arrowleft")) ang += 0.9;
  if (activeTeleopKeys.has("d") || activeTeleopKeys.has("arrowright")) ang -= 0.9;

  if (activeTeleopKeys.has("e")) vert += 0.6;
  if (activeTeleopKeys.has("q")) vert -= 0.6;

  if (activeTeleopKeys.size > 0) {
    currentTeleop.lin = lin;
    currentTeleop.ang = ang;
    currentTeleop.vert = vert;

    if (appState.selectedActor && appState.actors[appState.selectedActor]) {
      const actor = appState.actors[appState.selectedActor];
      actor.vx = lin;
      actor.wz = ang;
      actor.current_action = (Math.abs(lin) > 0.05 || Math.abs(ang) > 0.05) ? "walk" : (actor.teleop_idle_action || "idle");
      updateKinematicsPanel();
    }

    sendTeleopCmd();

    if (!teleopInterval) {
      teleopInterval = setInterval(() => {
        sendTeleopCmd();
      }, 100);
    }
  } else if (!isDpadActive) {
    stopAllTeleop();
  }
}

function startTeleop(lin, ang) {
  isDpadActive = true;
  const speedSlider = document.getElementById("teleop-speed-slider");
  const scale = speedSlider ? parseFloat(speedSlider.value) / 0.70 : 1.0;
  const scaledLin = lin * scale;

  currentTeleop.lin = scaledLin;
  currentTeleop.ang = ang;
  sendTeleopCmd();

  // Actualización optimista inmediata
  if (appState.selectedActor && appState.actors[appState.selectedActor]) {
    const actor = appState.actors[appState.selectedActor];
    actor.vx = scaledLin;
    actor.wz = ang;
    actor.current_action = "walk";
    updateKinematicsPanel();
  }

  if (teleopInterval) clearInterval(teleopInterval);
  teleopInterval = setInterval(() => {
    sendTeleopCmd();
  }, 100);
}

function stopTeleop() {
  isDpadActive = false;
  if (activeTeleopKeys.size > 0) {
    updateTeleopFromActiveKeys();
    return;
  }
  currentTeleop.lin = 0.0;
  currentTeleop.ang = 0.0;
  sendTeleopCmd();
  if (Math.abs(currentTeleop.vert) < 0.001 && teleopInterval) {
    clearInterval(teleopInterval);
    teleopInterval = null;
  }
  if (appState.selectedActor && appState.actors[appState.selectedActor]) {
    const actor = appState.actors[appState.selectedActor];
    actor.vx = 0.0;
    actor.wz = 0.0;
    actor.current_action = actor.teleop_idle_action || "idle";
    updateKinematicsPanel();
  }
}

function startTeleopVert(vertSpeed) {
  isDpadActive = true;
  currentTeleop.vert = vertSpeed;
  sendTeleopCmd();

  if (teleopInterval) clearInterval(teleopInterval);
  teleopInterval = setInterval(() => {
    sendTeleopCmd();
  }, 100);
}

function stopTeleopVert() {
  isDpadActive = false;
  if (activeTeleopKeys.size > 0) {
    updateTeleopFromActiveKeys();
    return;
  }
  currentTeleop.vert = 0.0;
  sendTeleopCmd();
  if (Math.abs(currentTeleop.lin) < 0.001 && Math.abs(currentTeleop.ang) < 0.001 && teleopInterval) {
    clearInterval(teleopInterval);
    teleopInterval = null;
  }
}

function stopAllTeleop() {
  activeTeleopKeys.clear();
  isDpadActive = false;
  if (teleopInterval) {
    clearInterval(teleopInterval);
    teleopInterval = null;
  }
  currentTeleop.lin = 0.0;
  currentTeleop.ang = 0.0;
  currentTeleop.vert = 0.0;
  sendTeleopCmd(0.0, 0.0, 0.0);

  // Actualización optimista inmediata
  if (appState.selectedActor && appState.actors[appState.selectedActor]) {
    const actor = appState.actors[appState.selectedActor];
    actor.vx = 0.0;
    actor.wz = 0.0;
    actor.current_action = actor.teleop_idle_action || "idle";
    updateKinematicsPanel();
  }
}

async function stepActorZ(deltaZ) {
  if (!appState.selectedActor || !appState.actors[appState.selectedActor]) return;
  try {
    await fetch(`/api/actors/${appState.selectedActor}/adjust_z`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ step: deltaZ })
    });
    await fetchStatus();
  } catch (err) {
    console.error("Error al ajustar Z:", err);
  }
}

async function resetActorZ() {
  if (!appState.selectedActor || !appState.actors[appState.selectedActor]) return;
  try {
    await fetch(`/api/actors/${appState.selectedActor}/adjust_z`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ reset_ground: true })
    });
    await fetchStatus();
  } catch (err) {
    console.error("Error al resetear cota Z:", err);
  }
}

function initKeyboardListeners() {
  window.addEventListener("keydown", (e) => {
    // Solo responder si no se está escribiendo en un input de texto
    const tag = e.target.tagName;
    const isTextInput = tag === "TEXTAREA" || (tag === "INPUT" && ["text", "number", "password", "email", "search"].includes(e.target.type));
    if (isTextInput) return;

    const k = e.key.toLowerCase();
    const teleopKeys = ["w", "a", "s", "d", "q", "e", "arrowup", "arrowdown", "arrowleft", "arrowright", " ", "x"];
    if (teleopKeys.includes(k)) {
      if (tag === "INPUT" || tag === "SELECT" || tag === "BUTTON") {
        e.target.blur();
      }
      e.preventDefault();

      if (k === " " || k === "x") {
        stopAllTeleop();
        return;
      }

      if (!activeTeleopKeys.has(k)) {
        activeTeleopKeys.add(k);
        updateTeleopFromActiveKeys();
      }
    }
  });

  window.addEventListener("keyup", (e) => {
    const tag = e.target.tagName;
    const isTextInput = tag === "TEXTAREA" || (tag === "INPUT" && ["text", "number", "password", "email", "search"].includes(e.target.type));
    if (isTextInput) return;

    const k = e.key.toLowerCase();
    if (activeTeleopKeys.has(k)) {
      activeTeleopKeys.delete(k);
      updateTeleopFromActiveKeys();
    }
  });

  window.addEventListener("blur", () => {
    if (activeTeleopKeys.size > 0 || isDpadActive) {
      stopAllTeleop();
    }
  });

  window.addEventListener("mouseup", () => {
    if (isDpadActive) {
      isDpadActive = false;
      if (activeTeleopKeys.size === 0) {
        stopAllTeleop();
      } else {
        updateTeleopFromActiveKeys();
      }
    }
  });

  window.addEventListener("touchend", () => {
    if (isDpadActive) {
      isDpadActive = false;
      if (activeTeleopKeys.size === 0) {
        stopAllTeleop();
      } else {
        updateTeleopFromActiveKeys();
      }
    }
  });
}

/* Helpers para Sliders */
function updateRangeText(rangeId, textId) {
  const r = document.getElementById(rangeId);
  const t = document.getElementById(textId);
  if (r && t) t.value = r.value;
}

function updateNumberRange(textId, rangeId) {
  const t = document.getElementById(textId);
  const r = document.getElementById(rangeId);
  if (t && r) r.value = t.value;
}

/* ==========================================================================
   PERSISTENCIA Y GESTIÓN DE CONFIGURACIONES (GUARDAR Y CARGAR)
   ========================================================================== */

function showToast(message, duration = 3500) {
  const container = document.getElementById("toast-container");
  if (!container) return;
  const toast = document.createElement("div");
  toast.className = "toast-msg";
  toast.innerHTML = message;
  container.appendChild(toast);
  setTimeout(() => {
    toast.style.opacity = "0";
    toast.style.transition = "opacity 0.3s ease-out";
    setTimeout(() => toast.remove(), 300);
  }, duration);
}

function openSaveConfigModal() {
  const modal = document.getElementById("modal-save-config");
  if (modal) modal.style.display = "flex";
  const nameInput = document.getElementById("preset-save-name");
  if (nameInput) {
    const dateStr = new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19);
    nameInput.value = `escenario_${dateStr}.json`;
  }
}

function closeSaveConfigModal() {
  const modal = document.getElementById("modal-save-config");
  if (modal) modal.style.display = "none";
}

function openLoadConfigModal() {
  const modal = document.getElementById("modal-load-config");
  if (modal) modal.style.display = "flex";
  fetchPresetsList();
}

function closeLoadConfigModal() {
  const modal = document.getElementById("modal-load-config");
  if (modal) modal.style.display = "none";
}

async function fetchPresetsList() {
  const sel = document.getElementById("preset-config-select");
  if (!sel) return;
  sel.innerHTML = `<option value="">${typeof t === "function" ? t("loading_presets") : "-- Cargando presets disponibles... --"}</option>`;
  try {
    const res = await fetch("/api/config/presets");
    const data = await res.json();
    sel.innerHTML = "";
    const presets = data.presets || [];
    if (presets.length === 0) {
      sel.innerHTML = `<option value="">${typeof t === "function" ? t("no_presets") : "-- No hay presets guardados en el servidor --"}</option>`;
      return;
    }
    presets.forEach(p => {
      const opt = document.createElement("option");
      opt.value = p.filename;
      let label = p.filename;
      if (p.description) label += ` (${p.description})`;
      else if (p.actors_count) label += ` [${p.actors_count} ${typeof t === "function" ? t("actors_count_label") : "actores"}]`;
      opt.innerText = label;
      sel.appendChild(opt);
    });
  } catch (err) {
    const errText = typeof t === "function" ? t("error_loading_presets") : "Error al cargar lista de presets";
    sel.innerHTML = `<option value="">-- ${errText} --</option>`;
  }
}

async function downloadScenarioJson() {
  try {
    const res = await fetch("/api/config/export");
    if (!res.ok) throw new Error("HTTP error " + res.status);
    const data = await res.json();
    const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(data, null, 2));
    const downloadAnchor = document.createElement("a");
    downloadAnchor.setAttribute("href", dataStr);
    const dateStr = new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19);
    downloadAnchor.setAttribute("download", `gazebo_actors_scenario_${dateStr}.json`);
    document.body.appendChild(downloadAnchor);
    downloadAnchor.click();
    downloadAnchor.remove();
    showToast("Archivo JSON de configuración descargado con éxito");
    closeSaveConfigModal();
  } catch (err) {
    alert("Error al descargar configuración: " + err.message);
  }
}

async function savePresetOnServer() {
  const input = document.getElementById("preset-save-name");
  let name = input ? input.value.trim() : "";
  if (!name) name = "escenario_" + Date.now() + ".json";
  if (!name.endsWith(".json") && !name.endsWith(".yaml") && !name.endsWith(".yml")) {
    name += ".json";
  }

  try {
    const res = await fetch("/api/config/presets/save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ filename: name })
    });
    const result = await res.json();
    if (result.success) {
      showToast(result.message || "Preset guardado con éxito");
      closeSaveConfigModal();
    } else {
      alert("Error al guardar preset: " + (result.error || result.message));
    }
  } catch (err) {
    alert("Error al comunicar con servidor: " + err.message);
  }
}

function triggerFileInput() {
  const fileInput = document.getElementById("config-file-hidden-input");
  if (fileInput) fileInput.click();
}

function handleFileSelected(event) {
  const file = event.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = async (e) => {
    try {
      const configData = JSON.parse(e.target.result);
      await importScenarioPayload(configData);
    } catch (err) {
      alert("Error al parsear el archivo JSON: " + err.message);
    }
  };
  reader.readAsText(file);
  event.target.value = "";
}

async function importScenarioPayload(configData) {
  try {
    showToast("Sincronizando nuevo escenario con Gazebo Sim...", 3000);
    const res = await fetch("/api/config/import", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(configData)
    });
    const result = await res.json();
    if (result.success) {
      await fetchStatus();
      closeLoadConfigModal();
      showToast(result.message || "Escenario cargado con éxito");
      const actorKeys = Object.keys(appState.actors);
      if (actorKeys.length > 0) {
        selectActor(actorKeys[0]);
      }
      drawMap();
    } else {
      alert("Error al importar escenario: " + (result.error || result.message));
    }
  } catch (err) {
    alert("Error de conexión al cargar escenario: " + err.message);
  }
}

async function loadSelectedPreset() {
  const sel = document.getElementById("preset-config-select");
  const filename = sel ? sel.value : "";
  if (!filename) {
    alert("Por favor selecciona un preset de la lista.");
    return;
  }
  try {
    showToast("Cargando preset y sincronizando actores en Gazebo...", 3000);
    const res = await fetch("/api/config/presets/load", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ filename: filename })
    });
    const result = await res.json();
    if (result.success) {
      await fetchStatus();
      closeLoadConfigModal();
      showToast(result.message || "Preset cargado con éxito");
      const actorKeys = Object.keys(appState.actors);
      if (actorKeys.length > 0) {
        selectActor(actorKeys[0]);
      }
      drawMap();
    } else {
      alert("Error al cargar preset: " + (result.error || result.message));
    }
  } catch (err) {
    alert("Error al cargar preset: " + err.message);
  }
}


