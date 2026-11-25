

import { sendCommand, appendLog, showToast, escapeHtml } from "./ble.js";
import { servoAngles, applyReceivedAngles } from "./servos.js";

const DEFAULT_ACTIONS = {
  1: { name: "Home", steps: [{ s1: 90, s2: 90, s3: 90, s4: 90, move: 500, wait: 0 }] },
  2: { name: "Open & Lift", steps: [{ s1: 90, s2: 60, s3: 170, s4: 180, move: 400, wait: 0 }] },
  3: { name: "Grab & Lower", steps: [{ s1: 90, s2: 120, s3: 20, s4: 0, move: 400, wait: 0 }] },
  4: {
    name: "Pick & Place",
    steps: [
      { s1: 45, s2: 130, s3: 160, s4: 0, move: 500, wait: 200 },
      { s1: 45, s2: 130, s3: 25, s4: 0, move: 400, wait: 300 },
      { s1: 45, s2: 60, s3: 25, s4: 120, move: 500, wait: 200 },
      { s1: 135, s2: 60, s3: 25, s4: 120, move: 600, wait: 200 },
      { s1: 135, s2: 130, s3: 25, s4: 0, move: 500, wait: 200 },
      { s1: 135, s2: 130, s3: 160, s4: 0, move: 400, wait: 200 },
      { s1: 90, s2: 90, s3: 90, s4: 90, move: 500, wait: 0 }
    ]
  },
  5: {
    name: "Wave",
    steps: [
      { s1: 90, s2: 45, s3: 90, s4: 90, move: 400, wait: 100 },
      { s1: 45, s2: 45, s3: 150, s4: 150, move: 300, wait: 150 },
      { s1: 135, s2: 45, s3: 30, s4: 30, move: 300, wait: 150 },
      { s1: 45, s2: 45, s3: 150, s4: 150, move: 300, wait: 150 },
      { s1: 135, s2: 45, s3: 30, s4: 30, move: 300, wait: 150 },
      { s1: 90, s2: 90, s3: 90, s4: 90, move: 400, wait: 0 }
    ]
  },
  6: {
    name: "Sweep",
    steps: [
      { s1: 10, s2: 90, s3: 90, s4: 45, move: 600, wait: 200 },
      { s1: 170, s2: 90, s3: 90, s4: 135, move: 800, wait: 200 },
      { s1: 90, s2: 90, s3: 90, s4: 90, move: 600, wait: 0 }
    ]
  },
  7: { name: "Rest", steps: [{ s1: 90, s2: 160, s3: 10, s4: 0, move: 600, wait: 0 }] },
  8: {
    name: "Nod",
    steps: [
      { s1: 90, s2: 40, s3: 90, s4: 90, move: 350, wait: 150 },
      { s1: 90, s2: 110, s3: 90, s4: 120, move: 350, wait: 150 },
      { s1: 90, s2: 40, s3: 90, s4: 90, move: 350, wait: 150 },
      { s1: 90, s2: 90, s3: 90, s4: 90, move: 400, wait: 0 }
    ]
  }
};

let armActions = {};
let actionTapMode = "run";
let editingSlot = 1;
let editorSteps = [];
let activeRunningSlot = 0;
let activeRunningState = false;
let activeLoopState = false;

function cloneDefaultActions() {
  return JSON.parse(JSON.stringify(DEFAULT_ACTIONS));
}

function initActionsStorage() {
  try {
    const stored = localStorage.getItem("arm_scissor_actions_v2");
    armActions = stored ? JSON.parse(stored) : cloneDefaultActions();
  } catch (e) {
    armActions = cloneDefaultActions();
  }
  renderActionsGrid();
}

function persistActionsLocal() {
  try {
    localStorage.setItem("arm_scissor_actions_v2", JSON.stringify(armActions));
  } catch (e) {}
}

function stepsToString(steps) {
  return steps
    .map(st => `${clampAngle(st.s1)},${clampAngle(st.s2)},${clampAngle(st.s3)},${clampAngle(st.s4 !== undefined ? st.s4 : 90)},${clampMs(st.move, 10000)},${clampMs(st.wait, 30000)}`)
    .join(";");
}

function stringToSteps(str) {
  if (!str) return [];
  return str
    .split(";")
    .map(tok => tok.trim())
    .filter(Boolean)
    .map(tok => {
      const p = tok.split(",").map(n => parseInt(n.trim(), 10));
      if (p.length >= 6) {
        return {
          s1: clampAngle(isNaN(p[0]) ? 90 : p[0]),
          s2: clampAngle(isNaN(p[1]) ? 90 : p[1]),
          s3: clampAngle(isNaN(p[2]) ? 90 : p[2]),
          s4: clampAngle(isNaN(p[3]) ? 90 : p[3]),
          move: clampMs(isNaN(p[4]) ? 400 : p[4], 10000),
          wait: clampMs(isNaN(p[5]) ? 0 : p[5], 30000)
        };
      }
      return {
        s1: clampAngle(isNaN(p[0]) ? 90 : p[0]),
        s2: clampAngle(isNaN(p[1]) ? 90 : p[1]),
        s3: clampAngle(isNaN(p[2]) ? 90 : p[2]),
        s4: 90,
        move: clampMs(isNaN(p[3]) ? 400 : p[3], 10000),
        wait: clampMs(isNaN(p[4]) ? 0 : p[4], 30000)
      };
    });
}

function clampAngle(v) {
  const n = parseInt(v, 10);
  if (isNaN(n)) return 90;
  return Math.max(0, Math.min(180, n));
}

function clampMs(v, maxVal) {
  const n = parseInt(v, 10);
  if (isNaN(n)) return 0;
  return Math.max(0, Math.min(maxVal, n));
}

function renderActionsGrid() {
  const grid = document.getElementById("actions-grid");
  if (!grid) return;
  grid.innerHTML = "";

  for (let slot = 1; slot <= 8; slot++) {
    const act = armActions[slot] || DEFAULT_ACTIONS[slot];
    const card = document.createElement("div");
    card.id = `act-card-${slot}`;
    card.className = "btn-card";

    if (activeRunningState && activeRunningSlot === slot) {
      card.classList.add("running-btn");
    }
    if (actionTapMode === "edit" && editingSlot === slot) {
      card.classList.add("selected-btn");
    }

    card.onclick = () => onActionCardClick(slot);

    const stepCount = act.steps ? act.steps.length : 0;
    const modeTag = (activeRunningState && activeRunningSlot === slot)
      ? (activeLoopState ? "LOOPING" : "RUNNING")
      : `${stepCount} step${stepCount === 1 ? "" : "s"}`;

    card.innerHTML = `
      <div style="display:flex; justify-content:space-between; align-items:center; width:100%;">
        <span style="font-size:12px; font-weight:700; color:#f8fafc; overflow:hidden; text-overflow:ellipsis; white-space:nowrap;">${escapeHtml(act.name)}</span>
        <span style="font-size:9px; font-family:monospace; color:#38bdf8;">#${slot}</span>
      </div>
      <div style="font-size:9.5px; color:rgba(255,255,255,0.55);">${modeTag}</div>
    `;
    grid.appendChild(card);
  }
}

function setActionTapMode(mode) {
  actionTapMode = mode;
  const runBtn = document.getElementById("mode-btn-run");
  const loopBtn = document.getElementById("mode-btn-loop");
  const editBtn = document.getElementById("mode-btn-edit");
  const hint = document.getElementById("action-mode-hint");
  const editor = document.getElementById("action-editor-box");

  if (runBtn) runBtn.className = "mode-pill" + (mode === "run" ? " active" : "");
  if (loopBtn) loopBtn.className = "mode-pill" + (mode === "loop" ? " active" : "");
  if (editBtn) editBtn.className = "mode-pill" + (mode === "edit" ? " active-edit" : "");

  if (mode === "run") {
    if (hint) hint.innerText = "Tap an action to run its programmed steps once.";
    if (editor) editor.style.display = "none";
  } else if (mode === "loop") {
    if (hint) hint.innerText = "Tap an action to loop continuously until Stop.";
    if (editor) editor.style.display = "none";
  } else {
    if (hint) hint.innerText = "Tap any slot to program 4-axis steps, speeds, and delays.";
    openActionEditor(editingSlot);
  }
  renderActionsGrid();
}

function onActionCardClick(slot) {
  const card = document.getElementById(`act-card-${slot}`);
  if (card && actionTapMode !== "edit") {
    card.classList.add("pressed");
    setTimeout(() => card.classList.remove("pressed"), 140);
  }

  if (actionTapMode === "run") {
    sendCommand(`action_run ${slot}`);
    appendLog("Action", `Run #${slot} (${armActions[slot]?.name || ""})`);
  } else if (actionTapMode === "loop") {
    sendCommand(`action_loop ${slot}`);
    appendLog("Action", `Loop #${slot} (${armActions[slot]?.name || ""})`);
  } else {
    openActionEditor(slot);
  }
}

function openActionEditor(slot) {
  editingSlot = slot;
  const act = armActions[slot] || DEFAULT_ACTIONS[slot];
  editorSteps = JSON.parse(JSON.stringify(act.steps || []));

  const title = document.getElementById("action-editor-title");
  if (title) title.innerText = `Program Action Slot #${slot}`;
  const nameInp = document.getElementById("action-name-input");
  if (nameInp) nameInp.value = act.name || `Action ${slot}`;
  const editorBox = document.getElementById("action-editor-box");
  if (editorBox) editorBox.style.display = "flex";

  renderEditorSteps();
  renderActionsGrid();
}

function closeActionEditor() {
  setActionTapMode("run");
}

function renderEditorSteps() {
  const list = document.getElementById("action-steps-list");
  if (!list) return;
  list.innerHTML = "";

  if (editorSteps.length === 0) {
    list.innerHTML = '<div style="font-size:11px; color:rgba(255,255,255,0.45); text-align:center; padding:12px;">No steps yet. Tap "+ Capture Pose" to add current arm &amp; scissor pose.</div>';
    return;
  }

  editorSteps.forEach((st, idx) => {
    const s4Val = (st.s4 !== undefined) ? st.s4 : 90;
    const row = document.createElement("div");
    row.className = "step-row";
    row.innerHTML = `
      <div style="display:flex; justify-content:space-between; align-items:center;">
        <span style="font-size:10px; font-weight:700; color:#38bdf8;">Step ${idx + 1}</span>
        <div style="display:flex; gap:6px;">
          <button type="button" onclick="previewStep(${idx})" style="background:#0284c7; color:white; border:none; border-radius:4px; padding:2px 7px; font-size:9.5px; font-weight:700; cursor:pointer;">Preview</button>
          <button type="button" onclick="moveStep(${idx}, -1)" style="background:#334155; color:white; border:none; border-radius:4px; padding:3px 6px; font-size:9.5px; cursor:pointer; display:inline-flex; align-items:center;"><svg width="10" height="10" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"><line x1="12" y1="19" x2="12" y2="5"></line><polyline points="5 12 12 5 19 12"></polyline></svg></button>
          <button type="button" onclick="moveStep(${idx}, 1)" style="background:#334155; color:white; border:none; border-radius:4px; padding:3px 6px; font-size:9.5px; cursor:pointer; display:inline-flex; align-items:center;"><svg width="10" height="10" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"><line x1="12" y1="5" x2="12" y2="19"></line><polyline points="19 12 12 19 5 12"></polyline></svg></button>
          <button type="button" onclick="deleteStep(${idx})" style="background:#ef4444; color:white; border:none; border-radius:4px; padding:3px 6px; font-size:9.5px; font-weight:700; cursor:pointer; display:inline-flex; align-items:center;"><svg width="10" height="10" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"><line x1="18" y1="6" x2="6" y2="18"></line><line x1="6" y1="6" x2="18" y2="18"></line></svg></button>
        </div>
      </div>
      <div class="step-inputs">
        <div class="step-field">
          <label>S1 Claw</label>
          <input type="number" min="0" max="180" value="${st.s1}" onchange="updateStepField(${idx}, 's1', this.value)">
        </div>
        <div class="step-field">
          <label>S2 Arm</label>
          <input type="number" min="0" max="180" value="${st.s2}" onchange="updateStepField(${idx}, 's2', this.value)">
        </div>
        <div class="step-field">
          <label>S3 Shld</label>
          <input type="number" min="0" max="180" value="${st.s3}" onchange="updateStepField(${idx}, 's3', this.value)">
        </div>
        <div class="step-field">
          <label style="color:#fbbf24;">S4 Scis</label>
          <input type="number" min="0" max="180" value="${s4Val}" onchange="updateStepField(${idx}, 's4', this.value)">
        </div>
        <div class="step-field">
          <label>Move ms</label>
          <input type="number" min="0" max="10000" step="50" value="${st.move}" onchange="updateStepField(${idx}, 'move', this.value)">
        </div>
        <div class="step-field">
          <label>Wait ms</label>
          <input type="number" min="0" max="30000" step="50" value="${st.wait}" onchange="updateStepField(${idx}, 'wait', this.value)">
        </div>
      </div>
    `;
    list.appendChild(row);
  });
}

function captureCurrentPoseStep() {
  if (editorSteps.length >= 24) {
    showToast("Max 24 steps per action.", "warn");
    return;
  }
  editorSteps.push({
    s1: servoAngles[0],
    s2: servoAngles[1],
    s3: servoAngles[2],
    s4: servoAngles[3],
    move: 450,
    wait: 150
  });
  renderEditorSteps();
  showToast(`Captured (${servoAngles[0]}°, ${servoAngles[1]}°, ${servoAngles[2]}°, ${servoAngles[3]}°)`, "info");
}

function addBlankStep() {
  if (editorSteps.length >= 24) {
    showToast("Max 24 steps per action.", "warn");
    return;
  }
  editorSteps.push({ s1: 90, s2: 90, s3: 90, s4: 90, move: 450, wait: 150 });
  renderEditorSteps();
}

function updateStepField(idx, field, val) {
  if (!editorSteps[idx]) return;
  if (field === "s1" || field === "s2" || field === "s3" || field === "s4") {
    editorSteps[idx][field] = clampAngle(val);
  } else if (field === "move") {
    editorSteps[idx].move = clampMs(val, 10000);
  } else if (field === "wait") {
    editorSteps[idx].wait = clampMs(val, 30000);
  }
}

function previewStep(idx) {
  const st = editorSteps[idx];
  if (!st) return;
  const s4 = (st.s4 !== undefined) ? st.s4 : 90;
  sendCommand(`move ${st.s1} ${st.s2} ${st.s3} ${s4} ${st.move}`);
  applyReceivedAngles(st.s1, st.s2, st.s3, s4);
}

function moveStep(idx, dir) {
  const target = idx + dir;
  if (target < 0 || target >= editorSteps.length) return;
  const tmp = editorSteps[idx];
  editorSteps[idx] = editorSteps[target];
  editorSteps[target] = tmp;
  renderEditorSteps();
}

function deleteStep(idx) {
  editorSteps.splice(idx, 1);
  renderEditorSteps();
}

function testEditorSequence() {
  if (editorSteps.length === 0) {
    showToast("Add at least one step first.", "error");
    return;
  }
  const str = stepsToString(editorSteps);
  sendCommand(`seq ${str}`);
  appendLog("Action", `Testing sequence (${editorSteps.length} steps)`);
}

async function saveEditorAction() {
  const nameInput = document.getElementById("action-name-input");
  let name = (nameInput?.value || "").replace(/\|/g, "").trim();
  if (!name) name = `Action ${editingSlot}`;
  if (editorSteps.length === 0) {
    showToast("Add at least one step before saving.", "error");
    return;
  }

  const stepsCopy = JSON.parse(JSON.stringify(editorSteps));
  armActions[editingSlot] = { name, steps: stepsCopy };
  persistActionsLocal();
  renderActionsGrid();

  const payload = `${name}|${stepsToString(stepsCopy)}`;
  await sendCommand(`action_set ${editingSlot} ${payload}`);
  showToast(`Saved "${name}" to Slot #${editingSlot}`, "success");
  appendLog("Action", `Saved Slot #${editingSlot}: ${name}`);
}

function stopAllMotion() {
  sendCommand("stop");
  updateActionRunBadge(false, 0, false);
  showToast("Stopped motion", "warn");
}

function syncActionsFromEsp() {
  sendCommand("action_get");
  showToast("Syncing programmable actions...", "info");
}

function resetActionsDefaults() {
  armActions = cloneDefaultActions();
  persistActionsLocal();
  renderActionsGrid();
  if (actionTapMode === "edit") openActionEditor(editingSlot);
  sendCommand("action_reset");
  showToast("Factory actions restored", "info");
}

function applyReceivedAction(rawDef) {
  const eqIdx = rawDef.indexOf("=");
  if (eqIdx <= 0) return;
  const slot = parseInt(rawDef.substring(0, eqIdx), 10);
  if (isNaN(slot) || slot < 1 || slot > 8) return;

  const rest = rawDef.substring(eqIdx + 1);
  const pipeIdx = rest.indexOf("|");
  if (pipeIdx <= 0) return;

  const name = rest.substring(0, pipeIdx).trim();
  const steps = stringToSteps(rest.substring(pipeIdx + 1));
  if (steps.length > 0) {
    armActions[slot] = { name, steps };
    persistActionsLocal();
    renderActionsGrid();
    if (actionTapMode === "edit" && editingSlot === slot) {
      openActionEditor(slot);
    }
  }
}

function updateActionRunBadge(running, slot, loop) {
  activeRunningState = running;
  activeRunningSlot = slot;
  activeLoopState = loop;

  const badge = document.getElementById("action-run-badge");
  if (badge) {
    if (running) {
      const label = slot > 0 ? (armActions[slot]?.name || `Slot #${slot}`) : "Sequence";
      badge.innerText = loop ? `Looping: ${label}` : `Running: ${label}`;
      badge.style.background = "rgba(16, 185, 129, 0.2)";
      badge.style.color = "#34d399";
    } else {
      badge.innerText = "Idle";
      badge.style.background = "rgba(255,255,255,0.08)";
      badge.style.color = "rgba(255,255,255,0.5)";
    }
  }
  renderActionsGrid();
}

window.initActionsStorage = initActionsStorage;
window.setActionTapMode = setActionTapMode;
window.onActionCardClick = onActionCardClick;
window.openActionEditor = openActionEditor;
window.closeActionEditor = closeActionEditor;
window.captureCurrentPoseStep = captureCurrentPoseStep;
window.addBlankStep = addBlankStep;
window.updateStepField = updateStepField;
window.previewStep = previewStep;
window.moveStep = moveStep;
window.deleteStep = deleteStep;
window.testEditorSequence = testEditorSequence;
window.saveEditorAction = saveEditorAction;
window.stopAllMotion = stopAllMotion;
window.syncActionsFromEsp = syncActionsFromEsp;
window.resetActionsDefaults = resetActionsDefaults;
window.applyReceivedAction = applyReceivedAction;
window.updateActionRunBadge = updateActionRunBadge;

export {
  initActionsStorage,
  setActionTapMode,
  openActionEditor,
  closeActionEditor,
  captureCurrentPoseStep,
  addBlankStep,
  updateStepField,
  previewStep,
  moveStep,
  deleteStep,
  testEditorSequence,
  saveEditorAction,
  stopAllMotion,
  syncActionsFromEsp,
  resetActionsDefaults,
  applyReceivedAction,
  updateActionRunBadge
};

