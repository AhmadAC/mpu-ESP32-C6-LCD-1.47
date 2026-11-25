let servoAngles = [90, 90, 90, 90];
let servoOffsets = [0, 0, 0, 0];
let servoPins = [0, 1, 2, 3]; // Default unshared header pins for Waveshare ESP32-C6-LCD-1.47
let servoEnabled = [false, false, false, false];
let autoIdleEnabled = false;
let servoNames = ["Servo 1 — Claw", "Servo 2 — Arm", "Servo 3 — Shoulder", "Servo 4 — Scissor Lift"];
let isOffsetMode = false;

let scissorMin = 0;
let scissorMax = 180;
let draftMin = 0;
let draftMax = 180;
let scissorPercent = 50;

let isDraggingSlider = [false, false, false, false];
let lastSliderEditMs = [0, 0, 0, 0];
let isDraggingLift = false;
let pendingServoCmds = {};
let servoThrottleTimer = null;

window.addEventListener("pointerup", () => {
  isDraggingLift = false;
  for (let i = 0; i < 4; i++) isDraggingSlider[i] = false;
});
window.addEventListener("touchend", () => {
  isDraggingLift = false;
  for (let i = 0; i < 4; i++) isDraggingSlider[i] = false;
});

function initServoStorage() {
  try {
    const storedNames = localStorage.getItem("arm_servo_names_v3");
    if (storedNames) {
      const parsed = JSON.parse(storedNames);
      if (Array.isArray(parsed) && parsed.length >= 3) {
        servoNames[0] = parsed[0] || servoNames[0];
        servoNames[1] = parsed[1] || servoNames[1];
        servoNames[2] = parsed[2] || servoNames[2];
        if (parsed[3]) servoNames[3] = parsed[3];
      }
    }
    const storedOffsets = localStorage.getItem("arm_servo_offsets_v3");
    if (storedOffsets) {
      const parsed = JSON.parse(storedOffsets);
      if (Array.isArray(parsed)) {
        for (let i = 0; i < Math.min(4, parsed.length); i++) {
          servoOffsets[i] = parsed[i];
        }
      }
    }
    const storedRange = localStorage.getItem("scissor_lift_range");
    if (storedRange) {
      const parsed = JSON.parse(storedRange);
      if (parsed && typeof parsed.min === "number" && typeof parsed.max === "number") {
        scissorMin = Math.max(0, Math.min(179, Math.round(parsed.min)));
        scissorMax = Math.max(scissorMin + 1, Math.min(180, Math.round(parsed.max)));
        draftMin = scissorMin;
        draftMax = scissorMax;
      }
    }
    const storedPins = localStorage.getItem("arm_servo_pins_c6");
    if (storedPins) {
      const parsed = JSON.parse(storedPins);
      if (Array.isArray(parsed) && parsed.length === 4) {
        servoPins = parsed.map(n => parseInt(n, 10));
      }
    }
  } catch (e) {}

  scissorPercent = angleToPercent(servoAngles[3], scissorMin, scissorMax);
  applyAutoIdleState(autoIdleEnabled);
  updateAllServoUI();
}

function percentToAngle(pct, minA, maxA) {
  const clampedPct = Math.max(0, Math.min(100, pct));
  const span = Math.max(1, maxA - minA);
  return Math.round(minA + (clampedPct / 100) * span);
}

function angleToPercent(angle, minA, maxA) {
  const span = Math.max(1, maxA - minA);
  const pct = Math.round(((angle - minA) / span) * 100);
  return Math.max(0, Math.min(100, pct));
}

function updateServoStateBadges() {
  for (let i = 0; i < 4; i++) {
    const badge = document.getElementById(`s${i + 1}-state-badge`);
    if (!badge) continue;
    if (servoEnabled[i]) {
      badge.innerText = "ACTIVE";
      badge.style.background = "rgba(16,185,129,0.2)";
      badge.style.color = "#34d399";
      badge.style.borderColor = "rgba(16,185,129,0.4)";
    } else {
      badge.innerText = "IDLE";
      badge.style.background = "rgba(148,163,184,0.15)";
      badge.style.color = "#94a3b8";
      badge.style.borderColor = "rgba(148,163,184,0.3)";
    }
  }
}

function updatePinDisplay() {
  const p1Lbl = document.getElementById("s1-pin-label");
  const p2Lbl = document.getElementById("s2-pin-label");
  const p3Lbl = document.getElementById("s3-pin-label");
  const p4Lbl = document.getElementById("s4-pin-label");

  if (p1Lbl) p1Lbl.innerText = `GP${servoPins[0]}`;
  if (p2Lbl) p2Lbl.innerText = `GP${servoPins[1]}`;
  if (p3Lbl) p3Lbl.innerText = `GP${servoPins[2]}`;
  if (p4Lbl) p4Lbl.innerText = `GP${servoPins[3]}`;

  const inp1 = document.getElementById("pin-s1-input");
  const inp2 = document.getElementById("pin-s2-input");
  const inp3 = document.getElementById("pin-s3-input");
  const inp4 = document.getElementById("pin-s4-input");

  if (inp1 && !document.activeElement?.isSameNode(inp1)) inp1.value = servoPins[0];
  if (inp2 && !document.activeElement?.isSameNode(inp2)) inp2.value = servoPins[1];
  if (inp3 && !document.activeElement?.isSameNode(inp3)) inp3.value = servoPins[2];
  if (inp4 && !document.activeElement?.isSameNode(inp4)) inp4.value = servoPins[3];
}

function updateScissorUI() {
  const liftSl = document.getElementById("lift-slider");
  const liftVal = document.getElementById("lift-val");
  const rangeBadge = document.getElementById("saved-range-badge");
  const trimBadge = document.getElementById("trim-sync-badge");

  if (liftSl && !isDraggingLift) liftSl.value = scissorPercent;
  if (liftVal) liftVal.innerText = `${scissorPercent}% (${servoAngles[3]}°)`;
  if (rangeBadge) rangeBadge.innerText = `Range: ${scissorMin}° – ${scissorMax}°`;

  const qMin = document.getElementById("quick-min-lbl");
  const qMid = document.getElementById("quick-mid-lbl");
  const qMax = document.getElementById("quick-max-lbl");
  const midAngle = Math.round((scissorMin + scissorMax) / 2);
  if (qMin) qMin.innerText = `Min (${scissorMin}°)`;
  if (qMid) qMid.innerText = `Mid (${midAngle}°)`;
  if (qMax) qMax.innerText = `Max (${scissorMax}°)`;

  const minValEl = document.getElementById("min-trim-val");
  const maxValEl = document.getElementById("max-trim-val");
  const minSl = document.getElementById("min-trim-slider");
  const maxSl = document.getElementById("max-trim-slider");

  if (minValEl) minValEl.innerText = `${draftMin}°`;
  if (maxValEl) maxValEl.innerText = `${draftMax}°`;
  if (minSl) minSl.value = draftMin;
  if (maxSl) maxSl.value = draftMax;

  if (trimBadge) {
    const dirty = (draftMin !== scissorMin) || (draftMax !== scissorMax);
    if (dirty) {
      trimBadge.innerText = "Unsaved";
      trimBadge.className = "chip chip-warn";
    } else {
      trimBadge.innerText = "Saved";
      trimBadge.className = "chip chip-ok";
    }
  }
}

function updateAllServoUI() {
  for (let i = 0; i < 3; i++) {
    const n = i + 1;
    const offEl = document.getElementById(`o${n}-val`);
    if (offEl) offEl.innerText = `${servoOffsets[i] > 0 ? "+" : ""}${servoOffsets[i]}°`;

    const sl = document.getElementById(`s${n}-slider`);
    const hdr = document.getElementById(`s${n}-header`);

    const btnA = document.getElementById(`s${n}-btn-a`);
    const btnB = document.getElementById(`s${n}-btn-b`);
    const btnC = document.getElementById(`s${n}-btn-c`);
    const btnD = document.getElementById(`s${n}-btn-d`);

    if (isOffsetMode) {
      if (sl) {
        sl.classList.add("offset-slider");
        sl.min = "-45";
        sl.max = "45";
        sl.value = servoOffsets[i];
      }
      if (hdr) {
        hdr.innerHTML = `${escapeHtml(servoNames[i])} Trim: <span id="s${n}-val" style="color:#f59e0b; font-family:monospace; font-weight:700;">${servoOffsets[i] > 0 ? "+" : ""}${servoOffsets[i]}°</span>`;
      }
      if (btnA) btnA.innerText = "-1 Trim";
      if (btnB) btnB.innerText = "+1 Trim";
      if (btnC) btnC.innerText = "-5°";
      if (btnD) btnD.innerText = "+5°";
    } else {
      if (sl) {
        sl.classList.remove("offset-slider");
        sl.min = "0";
        sl.max = "180";
        sl.value = servoAngles[i];
      }
      if (hdr) {
        hdr.innerHTML = `${escapeHtml(servoNames[i])}: <span id="s${n}-val" style="color:#38bdf8; font-family:monospace; font-weight:700;">${servoAngles[i]}°</span>`;
      }
      if (btnA) btnA.innerText = "-10°";
      if (btnB) btnB.innerText = "-1°";
      if (btnC) btnC.innerText = "+1°";
      if (btnD) btnD.innerText = "+10°";
    }
  }

  updateScissorUI();
  updateServoStateBadges();
  updatePinDisplay();
}

function editServoName(idx) {
  const current = servoNames[idx] || `Servo ${idx + 1}`;
  const next = prompt(`Rename Servo ${idx + 1}:`, current);
  if (next && next.trim().length > 0) {
    servoNames[idx] = next.trim().substring(0, 28);
    try { localStorage.setItem("arm_servo_names_v3", JSON.stringify(servoNames)); } catch (e) {}
    updateAllServoUI();
  }
}

function toggleOffsetMode() {
  isOffsetMode = !isOffsetMode;
  const btn = document.getElementById("offset-mode-btn");
  if (btn) {
    if (isOffsetMode) {
      btn.innerText = "Trim: ON";
      btn.style.background = "#f59e0b";
      btn.style.color = "#000000";
      showToast("Calibration Trim Mode Active", "warn");
    } else {
      btn.innerText = "Trim: OFF";
      btn.style.background = "#1e293b";
      btn.style.color = "#f59e0b";
      showToast("Servo Angle Mode Active", "info");
    }
  }
  updateAllServoUI();
}

function toggleAutoIdle() {
  autoIdleEnabled = !autoIdleEnabled;
  applyAutoIdleState(autoIdleEnabled);
  sendCommand(`autoidle ${autoIdleEnabled ? "on" : "off"}`);
  showToast(autoIdleEnabled ? "Auto-Idle ON (relaxes after 3s)" : "Auto-Idle OFF (holds torque)", "info");
}

function applyAutoIdleState(enabled) {
  autoIdleEnabled = !!enabled;
  const btn = document.getElementById("auto-idle-btn");
  if (!btn) return;
  if (autoIdleEnabled) {
    btn.innerText = "Auto-Idle: ON";
    btn.style.background = "#10b981";
    btn.style.color = "#000000";
    btn.style.borderColor = "#10b981";
  } else {
    btn.innerText = "Auto-Idle: OFF";
    btn.style.background = "#1e293b";
    btn.style.color = "rgba(255,255,255,0.75)";
    btn.style.borderColor = "#475569";
  }
}

function onSliderStart(idx) {
  isDraggingSlider[idx] = true;
  lastSliderEditMs[idx] = Date.now();
}

function onServoSlider(idx, val) {
  const parsed = parseInt(val, 10);
  if (isNaN(parsed)) return;
  const n = idx + 1;

  lastSliderEditMs[idx] = Date.now();
  servoEnabled[idx] = true;
  updateServoStateBadges();

  if (isOffsetMode) {
    servoOffsets[idx] = parsed;
    const offEl = document.getElementById(`o${n}-val`);
    if (offEl) offEl.innerText = `${parsed > 0 ? "+" : ""}${parsed}°`;
    const valEl = document.getElementById(`s${n}-val`);
    if (valEl) valEl.innerText = `${parsed > 0 ? "+" : ""}${parsed}°`;
    pendingServoCmds[`o${n}`] = `offset s${n} ${parsed}`;
    try { localStorage.setItem("arm_servo_offsets_v3", JSON.stringify(servoOffsets)); } catch (e) {}
  } else {
    servoAngles[idx] = parsed;
    const valEl = document.getElementById(`s${n}-val`);
    if (valEl) valEl.innerText = `${parsed}°`;
    pendingServoCmds[`s${n}`] = `s${n} ${parsed}`;
  }

  triggerThrottledSend();
}

function onSliderCommit(idx, val) {
  isDraggingSlider[idx] = false;
  lastSliderEditMs[idx] = Date.now();
  const parsed = parseInt(val, 10);
  if (isNaN(parsed)) return;
  const n = idx + 1;

  if (isOffsetMode) {
    delete pendingServoCmds[`o${n}`];
    servoOffsets[idx] = parsed;
    sendCommand(`offset s${n} ${parsed}`);
    try { localStorage.setItem("arm_servo_offsets_v3", JSON.stringify(servoOffsets)); } catch (e) {}
  } else {
    delete pendingServoCmds[`s${n}`];
    servoAngles[idx] = parsed;
    sendCommand(`s${n} ${parsed}`);
  }
}

function onLiftSliderStart() {
  isDraggingLift = true;
  lastSliderEditMs[3] = Date.now();
}

function onLiftSlider(pctVal) {
  const parsedPct = parseInt(pctVal, 10);
  if (isNaN(parsedPct)) return;

  lastSliderEditMs[3] = Date.now();
  servoEnabled[3] = true;
  scissorPercent = Math.max(0, Math.min(100, parsedPct));
  servoAngles[3] = percentToAngle(scissorPercent, scissorMin, scissorMax);

  const valEl = document.getElementById("lift-val");
  if (valEl) valEl.innerText = `${scissorPercent}% (${servoAngles[3]}°)`;
  updateServoStateBadges();

  pendingServoCmds["s4"] = `s4 ${servoAngles[3]}`;
  triggerThrottledSend();
}

function onLiftSliderCommit(pctVal) {
  isDraggingLift = false;
  lastSliderEditMs[3] = Date.now();
  const parsedPct = parseInt(pctVal, 10);
  if (isNaN(parsedPct)) return;

  delete pendingServoCmds["s4"];
  scissorPercent = Math.max(0, Math.min(100, parsedPct));
  servoAngles[3] = percentToAngle(scissorPercent, scissorMin, scissorMax);

  const valEl = document.getElementById("lift-val");
  if (valEl) valEl.innerText = `${scissorPercent}% (${servoAngles[3]}°)`;
  sendCommand(`s4 ${servoAngles[3]}`);
}

function nudgeLiftPercent(deltaPct) {
  lastSliderEditMs[3] = Date.now();
  scissorPercent = Math.max(0, Math.min(100, scissorPercent + deltaPct));
  servoAngles[3] = percentToAngle(scissorPercent, scissorMin, scissorMax);
  servoEnabled[3] = true;
  updateAllServoUI();
  sendCommand(`s4 ${servoAngles[3]}`);
}

function moveToSavedMin() {
  servoAngles[3] = scissorMin;
  scissorPercent = 0;
  servoEnabled[3] = true;
  updateAllServoUI();
  sendCommand("min");
  showToast(`Scissor to Min (${scissorMin}°)`, "info");
}

function moveToSavedMid() {
  const mid = Math.round((scissorMin + scissorMax) / 2);
  servoAngles[3] = mid;
  scissorPercent = 50;
  servoEnabled[3] = true;
  updateAllServoUI();
  sendCommand("mid");
  showToast(`Scissor to Mid (${mid}°)`, "info");
}

function moveToSavedMax() {
  servoAngles[3] = scissorMax;
  scissorPercent = 100;
  servoEnabled[3] = true;
  updateAllServoUI();
  sendCommand("max");
  showToast(`Scissor to Max (${scissorMax}°)`, "info");
}

function onMinTrimSlider(val) {
  const parsed = parseInt(val, 10);
  if (isNaN(parsed)) return;
  setMinTrimAngle(parsed, true);
}

function nudgeMinTrim(delta) {
  setMinTrimAngle(draftMin + delta, false);
}

function setMinTrimAngle(target, throttled) {
  lastSliderEditMs[3] = Date.now();
  let next = Math.max(0, Math.min(179, target));
  if (next >= draftMax) {
    next = draftMax - 1;
    showToast("Bottom end must be below top end", "warn");
  }
  draftMin = next;
  servoAngles[3] = next;
  scissorPercent = 0;
  servoEnabled[3] = true;
  updateAllServoUI();

  if (throttled) {
    pendingServoCmds["s4"] = `s4 ${next}`;
    triggerThrottledSend();
  } else {
    delete pendingServoCmds["s4"];
    sendCommand(`s4 ${next}`);
  }
}

function onMaxTrimSlider(val) {
  const parsed = parseInt(val, 10);
  if (isNaN(parsed)) return;
  setMaxTrimAngle(parsed, true);
}

function nudgeMaxTrim(delta) {
  setMaxTrimAngle(draftMax + delta, false);
}

function setMaxTrimAngle(target, throttled) {
  lastSliderEditMs[3] = Date.now();
  let next = Math.max(1, Math.min(180, target));
  if (next <= draftMin) {
    next = draftMin + 1;
    showToast("Top end must be above bottom end", "warn");
  }
  draftMax = next;
  servoAngles[3] = next;
  scissorPercent = 100;
  servoEnabled[3] = true;
  updateAllServoUI();

  if (throttled) {
    pendingServoCmds["s4"] = `s4 ${next}`;
    triggerThrottledSend();
  } else {
    delete pendingServoCmds["s4"];
    sendCommand(`s4 ${next}`);
  }
}

function testTrimEnd(which) {
  lastSliderEditMs[3] = Date.now();
  const target = (which === "min") ? draftMin : draftMax;
  servoAngles[3] = target;
  scissorPercent = (which === "min") ? 0 : 100;
  servoEnabled[3] = true;
  updateAllServoUI();
  sendCommand(`move ${servoAngles[0]} ${servoAngles[1]} ${servoAngles[2]} ${target} 350`);
}

async function saveScissorRange() {
  const btn = document.getElementById("save-scissor-range-btn");
  const txt = document.getElementById("save-scissor-range-text");
  if (txt) txt.innerText = "Saving...";
  if (btn) btn.style.background = "#0284c7";

  scissorMin = draftMin;
  scissorMax = draftMax;
  scissorPercent = angleToPercent(servoAngles[3], scissorMin, scissorMax);

  try {
    localStorage.setItem("scissor_lift_range", JSON.stringify({ min: scissorMin, max: scissorMax }));
  } catch (e) {}

  await sendCommand(`range_set ${scissorMin} ${scissorMax}`);
  updateAllServoUI();

  appendLog("Calibration", `Saved Scissor Lift Range: ${scissorMin}° – ${scissorMax}°`);
  showToast(`Saved Scissor Trim (${scissorMin}°–${scissorMax}°) to Flash!`, "success");

  if (txt) txt.innerText = "Saved to Flash";
  if (btn) btn.style.background = "#059669";
  setTimeout(() => {
    if (txt) txt.innerText = "Save Trim to Flash";
    if (btn) btn.style.background = "#10b981";
  }, 1500);
}

async function resetScissorRange() {
  draftMin = 0;
  draftMax = 180;
  scissorMin = 0;
  scissorMax = 180;
  scissorPercent = angleToPercent(servoAngles[3], scissorMin, scissorMax);

  try {
    localStorage.setItem("scissor_lift_range", JSON.stringify({ min: 0, max: 180 }));
  } catch (e) {}

  await sendCommand("range_set 0 180");
  updateAllServoUI();
  appendLog("Calibration", "Reset Scissor Lift Range to 0° – 180°");
  showToast("Scissor range reset to 0°–180°", "info");
}

function triggerThrottledSend() {
  if (!servoThrottleTimer) {
    servoThrottleTimer = setTimeout(() => {
      servoThrottleTimer = null;
      const cmds = pendingServoCmds;
      pendingServoCmds = {};
      for (const k in cmds) {
        sendCommand(cmds[k]);
      }
    }, 50);
  }
}

function servoStepAction(idx, normalDelta, trimDelta) {
  if (isOffsetMode) {
    if (trimDelta === -1 || trimDelta === 1) {
      adjustOffset(idx, trimDelta);
    } else {
      nudgeServo(idx, trimDelta);
    }
  } else {
    nudgeServo(idx, normalDelta);
  }
}

function nudgeServo(idx, delta) {
  lastSliderEditMs[idx] = Date.now();
  let next = (servoAngles[idx] || 90) + delta;
  if (next < 0) next = 0;
  if (next > 180) next = 180;
  servoAngles[idx] = next;
  servoEnabled[idx] = true;
  updateServoStateBadges();

  const n = idx + 1;
  if (!isOffsetMode) {
    const sl = document.getElementById(`s${n}-slider`);
    if (sl) sl.value = next;
    const valEl = document.getElementById(`s${n}-val`);
    if (valEl) valEl.innerText = `${next}°`;
  }
  sendCommand(`s${n} ${next}`);
}

function adjustOffset(idx, delta) {
  lastSliderEditMs[idx] = Date.now();
  let next = (servoOffsets[idx] || 0) + delta;
  if (next < -45) next = -45;
  if (next > 45) next = 45;
  servoOffsets[idx] = next;
  servoEnabled[idx] = true;
  updateServoStateBadges();

  const n = idx + 1;
  const offEl = document.getElementById(`o${n}-val`);
  if (offEl) offEl.innerText = `${next > 0 ? "+" : ""}${next}°`;
  if (isOffsetMode) {
    const sl = document.getElementById(`s${n}-slider`);
    if (sl) sl.value = next;
    const valEl = document.getElementById(`s${n}-val`);
    if (valEl) valEl.innerText = `${next > 0 ? "+" : ""}${next}°`;
  }
  sendCommand(`offset s${n} ${next}`);
  appendLog("Trim", `S${n} Offset -> ${next}°`);
  try { localStorage.setItem("arm_servo_offsets_v3", JSON.stringify(servoOffsets)); } catch (e) {}
}

async function saveCalibrationOffsets() {
  const btn = document.getElementById("save-cal-btn");
  const txt = document.getElementById("save-cal-text");
  if (txt) txt.innerText = "Saving...";
  if (btn) btn.style.background = "#0284c7";

  try {
    localStorage.setItem("arm_servo_offsets_v3", JSON.stringify(servoOffsets));
  } catch (e) {}

  await sendCommand(`cal_set ${servoOffsets[0]} ${servoOffsets[1]} ${servoOffsets[2]} ${servoOffsets[3]}`);
  if (isOffsetMode) toggleOffsetMode();
  else updateAllServoUI();

  appendLog("Calibration", `Saved Arm Trim: S1=${servoOffsets[0]}, S2=${servoOffsets[1]}, S3=${servoOffsets[2]}, S4=${servoOffsets[3]}`);
  showToast("Trim offsets saved to ESP32 Flash!", "success");

  if (txt) txt.innerText = "Saved to Flash";
  if (btn) btn.style.background = "#059669";
  setTimeout(() => {
    if (txt) txt.innerText = "Save Trim to Flash";
    if (btn) btn.style.background = "#10b981";
  }, 1500);
}

function homeAllServos() {
  servoAngles = [90, 90, 90, 90];
  servoEnabled = [true, true, true, true];
  scissorPercent = angleToPercent(90, scissorMin, scissorMax);
  if (isOffsetMode) toggleOffsetMode();
  else updateAllServoUI();
  sendCommand("home");
  showToast("Homing all 4 servos to 90°", "info");
}

function detachAllServos() {
  servoEnabled = [false, false, false, false];
  updateServoStateBadges();
  sendCommand("detach");
  showToast("All servos detached (Idle)", "warn");
}

function requestCalibrationSync() {
  sendCommand("cal_get");
  showToast("Fetching trim offsets & scissor range...", "info");
}

function requestStatusSync() {
  sendCommand("status");
  sendCommand("mpu_get");
  showToast("Syncing arm & scissor state...", "info");
}

function requestPinSync() {
  sendCommand("pins");
  showToast("Fetching hardware pin configuration...", "info");
}

function setServoPin(servoIndex, gpioPin, saveToFlash = true) {
  let idx = -1;
  if (typeof servoIndex === "number") idx = servoIndex;
  else if (typeof servoIndex === "string") {
    const s = servoIndex.toLowerCase().trim();
    if (s === "s1" || s === "claw") idx = 0;
    else if (s === "s2" || s === "arm") idx = 1;
    else if (s === "s3" || s === "shoulder") idx = 2;
    else if (s === "s4" || s === "scissor" || s === "lift") idx = 3;
    else idx = parseInt(s, 10) - 1;
  }

  const pin = parseInt(gpioPin, 10);
  if (isNaN(idx) || idx < 0 || idx >= 4) {
    showToast("Invalid servo index. Use 0..3 or 's1'..'s4'", "error");
    return false;
  }
  if (isNaN(pin) || pin < 0 || pin > 30) {
    showToast(`Invalid GPIO pin ${gpioPin}. Use safe pin 0..23`, "error");
    return false;
  }

  servoPins[idx] = pin;
  updatePinDisplay();

  try {
    localStorage.setItem("arm_servo_pins_c6", JSON.stringify(servoPins));
  } catch (e) {}

  sendCommand(`pin s${idx + 1} ${pin}`);
  appendLog("Config", `Mapped S${idx + 1} -> GPIO ${pin}`);
  showToast(`S${idx + 1} mapped to GPIO ${pin}`, "success");
  return true;
}

function setAllPins(p1, p2, p3, p4) {
  const pins = [parseInt(p1, 10), parseInt(p2, 10), parseInt(p3, 10), parseInt(p4, 10)];
  for (let i = 0; i < 4; i++) {
    if (isNaN(pins[i]) || pins[i] < 0 || pins[i] > 30) {
      showToast(`Invalid pin value: ${pins[i]}`, "error");
      return false;
    }
  }

  servoPins = pins;
  updatePinDisplay();

  try {
    localStorage.setItem("arm_servo_pins_c6", JSON.stringify(servoPins));
  } catch (e) {}

  sendCommand(`pins ${pins[0]} ${pins[1]} ${pins[2]} ${pins[3]}`);
  appendLog("Config", `Saved Hardware Pins: S1=${pins[0]}, S2=${pins[1]}, S3=${pins[2]}, S4=${pins[3]}`);
  showToast(`Pins applied: S1=${pins[0]}, S2=${pins[1]}, S3=${pins[2]}, S4=${pins[3]}`, "success");
  return true;
}

function saveHardwarePins() {
  const p1 = parseInt(document.getElementById("pin-s1-input")?.value, 10);
  const p2 = parseInt(document.getElementById("pin-s2-input")?.value, 10);
  const p3 = parseInt(document.getElementById("pin-s3-input")?.value, 10);
  const p4 = parseInt(document.getElementById("pin-s4-input")?.value, 10);
  return setAllPins(p1, p2, p3, p4);
}

function applyReceivedPins(p1, p2, p3, p4) {
  if (isNaN(p1) || isNaN(p2) || isNaN(p3) || isNaN(p4)) return;
  servoPins = [p1, p2, p3, p4];
  try {
    localStorage.setItem("arm_servo_pins_c6", JSON.stringify(servoPins));
  } catch (e) {}
  updatePinDisplay();
}

function applyReceivedAngles(s1, s2, s3, s4, forceSync = false) {
  const incoming = [s1, s2, s3, (s4 !== undefined ? s4 : servoAngles[3])];
  const now = Date.now();

  for (let i = 0; i < 4; i++) {
    if (isNaN(incoming[i])) continue;
    if (!forceSync && (isDraggingSlider[i] || (i === 3 && isDraggingLift) || (now - lastSliderEditMs[i]) < 600)) {
      continue;
    }
    servoAngles[i] = incoming[i];
    const n = i + 1;
    if (i < 3 && !isOffsetMode) {
      const sl = document.getElementById(`s${n}-slider`);
      if (sl) sl.value = incoming[i];
      const lbl = document.getElementById(`s${n}-val`);
      if (lbl) lbl.innerText = `${incoming[i]}°`;
    }
  }

  if (!isDraggingLift && (now - lastSliderEditMs[3]) >= 600) {
    scissorPercent = angleToPercent(servoAngles[3], scissorMin, scissorMax);
  }
  updateScissorUI();
}

function applyReceivedOffsets(o1, o2, o3, o4) {
  const incoming = [o1, o2, o3, (o4 !== undefined ? o4 : servoOffsets[3])];
  for (let i = 0; i < 4; i++) {
    if (isNaN(incoming[i])) continue;
    servoOffsets[i] = incoming[i];
    const n = i + 1;
    const offEl = document.getElementById(`o${n}-val`);
    if (offEl) offEl.innerText = `${incoming[i] > 0 ? "+" : ""}${incoming[i]}°`;
  }
  if (isOffsetMode) updateAllServoUI();
}

function applyReceivedRange(minA, maxA) {
  if (isNaN(minA) || isNaN(maxA)) return;
  scissorMin = Math.max(0, Math.min(179, minA));
  scissorMax = Math.max(scissorMin + 1, Math.min(180, maxA));
  draftMin = scissorMin;
  draftMax = scissorMax;
  scissorPercent = angleToPercent(servoAngles[3], scissorMin, scissorMax);
  try {
    localStorage.setItem("scissor_lift_range", JSON.stringify({ min: scissorMin, max: scissorMax }));
  } catch (e) {}
  updateScissorUI();
}

function applyReceivedEnableStates(e1, e2, e3, e4) {
  servoEnabled = [!!e1, !!e2, !!e3, (e4 !== undefined ? !!e4 : servoEnabled[3])];
  updateServoStateBadges();
}

// Global exports for inline HTML handlers & window access
window.initServoStorage = initServoStorage;
window.setServoPin = setServoPin;
window.setAllPins = setAllPins;
window.saveHardwarePins = saveHardwarePins;
window.requestPinSync = requestPinSync;
window.applyReceivedPins = applyReceivedPins;
window.applyReceivedAngles = applyReceivedAngles;
window.applyReceivedOffsets = applyReceivedOffsets;
window.applyReceivedRange = applyReceivedRange;
window.applyReceivedEnableStates = applyReceivedEnableStates;
window.applyAutoIdleState = applyAutoIdleState;
window.homeAllServos = homeAllServos;
window.detachAllServos = detachAllServos;
window.toggleOffsetMode = toggleOffsetMode;
window.toggleAutoIdle = toggleAutoIdle;
window.onSliderStart = onSliderStart;
window.onServoSlider = onServoSlider;
window.onSliderCommit = onSliderCommit;
window.onLiftSliderStart = onLiftSliderStart;
window.onLiftSlider = onLiftSlider;
window.onLiftSliderCommit = onLiftSliderCommit;
window.nudgeLiftPercent = nudgeLiftPercent;
window.moveToSavedMin = moveToSavedMin;
window.moveToSavedMid = moveToSavedMid;
window.moveToSavedMax = moveToSavedMax;
window.onMinTrimSlider = onMinTrimSlider;
window.nudgeMinTrim = nudgeMinTrim;
window.setMinTrimAngle = setMinTrimAngle;
window.onMaxTrimSlider = onMaxTrimSlider;
window.nudgeMaxTrim = nudgeMaxTrim;
window.setMaxTrimAngle = setMaxTrimAngle;
window.testTrimEnd = testTrimEnd;
window.saveScissorRange = saveScissorRange;
window.resetScissorRange = resetScissorRange;
window.servoStepAction = servoStepAction;
window.saveCalibrationOffsets = saveCalibrationOffsets;
window.requestCalibrationSync = requestCalibrationSync;
window.requestStatusSync = requestStatusSync;
window.editServoName = editServoName;

export {
  servoAngles,
  servoOffsets,
  servoPins,
  servoEnabled,
  initServoStorage,
  setServoPin,
  setAllPins,
  saveHardwarePins,
  requestPinSync,
  applyReceivedPins,
  applyReceivedAngles,
  applyReceivedOffsets,
  applyReceivedRange,
  applyReceivedEnableStates,
  applyAutoIdleState,
  homeAllServos,
  detachAllServos,
  toggleOffsetMode,
  toggleAutoIdle,
  saveScissorRange,
  resetScissorRange
};