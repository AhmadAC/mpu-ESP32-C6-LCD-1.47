

const UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
const UART_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
const UART_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

let isConnected = false;
let bleBackend = null;
let nativeDeviceId = null;
let webDevice = null;
let webServer = null;
let webTxChar = null;
let webNotifyChar = null;

let rxCount = 0;
let txCount = 0;
let rawLogLines = [];
let cmdQueue = [];
let isProcessingQueue = false;
let consecutiveWriteErrors = 0;
let toastTimeout = null;
let rxLineBuffer = "";
let brownoutNotified = false;
let activeBleListeners = [];
let activeNotifChannel = null;

function updateCounterDisplay() {
  const el = document.getElementById("ble-counters");
  if (el) el.innerText = `RX: ${rxCount} | TX: ${txCount}`;
}

function showToast(msg, type = "info") {
  const toast = document.getElementById("toast-notification");
  const text = document.getElementById("toast-message");
  const icon = document.getElementById("toast-icon");
  if (!toast || !text) return;

  text.innerText = msg;
  if (type === "success") {
    toast.style.borderColor = "#10b981";
    if (icon) icon.style.color = "#10b981";
  } else if (type === "error") {
    toast.style.borderColor = "#ef4444";
    if (icon) icon.style.color = "#ef4444";
  } else if (type === "warn") {
    toast.style.borderColor = "#f59e0b";
    if (icon) icon.style.color = "#f59e0b";
  } else {
    toast.style.borderColor = "#38bdf8";
    if (icon) icon.style.color = "#38bdf8";
  }

  toast.classList.add("show");
  if (toastTimeout) clearTimeout(toastTimeout);
  toastTimeout = setTimeout(() => { toast.classList.remove("show"); }, 2600);
}

function withTimeout(promise, ms, label) {
  return Promise.race([
    promise,
    new Promise((_, reject) => setTimeout(() => reject(new Error(`${label} timeout (${ms}ms)`)), ms))
  ]);
}

function getBleBackend() {
  if (window.BluetoothLe) return { type: "capacitor", ble: window.BluetoothLe };
  if (window.parent?.BluetoothLe) return { type: "capacitor", ble: window.parent.BluetoothLe };
  if (window.Capacitor?.Plugins?.BluetoothLe) return { type: "capacitor", ble: window.Capacitor.Plugins.BluetoothLe };
  if (window.parent?.Capacitor?.Plugins?.BluetoothLe) return { type: "capacitor", ble: window.parent.Capacitor.Plugins.BluetoothLe };
  if (navigator.bluetooth) return { type: "web-bluetooth" };
  return null;
}

function initBackendUI() {
  const backend = getBleBackend();
  const badge = document.getElementById("backend-badge");
  if (!backend) {
    if (badge) badge.innerText = "No BLE";
    return;
  }
  if (backend.type === "capacitor") {
    bleBackend = "capacitor";
    if (badge) {
      badge.innerText = "iOS CoreBLE";
      badge.style.color = "#6ee7b7";
    }
  } else {
    bleBackend = "web-bluetooth";
    if (badge) {
      badge.innerText = "Web BLE";
      badge.style.color = "#7dd3fc";
    }
  }
}

async function clearBleListeners() {
  const handles = activeBleListeners;
  activeBleListeners = [];
  for (const h of handles) {
    try {
      if (h && typeof h.remove === "function") await h.remove();
    } catch (e) {}
  }
}

async function toggleConnection() {
  if (isConnected) await disconnectDevice();
  else await connectDevice();
}

async function connectDevice() {
  const backend = getBleBackend();
  if (!backend) {
    showToast("Bluetooth is unavailable in this environment.", "error");
    appendLog("Error", "No BluetoothLe plugin found in window or parent.");
    return;
  }

  setUiState("Connecting...", "#f59e0b", "Connecting");
  appendLog("BLE", "Opening BLE device picker...");
  await clearBleListeners();
  activeNotifChannel = null;

  try {
    if (backend.type === "capacitor") {
      const Ble = backend.ble;
      await Ble.initialize();

      const device = await Ble.requestDevice({ services: [UART_SERVICE_UUID] });
      if (!device || !device.deviceId) throw new Error("No device selected.");

      nativeDeviceId = device.deviceId;
      const devName = device.name || "ESP32C6-Node";
      appendLog("BLE", `Connecting to ${devName}...`);

      try { if (Ble.stopLEScan) await Ble.stopLEScan(); } catch (e) {}

      await Ble.connect({ deviceId: nativeDeviceId }, () => {
        if (isConnected) {
          showToast("Device disconnected", "error");
          onDisconnectedCleanup();
        }
      });
      await new Promise(r => setTimeout(r, 600));

      const handleCapacitorNotif = (channelTag, result) => {
        if (!result) return;
        if (activeNotifChannel === null) activeNotifChannel = channelTag;
        else if (activeNotifChannel !== channelTag) return;
        const decoded = universalBleDecode(result.value !== undefined ? result.value : result);
        if (decoded) handleIncomingBleData(decoded, "NOTIF");
      };

      if (Ble.addListener) {
        const scopedKeyLower = `notification|${nativeDeviceId}|${UART_SERVICE_UUID}|${UART_TX_CHAR_UUID}`;
        const scopedKeyUpper = `notification|${nativeDeviceId}|${UART_SERVICE_UUID.toUpperCase()}|${UART_TX_CHAR_UUID.toUpperCase()}`;

        try {
          const hDisc = await Ble.addListener(`disconnected|${nativeDeviceId}`, () => {
            if (isConnected) {
              showToast("Device disconnected", "error");
              onDisconnectedCleanup();
            }
          });
          if (hDisc) activeBleListeners.push(hDisc);
        } catch (e) {}

        try {
          const hLower = await Ble.addListener(scopedKeyLower, (result) => {
            handleCapacitorNotif("scopedLower", result);
          });
          if (hLower) activeBleListeners.push(hLower);
        } catch (e) {}

        try {
          const hUpper = await Ble.addListener(scopedKeyUpper, (result) => {
            handleCapacitorNotif("scopedUpper", result);
          });
          if (hUpper) activeBleListeners.push(hUpper);
        } catch (e) {}

        try {
          const hGeneric = await Ble.addListener("notification", (result) => {
            handleCapacitorNotif("generic", result);
          });
          if (hGeneric) activeBleListeners.push(hGeneric);
        } catch (e) {}
      }

      try {
        await Ble.startNotifications({
          deviceId: nativeDeviceId,
          service: UART_SERVICE_UUID,
          characteristic: UART_TX_CHAR_UUID
        }, (result) => {
          handleCapacitorNotif("startNotifCb", result);
        });
        appendLog("BLE", "Notifications registered on TX Characteristic.");
      } catch (notifErr) {
        appendLog("BLE Warn", `Notify registration: ${notifErr.message || notifErr}`);
      }

      onConnectedSuccess(devName);

    } else if (backend.type === "web-bluetooth") {
      webDevice = await navigator.bluetooth.requestDevice({
        filters: [
          { services: [UART_SERVICE_UUID] },
          { namePrefix: "ESP" },
          { namePrefix: "Arm" },
          { namePrefix: "MPU" },
          { namePrefix: "C6" }
        ],
        optionalServices: [UART_SERVICE_UUID]
      });

      webDevice.addEventListener("gattserverdisconnected", () => {
        if (isConnected) showToast("Device disconnected", "error");
        onDisconnectedCleanup();
      });
      webServer = await webDevice.gatt.connect();

      const service = await webServer.getPrimaryService(UART_SERVICE_UUID);
      webTxChar = await service.getCharacteristic(UART_RX_CHAR_UUID);

      try {
        webNotifyChar = await service.getCharacteristic(UART_TX_CHAR_UUID);
        await webNotifyChar.startNotifications();
        webNotifyChar.addEventListener("characteristicvaluechanged", (e) => {
          const decoded = universalBleDecode(e.target.value);
          if (decoded) handleIncomingBleData(decoded, "WEB-BLE");
        });
      } catch (notifErr) {}

      onConnectedSuccess(webDevice.name || "ESP32C6-Node");
    }
  } catch (err) {
    appendLog("Error", `Connection failed: ${err.message || err}`);
    showToast("Bluetooth failed to connect", "error");
    await onDisconnectedCleanup();
  }
}

async function disconnectDevice() {
  const backend = getBleBackend();
  try {
    if (backend && backend.type === "capacitor" && nativeDeviceId) {
      await backend.ble.disconnect({ deviceId: nativeDeviceId });
    } else if (webDevice?.gatt?.connected) {
      await webDevice.gatt.disconnect();
    }
  } catch (e) {}
  await onDisconnectedCleanup();
}

async function tryFallbackBleRead() {
  const backend = getBleBackend();
  try {
    if (backend && backend.type === "capacitor" && nativeDeviceId) {
      const res = await backend.ble.read({
        deviceId: nativeDeviceId,
        service: UART_SERVICE_UUID,
        characteristic: UART_TX_CHAR_UUID
      });
      const decoded = universalBleDecode(res && res.value !== undefined ? res.value : res);
      if (decoded) handleIncomingBleData(decoded, "READ");
    } else if (webNotifyChar) {
      const val = await webNotifyChar.readValue();
      const decoded = universalBleDecode(val);
      if (decoded) handleIncomingBleData(decoded, "READ");
    }
  } catch (e) {}
}

function onConnectedSuccess(name) {
  isConnected = true;
  consecutiveWriteErrors = 0;
  cmdQueue = [];
  isProcessingQueue = false;
  rxLineBuffer = "";
  brownoutNotified = false;
  document.getElementById("device-name").innerText = name;
  setUiState("Connected", "#10b981", "Disconnect");
  appendLog("BLE", `Connected to ${name}.`);
  showToast(`Connected to ${name}`, "success");

  setTimeout(() => {
    sendCommand("status");
    setTimeout(() => sendCommand("espnow_get"), 250);
    setTimeout(() => sendCommand("action_get"), 500);
    setTimeout(() => sendCommand("pins"), 750);
    setTimeout(() => sendCommand("mpu_get"), 1000);
    setTimeout(() => sendCommand("mpu on"), 1250);
    setTimeout(async () => {
      if (isConnected && rxCount === 0) {
        await tryFallbackBleRead();
      }
    }, 1700);
  }, 450);
}

async function onDisconnectedCleanup() {
  const wasConnected = isConnected;
  isConnected = false;
  nativeDeviceId = null;
  webDevice = null;
  webServer = null;
  webTxChar = null;
  webNotifyChar = null;
  cmdQueue = [];
  isProcessingQueue = false;
  consecutiveWriteErrors = 0;
  rxLineBuffer = "";
  activeNotifChannel = null;
  await clearBleListeners();
  document.getElementById("device-name").innerText = "None Connected";
  setUiState("Disconnected", "#f43f5e", "Connect BLE");
  if (wasConnected) {
    appendLog("BLE", "Device disconnected.");
  }
  if (typeof window.applyReceivedEnableStates === "function") {
    window.applyReceivedEnableStates(false, false, false, false);
  }
  if (typeof window.updateActionRunBadge === "function") {
    window.updateActionRunBadge(false, 0, false);
  }
  if (typeof window.onImuLinkLost === "function") {
    window.onImuLinkLost();
  }
  if (typeof window.onEspNowLinkLost === "function") {
    window.onEspNowLinkLost();
  }
}

function getCoalesceKey(cmd) {
  if (cmd.startsWith("s1 ")) return "s1";
  if (cmd.startsWith("s2 ")) return "s2";
  if (cmd.startsWith("s3 ")) return "s3";
  if (cmd.startsWith("s4 ")) return "s4";
  if (cmd.startsWith("offset s1 ")) return "o1";
  if (cmd.startsWith("offset s2 ")) return "o2";
  if (cmd.startsWith("offset s3 ")) return "o3";
  if (cmd.startsWith("offset s4 ")) return "o4";
  if (cmd === "status") return "status";
  if (cmd.startsWith("mpu_gain ")) return "mpugain";
  return null;
}

async function sendCommand(commandText) {
  if (!isConnected) {
    appendLog("Offline", commandText);
    return;
  }

  const key = getCoalesceKey(commandText);
  if (key !== null) {
    for (let i = cmdQueue.length - 1; i >= 0; i--) {
      if (getCoalesceKey(cmdQueue[i]) === key) {
        cmdQueue.splice(i, 1);
      }
    }
  }

  cmdQueue.push(commandText);
  if (isProcessingQueue) return;
  isProcessingQueue = true;

  try {
    while (cmdQueue.length > 0 && isConnected) {
      const nextCmd = cmdQueue.shift();
      await executeSendCommand(nextCmd);
    }
  } finally {
    isProcessingQueue = false;
  }
}

function textToHexBytes(str) {
  let hex = "";
  for (let i = 0; i < str.length; i++) {
    let h = str.charCodeAt(i).toString(16);
    hex += (h.length === 1 ? "0" + h : h) + " ";
  }
  return hex.trim();
}

async function writeChunk(backend, chunkStr) {
  if (backend && backend.type === "capacitor" && nativeDeviceId) {
    const Ble = backend.ble;
    const hexPayload = textToHexBytes(chunkStr);
    try {
      await withTimeout(Ble.writeWithoutResponse({
        deviceId: nativeDeviceId,
        service: UART_SERVICE_UUID,
        characteristic: UART_RX_CHAR_UUID,
        value: hexPayload
      }), 1500, "BLE writeNoRsp");
    } catch (writeErr) {
      if (!isConnected) throw writeErr;
      await withTimeout(Ble.write({
        deviceId: nativeDeviceId,
        service: UART_SERVICE_UUID,
        characteristic: UART_RX_CHAR_UUID,
        value: hexPayload
      }), 2000, "BLE write");
    }
  } else if (webTxChar) {
    const encoder = new TextEncoder();
    const bytes = encoder.encode(chunkStr);
    if (webTxChar.writeValueWithoutResponse) {
      await withTimeout(webTxChar.writeValueWithoutResponse(bytes), 1500, "WebBLE writeNoRsp");
    } else {
      await withTimeout(webTxChar.writeValue(bytes), 2000, "WebBLE write");
    }
  }
}

async function executeSendCommand(commandText) {
  if (!isConnected) return;

  const backend = getBleBackend();
  const payload = commandText.endsWith("\n") ? commandText : (commandText + "\n");

  try {
    const CHUNK_SIZE = 90;
    for (let i = 0; i < payload.length; i += CHUNK_SIZE) {
      const piece = payload.substring(i, i + CHUNK_SIZE);
      await writeChunk(backend, piece);
      if (i + CHUNK_SIZE < payload.length) {
        await new Promise(r => setTimeout(r, 15));
      }
    }

    consecutiveWriteErrors = 0;
    txCount++;
    updateCounterDisplay();
    if (!commandText.startsWith("s1 ") && !commandText.startsWith("s2 ") && !commandText.startsWith("s3 ") && !commandText.startsWith("s4 ") && commandText !== "status") {
      appendLog("TX", commandText);
    }
  } catch (err) {
    consecutiveWriteErrors++;
    const errMsg = err?.message || String(err);
    appendLog("Send Error", errMsg);

    const lower = errMsg.toLowerCase();
    if (lower.includes("not connected") || lower.includes("disconnected") || consecutiveWriteErrors >= 2) {
      showToast("BLE connection lost - reconnect", "error");
      await onDisconnectedCleanup();
    }
  }
}

function universalBleDecode(val) {
  if (val === null || val === undefined) return "";

  if (typeof val === "object" && val !== null && val.value !== undefined) {
    val = val.value;
  }

  if (typeof val === "object" && val !== null) {
    try {
      const buf = val.buffer ? val.buffer : val;
      if (buf instanceof ArrayBuffer || ArrayBuffer.isView(val)) {
        return new TextDecoder().decode(val);
      }
      if (Array.isArray(val.data)) {
        return new TextDecoder().decode(new Uint8Array(val.data));
      }
      if (Array.isArray(val)) {
        return new TextDecoder().decode(new Uint8Array(val));
      }
      const keys = Object.keys(val);
      if (keys.length > 0 && keys.every(k => /^\d+$/.test(k))) {
        const bytes = new Uint8Array(keys.length);
        for (let i = 0; i < keys.length; i++) {
          bytes[i] = val[i] & 0xff;
        }
        return new TextDecoder().decode(bytes);
      }
    } catch (e) {}
  }

  if (typeof val === "string") {
    let raw = val.trim();
    if (!raw) return "";

    const cleanHex = raw.replace(/[\s:]+/g, "");
    if (/^[0-9a-fA-F]+$/.test(cleanHex) && cleanHex.length >= 2 && cleanHex.length % 2 === 0) {
      let hexDecoded = "";
      for (let i = 0; i < cleanHex.length; i += 2) {
        const charCode = parseInt(cleanHex.substring(i, i + 2), 16);
        if (charCode !== 0) hexDecoded += String.fromCharCode(charCode);
      }
      if (hexDecoded && (/^[\x20-\x7E\r\n\t]+$/.test(hexDecoded) ||
          hexDecoded.includes("STATUS:") || hexDecoded.includes("ANG:") ||
          hexDecoded.includes("CAL:") || hexDecoded.includes("ACT:") ||
          hexDecoded.includes("RANGE:") || hexDecoded.includes("PINS:") ||
          hexDecoded.includes("ACT_STATE:") || hexDecoded.includes("SERVO_EN:") ||
          hexDecoded.includes("AUTO_IDLE:") || hexDecoded.includes("IMU:") ||
          hexDecoded.includes("MPUCAL:") || hexDecoded.includes("ESPNOW:") ||
          hexDecoded.includes(">>"))) {
        return hexDecoded;
      }
    }

    try {
      const b64Decoded = atob(raw);
      if (b64Decoded && (/^[\x20-\x7E\r\n\t]+$/.test(b64Decoded) ||
          b64Decoded.includes("STATUS:") || b64Decoded.includes("ANG:") ||
          b64Decoded.includes("CAL:") || b64Decoded.includes("ACT:") ||
          b64Decoded.includes("RANGE:") || b64Decoded.includes("PINS:") ||
          b64Decoded.includes("ACT_STATE:") || b64Decoded.includes("SERVO_EN:") ||
          b64Decoded.includes("AUTO_IDLE:") || b64Decoded.includes("IMU:") ||
          b64Decoded.includes("MPUCAL:") || b64Decoded.includes("ESPNOW:"))) {
        return b64Decoded;
      }
    } catch (e) {}

    return raw;
  }

  return String(val);
}

function handleIncomingBleData(rawText, sourceTag = "BLE") {
  if (!rawText) return;

  rxCount++;
  updateCounterDisplay();

  rxLineBuffer += rawText;
  const parts = rxLineBuffer.split("\n");
  rxLineBuffer = parts.pop();

  parts.forEach(line => {
    const text = line.trim();
    if (!text) return;

    if (!text.startsWith("STATUS:") && !text.startsWith("SERVO_EN:") &&
        !text.startsWith("IMU:") && !text.startsWith("MPUCALPROG:") && !text.startsWith("ESPNOW:")) {
      appendLog(sourceTag, text);
    }

    if (text.startsWith("ESPNOW:")) {
      if (typeof window.handleEspNowLine === "function") {
        window.handleEspNowLine(text);
      }
      return;
    }

    if (typeof window.handleImuLine === "function" && window.handleImuLine(text)) {
      return;
    }

    if (text.startsWith("ANG:")) {
      const vals = text.substring(4).split(",").map(n => parseInt(n.trim(), 10));
      if (vals.length >= 3 && typeof window.applyReceivedAngles === "function") {
        window.applyReceivedAngles(vals[0], vals[1], vals[2], vals[3]);
      }
      return;
    }

    if (text.startsWith("SERVO_EN:")) {
      const vals = text.substring(9).split(",").map(n => n.trim() === "1");
      if (vals.length >= 3 && typeof window.applyReceivedEnableStates === "function") {
        window.applyReceivedEnableStates(vals[0], vals[1], vals[2], vals[3]);
      }
      return;
    }

    if (text.startsWith("PINS:")) {
      const vals = text.substring(5).split(",").map(n => parseInt(n.trim(), 10));
      if (vals.length >= 4 && typeof window.applyReceivedPins === "function") {
        window.applyReceivedPins(vals[0], vals[1], vals[2], vals[3]);
      }
      return;
    }

    if (text.startsWith("RANGE:")) {
      const vals = text.substring(6).split(",").map(n => parseInt(n.trim(), 10));
      if (vals.length >= 2 && typeof window.applyReceivedRange === "function") {
        window.applyReceivedRange(vals[0], vals[1]);
      }
      return;
    }

    if (text.startsWith("AUTO_IDLE:")) {
      const en = text.substring(10).trim() === "1";
      if (typeof window.applyAutoIdleState === "function") {
        window.applyAutoIdleState(en);
      }
      return;
    }

    if (text.startsWith("CAL:")) {
      const vals = text.substring(4).split(",").map(n => parseInt(n.trim(), 10));
      if (vals.length >= 3 && typeof window.applyReceivedOffsets === "function") {
        window.applyReceivedOffsets(vals[0], vals[1], vals[2], vals[3]);
      }
      return;
    }

    if (text.startsWith("ACT:")) {
      if (typeof window.applyReceivedAction === "function") {
        window.applyReceivedAction(text.substring(4));
      }
      return;
    }

    if (text.startsWith("ACT_STATE:")) {
      const mRun = text.match(/running=(\d+)/);
      const mSlot = text.match(/slot=(\d+)/);
      const mLoop = text.match(/loop=(\d+)/);
      if (typeof window.updateActionRunBadge === "function") {
        window.updateActionRunBadge(
          mRun ? mRun[1] === "1" : false,
          mSlot ? parseInt(mSlot[1], 10) : 0,
          mLoop ? mLoop[1] === "1" : false
        );
      }
      return;
    }

    if (text.startsWith("STATUS:")) {
      const ms1 = text.match(/s1=(\d+)/);
      const ms2 = text.match(/s2=(\d+)/);
      const ms3 = text.match(/s3=(\d+)/);
      const ms4 = text.match(/s4=(\d+)/);
      const mo1 = text.match(/o1=(-?\d+)/);
      const mo2 = text.match(/o2=(-?\d+)/);
      const mo3 = text.match(/o3=(-?\d+)/);
      const mo4 = text.match(/o4=(-?\d+)/);
      const mMin = text.match(/min=(\d+)/);
      const mMax = text.match(/max=(\d+)/);
      const me1 = text.match(/en1=(\d+)/);
      const me2 = text.match(/en2=(\d+)/);
      const me3 = text.match(/en3=(\d+)/);
      const me4 = text.match(/en4=(\d+)/);
      const mIdle = text.match(/idle=(\d+)/);
      const mRun = text.match(/run=(\d+)/);
      const mAct = text.match(/act=(\d+)/);
      const mLoop = text.match(/loop=(\d+)/);
      const mBo = text.match(/bo=(\d+)/);
      const mMpuc = text.match(/mpuc=(\d+)/);

      if (mBo && mBo[1] === "1" && !brownoutNotified) {
        brownoutNotified = true;
        appendLog("Warn", "ESP32 previously rebooted due to BROWNOUT (battery voltage sag).");
        showToast("Warning: Battery sag caused brownout reset!", "warn");
      }

      if (mMin && mMax && typeof window.applyReceivedRange === "function") {
        window.applyReceivedRange(parseInt(mMin[1], 10), parseInt(mMax[1], 10));
      }
      if (ms1 && ms2 && ms3 && typeof window.applyReceivedAngles === "function") {
        const s4Val = ms4 ? parseInt(ms4[1], 10) : undefined;
        window.applyReceivedAngles(parseInt(ms1[1], 10), parseInt(ms2[1], 10), parseInt(ms3[1], 10), s4Val, true);
      }
      if (mo1 && mo2 && mo3 && typeof window.applyReceivedOffsets === "function") {
        const o4Val = mo4 ? parseInt(mo4[1], 10) : undefined;
        window.applyReceivedOffsets(parseInt(mo1[1], 10), parseInt(mo2[1], 10), parseInt(mo3[1], 10), o4Val);
      }
      if (me1 && me2 && me3 && typeof window.applyReceivedEnableStates === "function") {
        const e4Val = me4 ? (me4[1] === "1") : undefined;
        window.applyReceivedEnableStates(me1[1] === "1", me2[1] === "1", me3[1] === "1", e4Val);
      }
      if (mIdle && typeof window.applyAutoIdleState === "function") {
        window.applyAutoIdleState(mIdle[1] === "1");
      }
      if (mMpuc && typeof window.applyMpuControlFlag === "function") {
        window.applyMpuControlFlag(mMpuc[1] === "1");
      }
      if (typeof window.updateActionRunBadge === "function") {
        window.updateActionRunBadge(
          mRun ? mRun[1] === "1" : false,
          mAct ? parseInt(mAct[1], 10) : 0,
          mLoop ? mLoop[1] === "1" : false
        );
      }
    }
  });
}

function setUiState(statusText, dotColor, btnText) {
  const st = document.getElementById("status-text");
  if (st) st.innerText = statusText;
  const dot = document.getElementById("status-dot");
  if (dot) {
    dot.style.background = dotColor;
    dot.style.boxShadow = `0 0 6px ${dotColor}`;
  }
  const btn = document.getElementById("connect-btn");
  if (btn) {
    const sp = btn.querySelector("span");
    if (sp) sp.innerText = btnText;
  }
}

function appendLog(sender, text) {
  const term = document.getElementById("terminal");
  if (!term) return;
  const time = new Date().toLocaleTimeString().split(" ")[0];
  const entry = `[${time}] ${sender}: ${text}`;
  rawLogLines.push(entry);
  if (rawLogLines.length > 250) rawLogLines.shift();

  const div = document.createElement("div");
  let senderColor = "#38bdf8";
  if (sender === "NOTIF" || sender === "WEB-BLE" || sender === "READ") senderColor = "#34d399";
  if (sender === "Action") senderColor = "#c084fc";
  if (sender === "Trim" || sender === "Calibration" || sender === "MPU") senderColor = "#f59e0b";
  if (sender === "ESPNOW") senderColor = "#a855f7";
  if (sender === "Config") senderColor = "#818cf8";
  if (sender.includes("Error") || sender.includes("Warn")) senderColor = "#f87171";
  if (sender === "TX") senderColor = "#fbbf24";

  div.innerHTML = `<span style="color:rgba(255,255,255,0.3);">[${time}]</span> <span style="color:${senderColor}; font-weight:700;">${sender}:</span> <span>${escapeHtml(text)}</span>`;
  term.appendChild(div);
  term.scrollTop = term.scrollHeight;
}

function clearLogs() {
  rawLogLines = [];
  rxCount = 0;
  txCount = 0;
  updateCounterDisplay();
  const term = document.getElementById("terminal");
  if (term) term.innerHTML = '<div style="color:rgba(255,255,255,0.4); font-style:italic;">Logs cleared. Ready...</div>';
}

function escapeHtml(str) {
  return String(str).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");
}

function exportTerminalLogs() {
  const header = `ARM & SCISSOR LIFT DIAGNOSTIC LOG\nExported: ${new Date().toISOString()}\nBackend: ${bleBackend || "None"}\nRX: ${rxCount} | TX: ${txCount}\n\n`;
  const fullText = header + rawLogLines.join("\n");
  const filename = `arm-scissor-log-${new Date().toISOString().replace(/[:.]/g, "-")}.txt`;

  if (navigator.share && navigator.canShare) {
    try {
      const file = new File([fullText], filename, { type: "text/plain" });
      if (navigator.canShare({ files: [file] })) {
        navigator.share({ files: [file], title: "Arm & Scissor Log" }).catch(() => {});
        return;
      }
    } catch (e) {}
  }

  const blob = new Blob([fullText], { type: "text/plain;charset=utf-8" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

window.toggleConnection = toggleConnection;
window.connectDevice = connectDevice;
window.disconnectDevice = disconnectDevice;
window.sendCommand = sendCommand;
window.showToast = showToast;
window.appendLog = appendLog;
window.clearLogs = clearLogs;
window.exportTerminalLogs = exportTerminalLogs;
window.escapeHtml = escapeHtml;

export {
  isConnected,
  sendCommand,
  toggleConnection,
  connectDevice,
  disconnectDevice,
  showToast,
  appendLog,
  clearLogs,
  exportTerminalLogs,
  escapeHtml,
  initBackendUI
};

