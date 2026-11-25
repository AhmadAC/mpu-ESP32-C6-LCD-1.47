import { initBackendUI, sendCommand } from "./ble.js";
import { initServoStorage, setServoPin, setAllPins, saveHardwarePins, requestPinSync } from "./servos.js";
import { initActionsStorage } from "./actions.js";
import { initImuUI } from "./imu.js";

function handleSendCustom(e) {
  if (e && typeof e.preventDefault === "function") e.preventDefault();
  const input = document.getElementById("cmd-input");
  if (!input) return;
  const val = input.value.trim();
  if (!val) return;
  sendCommand(val);
  input.value = "";
}

window.handleSendCustom = handleSendCustom;
window.setServoPin = setServoPin;
window.setAllPins = setAllPins;
window.saveHardwarePins = saveHardwarePins;
window.requestPinSync = requestPinSync;

window.addEventListener("DOMContentLoaded", () => {
  initBackendUI();
  initServoStorage();
  initActionsStorage();
  initImuUI();
});

export { handleSendCustom };

