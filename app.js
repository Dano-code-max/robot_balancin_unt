// --- UUIDs NORDIC UART SERVICE (Coinciden con el ESP32) ---
const NUS_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const NUS_RX_UUID      = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'; // Escribir al ESP32
const NUS_TX_UUID      = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'; // Recibir del ESP32

// --- REFERENCIAS DOM ---
const btnViewGraph   = document.getElementById('btnViewGraph');
const btnViewConsole = document.getElementById('btnViewConsole');
const viewEmpty      = document.getElementById('viewEmpty');
const viewGraph      = document.getElementById('viewGraph');
const viewConsole    = document.getElementById('viewConsole');
const consoleOutput  = document.getElementById('consoleOutput');

const btnDemo = document.getElementById('btnDemo');
const btnBt   = document.getElementById('btnBt');

// --- ESTADO BLUETOOTH Y CONSOLA ---
let bleDevice = null;
let rxCharacteristic = null;
let txCharacteristic = null;
let rxBuffer = '';
let consolaPausada = false;
const encoder = new TextEncoder();
const decoder = new TextDecoder('utf-8');

// Historial de datos (últimas 100 muestras)
const MAX_POINTS = 100;
const dataAng  = [];
const dataPos  = [];
const dataVolt = [];

// --- 1. CONMUTACIÓN DEL DISPLAY CENTRAL (Inicia vacío) ---
function setActiveView(viewName) {
  viewEmpty.classList.remove('active');
  viewGraph.classList.remove('active');
  viewConsole.classList.remove('active');
  btnViewGraph.classList.remove('active');
  btnViewConsole.classList.remove('active');

  if (viewName === 'graph') {
    viewGraph.classList.add('active');
    btnViewGraph.classList.add('active');
    dibujarTodasLasGraficas();
  } else if (viewName === 'console') {
    viewConsole.classList.add('active');
    btnViewConsole.classList.add('active');
  } else {
    viewEmpty.classList.add('active');
  }
}

btnViewGraph.addEventListener('click', () => setActiveView('graph'));
btnViewConsole.addEventListener('click', () => setActiveView('console'));

// --- 2. PROCESAMIENTO DE LÍNEAS RECIBIDAS DEL ESP32 ---
function procesarLineaESP32(linea) {
  // Si está en STOP, ignora telemetría residual pero sí muestra mensajes de estado (#) y comandos (>)
  if (consolaPausada && !linea.startsWith('#') && !linea.startsWith('>')) {
    return;
  }

  const p = document.createElement('div');
  p.textContent = linea;
  consoleOutput.appendChild(p);
  if (consoleOutput.children.length > 120) {
    consoleOutput.removeChild(consoleOutput.firstChild);
  }
  consoleOutput.scrollTop = consoleOutput.scrollHeight;

  // Telemetría CSV de 6 columnas: millis, V, anguloY, posProm, posIzq, posDer
  const partes = linea.split(',');
  if (partes.length === 6) {
    const volt = parseFloat(partes[1]);
    const ang  = parseFloat(partes[2]);
    const pos  = parseFloat(partes[3]);

    if (!isNaN(volt) && !isNaN(ang) && !isNaN(pos)) {
      document.getElementById('valAng').textContent  = ang.toFixed(2);
      document.getElementById('valPos').textContent  = pos.toFixed(1);
      document.getElementById('valVolt').textContent = volt.toFixed(2);

      pushSample(dataAng, ang);
      pushSample(dataPos, pos);
      pushSample(dataVolt, volt);

      if (viewGraph.classList.contains('active')) {
        dibujarTodasLasGraficas();
      }
    }
  }
}

function pushSample(arr, val) {
  arr.push(val);
  if (arr.length > MAX_POINTS) arr.shift();
}

// --- 3. MOTOR DE GRÁFICAS CON AUTO-RANGO DINÁMICO ---
function dibujarGrafica(canvasId, rangeId, datos, colorLinea, minSpan, decimales) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return;
  const rect = canvas.getBoundingClientRect();
  if (rect.width === 0 || rect.height === 0) return;

  canvas.width = rect.width;
  canvas.height = rect.height;
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;

  ctx.clearRect(0, 0, w, h);

  let minVal = -minSpan / 2;
  let maxVal =  minSpan / 2;

  if (datos.length > 0) {
    const dMin = Math.min(...datos);
    const dMax = Math.max(...datos);
    const centro = (dMin + dMax) / 2;
    const span = Math.max((dMax - dMin) * 1.25, minSpan);
    minVal = centro - span / 2;
    maxVal = centro + span / 2;
  }

  const rangeEl = document.getElementById(rangeId);
  if (rangeEl) {
    rangeEl.textContent = `[${minVal.toFixed(decimales)} .. ${maxVal.toFixed(decimales)}]`;
  }

  ctx.strokeStyle = '#e5e7eb';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, h / 2);
  ctx.lineTo(w, h / 2);
  ctx.stroke();

  if (datos.length < 2) return;

  ctx.strokeStyle = colorLinea;
  ctx.lineWidth = 2;
  ctx.beginPath();

  datos.forEach((val, i) => {
    const x = (i / (MAX_POINTS - 1)) * w;
    const norm = (val - minVal) / (maxVal - minVal);
    const y = h - Math.max(0.03, Math.min(0.97, norm)) * h;
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

function dibujarTodasLasGraficas() {
  dibujarGrafica('canvasAng',  'rangeAng',  dataAng,  '#2563eb', 1.5, 1);
  dibujarGrafica('canvasPos',  'rangePos',  dataPos,  '#16a34a', 4.0, 1);
  dibujarGrafica('canvasVolt', 'rangeVolt', dataVolt, '#dc2626', 1.5, 1);
}

// --- 4. CONEXIÓN WEB BLUETOOTH REAL ---
async function conectarBLE() {
  if (!navigator.bluetooth) {
    alert('Este navegador no soporta Web Bluetooth. Ábrelo desde Chrome o Edge en http://127.0.0.1:5500.');
    return;
  }

  // Si ya está conectado, desconectar
  if (bleDevice && bleDevice.gatt.connected) {
    bleDevice.gatt.disconnect();
    return;
  }

  try {
    if (demoTimer) detenerDemo();

    btnBt.textContent = 'Buscando...';
    bleDevice = await navigator.bluetooth.requestDevice({
      filters: [
        { services: [NUS_SERVICE_UUID] },
        { namePrefix: 'Robot' }
      ],
      optionalServices: [NUS_SERVICE_UUID]
    });

    bleDevice.addEventListener('gattserverdisconnected', alDesconectarBLE);

    btnBt.textContent = 'Conectando...';
    const server  = await bleDevice.gatt.connect();
    const service = await server.getPrimaryService(NUS_SERVICE_UUID);

    rxCharacteristic = await service.getCharacteristic(NUS_RX_UUID);
    txCharacteristic = await service.getCharacteristic(NUS_TX_UUID);

    await txCharacteristic.startNotifications();
    txCharacteristic.addEventListener('characteristicvaluechanged', alRecibirDatosBLE);

    dataAng.length = 0;
    dataPos.length = 0;
    dataVolt.length = 0;
    rxBuffer = '';
    consolaPausada = false;

    btnBt.textContent = 'Conectado';
    btnBt.classList.add('connected');
    procesarLineaESP32(`#BLE_CONECTADO: ${bleDevice.name || 'RobotBalancin_UNT'}`);
  } catch (error) {
    console.error(error);
    alDesconectarBLE();
    procesarLineaESP32(`#ERROR_BT: ${error.name} - ${error.message}`);
  }
}

function alDesconectarBLE() {
  rxCharacteristic = null;
  txCharacteristic = null;
  consolaPausada = false;
  btnBt.textContent = 'Conectar BT';
  btnBt.classList.remove('connected');
  procesarLineaESP32('#BLE_DESCONECTADO');
}

function alRecibirDatosBLE(event) {
  const chunk = decoder.decode(event.target.value, { stream: true });
  rxBuffer += chunk;

  let saltoLinea;
  while ((saltoLinea = rxBuffer.indexOf('\n')) >= 0) {
    const linea = rxBuffer.slice(0, saltoLinea).trim();
    rxBuffer = rxBuffer.slice(saltoLinea + 1);
    if (linea.length > 0) {
      procesarLineaESP32(linea);
    }
  }
}

btnBt.addEventListener('click', conectarBLE);

// --- 5. ENVÍO UNIFICADO DE COMANDOS (STOP, RUN, ZERO Y FLECHAS) ---
let posOffsetDemo = 0;

async function enviarComando(cmd) {
  if (cmd === 'STOP') consolaPausada = true;
  if (cmd === 'RUN')  consolaPausada = false;

  procesarLineaESP32(`> ${cmd}`);

  if (rxCharacteristic) {
    try {
      const payload = encoder.encode(`${cmd}\n`);
      if (rxCharacteristic.writeValueWithoutResponse) {
        await rxCharacteristic.writeValueWithoutResponse(payload);
      } else {
        await rxCharacteristic.writeValue(payload);
      }
    } catch (err) {
      procesarLineaESP32(`#ERR_ENVIO: ${err.message}`);
    }
  } else if (cmd === 'ZERO' && demoTimer) {
    posOffsetDemo = Math.sin(tDemo * 0.8) * 8.0;
    dataPos.length = 0;
  }
}

document.querySelectorAll('[data-cmd]').forEach(btn => {
  btn.addEventListener('click', () => {
    const cmd = btn.getAttribute('data-cmd');
    enviarComando(cmd);
  });
});

// --- 6. MODO DEMO PARA PROBAR SIN ROBOT ---
let demoTimer = null;
let tDemo = 0;

function detenerDemo() {
  clearInterval(demoTimer);
  demoTimer = null;
  btnDemo.classList.remove('active');
  procesarLineaESP32('#DEMO_DETENIDO');
}

btnDemo.addEventListener('click', () => {
  if (demoTimer) {
    detenerDemo();
  } else {
    consolaPausada = false;
    btnDemo.classList.add('active');
    procesarLineaESP32('#DEMO_INICIADO');
    demoTimer = setInterval(() => {
      tDemo += 0.05;
      const v   = (Math.sin(tDemo * 3) * 1.4).toFixed(2);
      const ang = (5.75 + Math.cos(tDemo * 3) * 0.6).toFixed(2);
      const pos = (Math.sin(tDemo * 0.8) * 8.0 - posOffsetDemo).toFixed(2);
      procesarLineaESP32(`${Math.floor(tDemo * 1000)},${v},${ang},${pos},${pos},${pos}`);
    }, 50);
  }
});