#!/usr/bin/env python3
"""Generate ACU v3 documentation with Mermaid.ink diagrams."""

import base64
import json
from pathlib import Path

HERE = Path(__file__).parent
OUT = HERE / "index.html"

THEME_VARS = {
    "primaryColor": "#1a2530",
    "primaryTextColor": "#b3c7e5",
    "primaryBorderColor": "#00d4ff",
    "lineColor": "#4a5a70",
    "secondaryColor": "#111820",
    "tertiaryColor": "#0a0e14",
}


def mermaid_url(code: str) -> str:
    """Encode Mermaid code into a mermaid.ink SVG URL."""
    payload = {
        "code": code,
        "mermaid": {"theme": "dark"},
        "themeVariables": THEME_VARS,
    }
    raw = json.dumps(payload, separators=(",", ":")).encode()
    b64 = base64.urlsafe_b64encode(raw).rstrip(b"=").decode()
    return f"https://mermaid.ink/svg/{b64}"


# ─── Architecture ───────────────────────────────────────────────────
ARCH = """graph LR
    subgraph ext[External ECUs]
        JETSON["JETSON (0x061)"]
        VCU["VCU (0x600)"]
        DVAQT["DV / AQT (0x770 / 0x502)"]
    end
    subgraph acu[ACU - STM32F412Rx]
        ACU["ACU v3"]
    end
    subgraph peri[Onboard Peripherals]
        CAN["CAN Bus (1 Mbps)"]
        ADC["ADC + DMA (Pressure)"]
        UART["UART + GPIO (BLE / ASSI)"]
        SOL["Solenoid Driver (EBS)"]
        TIM["TIM + I2C (WDT / EEPROM)"]
    end
    JETSON --- ACU
    VCU --- ACU
    DVAQT --- ACU
    ACU --- CAN
    ACU --- ADC
    ACU --- UART
    ACU --- SOL
    ACU --- TIM"""

# ─── Vehicle State Machine ──────────────────────────────────────────
VEHICLE_SM = """stateDiagram-v2
    [*] --> IDLE: WDT Enable
    IDLE --> AS_ON: ASMS=1 & IGN=0
    AS_ON --> EMERGENCY: Fault
    EMERGENCY --> IDLE: ASMS=0 & rpm<10
    state EMERGENCY {
        Handle_Emergency
    }
    state AS_ON {
        Handle_autonomous_state
    }"""

# ─── Autonomous System ──────────────────────────────────────────────
AUTO_SM = """stateDiagram-v2
    OFF --> Initial_Sequence: AS_ON entry
    Initial_Sequence --> Monitor_Sequence: Sequence OK
    Monitor_Sequence --> Finish: ASMS=0 & rpm<10
    Finish --> IDLE: Vehicle_{rarr}_IDLE
    Initial_Sequence --> AS_Emergency: Timeout / fault
    Monitor_Sequence --> AS_Emergency: Mission mismatch
    AS_Emergency --> EMERGENCY"""

# ─── Startup Sequence ───────────────────────────────────────────────
STARTUP = """flowchart TD
    W["Watchdog_check SDC=1 500ms"] --> P["Pressure_check 6-10bar 2000ms"]
    P --> H["HV_activation IGN_{rarr}ON 5000ms"]
    H --> C["Correlation_check Hyd>10xPneu 2000ms"]
    C --> M1["MB1_Check S1=1 S2=0 2000ms"]
    M1 --> M2["MB2_Check S1=0 S2=1 2000ms"]
    M2 --> READY["READY"]
    W -->|timeout| ERR["Error_state"]
    P -->|timeout| ERR
    H -->|timeout| ERR
    C -->|timeout| ERR
    M1 -->|timeout| ERR
    M2 -->|timeout| ERR
    ERR --> EMERGENCY"""

# ─── BLE State Machine ──────────────────────────────────────────────
BLE_SM = """stateDiagram-v2
    BLE_IDLE --> ENTER_CMD: Send $$$
    ENTER_CMD --> CONNECT: Wait CMD> / Connect
    CONNECT --> READ_CURRENT_TIME: Wait CONNECTED / Read CTS
    READ_CURRENT_TIME --> PROCESS_DATE: Parse CHR / Set RTC
    PROCESS_DATE --> EXIT_CMD: Disconnect / Exit cmd
    EXIT_CMD --> BRIDGE
    ENTER_CMD --> EXIT_CMD: timeout 1s
    CONNECT --> EXIT_CMD: timeout 8s
    READ_CURRENT_TIME --> EXIT_CMD: timeout 3s
    EXIT_CMD --> BRIDGE: ERR / DISCON"""

# ─── Main Loop ──────────────────────────────────────────────────────
MAIN_LOOP = """flowchart LR
    A["Peripheral_aquisition"] --> B["Handle_state"]
    B --> C["toggle_wdt"]
    C --> D["LED_indicator"]
    D --> E["ASSI_control"]
    E --> F["Peripheral_actuation"]
    F --> G["handle_can_tx"]
    G --> H["can_buffer_pop"]
    H --> I["dbc_decode"]"""

# ─── Module Flowcharts ──────────────────────────────────────────────
MOD_INIT_SEQ = """flowchart TD
    START(["Start"]) --> W["Watchdog_check"]
    W --> SDC{"SDC == 1?"}
    SDC -->|N| ERR1["Error_state (timeout 500ms)"]
    SDC -->|Y| BP{"6 < BP < 10?"}
    BP -->|N| ERR2["Error_state (timeout 2s)"]
    BP -->|Y| CORR{"Hyd > 10x Pneu?"}
    CORR -->|N| ERR3["Error_state"]
    CORR -->|Y| OK(["Sequence OK"])"""

MOD_HANDLE_EMERG = """flowchart TD
    A["Disable WDT"] --> B["Clear Ignition"]
    B --> C["Open Solenoids"]"""

MOD_PERIPH_AQ = """flowchart TD
    A["Read ASMS"] --> B["Read SDC"]
    B --> C["Read IGN"]
    C --> D["Read ASSI"]"""

MOD_BLE = """flowchart TD
    S(["Idle"]) --> ENTER{"ENTER_CMD?"}
    ENTER -->|N| S
    ENTER -->|Y| CMD{"CMD > ?"}
    CMD -->|N| ENTER
    CMD -->|Y| CONN["Connect"]
    CONN --> OK{"CONNECTED?"}
    OK -->|N| CONN
    OK -->|Y| CTS["Read CTS"]
    CTS --> CHR{"CHR?"}
    CHR -->|N| CTS
    CHR -->|Y| RTC["Set RTC"]
    RTC --> EXIT["EXIT_CMD"]
    EXIT --> BRIDGE["BRIDGE"]"""

MOD_CAN_POP = """flowchart TD
    S(["Enter"]) --> C{"counter == 0?"}
    C -->|Y| RET["return NULL"]
    C -->|N| DEC{"tx or rx?"}
    DEC -->|TX| TXB["CAN_AddTxMessage"]
    DEC -->|RX| RXB["memcpy to buffer"]"""

MOD_EMA = """flowchart TD
    S(["Enter"]) --> INIT{"initialized?"}
    INIT -->|N| SET["y = input"]
    INIT -->|Y| CALC["y = alpha*x + (1-alpha)*y_prev"]
    SET --> OUT(["return y"])
    CALC --> OUT"""

MOD_EEPROM = """flowchart TD
    S(["Enter"]) --> HDR["Read header"]
    HDR --> MAGIC{"magic == 0xDEADBEEF?"}
    MAGIC -->|N| INIT["Initialize EEPROM"]
    INIT --> HDR
    MAGIC -->|Y| WRITE["Write data @ tail"]
    WRITE --> ADV["Advance tail"]
    ADV --> CHK{"count == MAX?"}
    CHK -->|N| INC["count++"]
    CHK -->|Y| HEAD["head++"]
    HEAD --> INC
    INC --> D(["Done"])"""


def img_html(code: str, alt: str = "", style: str = "") -> str:
    url = mermaid_url(code)
    s = f'style="max-width:100%;height:auto;{style}"' if style else 'style="max-width:100%;height:auto"'
    return f'<img src="{url}" alt="{alt}" loading="lazy" {s}>'


HTML = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ACU v3 — Interactive Documentation</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600;700;800&display=swap');
*, *::before, *::after {{ box-sizing: border-box; margin: 0; padding: 0; }}
:root {{
  --bg: #0a0e14;
  --bg2: #111820;
  --bg3: #1a2530;
  --fg: #b3c7e5;
  --accent: #00d4ff;
  --green: #00ff9d;
  --red: #ff3b5c;
  --orange: #ff9f43;
  --yellow: #ffd93d;
  --purple: #a855f7;
  --dim: #4a5a70;
  --border: #1e2d3d;
  --radius: 12px;
  --font: 'JetBrains Mono', 'Consolas', 'Courier New', monospace;
}}
html {{ font-size: 15px; scroll-behavior: smooth; }}
body {{
  font-family: var(--font);
  background: var(--bg);
  color: var(--fg);
  line-height: 1.5;
  min-height: 100vh;
}}
a {{ color: var(--accent); text-decoration: none; }}
a:hover {{ text-decoration: underline; }}
.container {{ max-width: 1200px; margin: 0 auto; padding: 0 20px; }}
.hero {{
  padding: 60px 0 40px;
  text-align: center;
  border-bottom: 1px solid var(--border);
  margin-bottom: 40px;
}}
.hero h1 {{
  font-size: 3rem;
  font-weight: 800;
  background: linear-gradient(135deg, var(--accent), var(--green));
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
  letter-spacing: -1px;
}}
.hero .subtitle {{
  font-size: 1.1rem;
  color: var(--dim);
  margin: 10px 0 20px;
}}
.hero .badges {{ display: flex; gap: 10px; justify-content: center; flex-wrap: wrap; }}
.badge {{
  display: inline-block;
  padding: 4px 14px;
  border-radius: 20px;
  font-size: .75rem;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: .5px;
}}
.badge-cyan {{ background: rgba(0,212,255,.15); color: var(--accent); border: 1px solid rgba(0,212,255,.3); }}
.badge-green {{ background: rgba(0,255,157,.15); color: var(--green); border: 1px solid rgba(0,255,157,.3); }}
.badge-red {{ background: rgba(255,59,92,.15); color: var(--red); border: 1px solid rgba(255,59,92,.3); }}
.badge-purple {{ background: rgba(168,85,247,.15); color: var(--purple); border: 1px solid rgba(168,85,247,.3); }}
section {{ margin-bottom: 60px; }}
.section-title {{
  font-size: 1.4rem;
  font-weight: 700;
  color: var(--accent);
  margin-bottom: 8px;
  display: flex; align-items: center; gap: 10px;
}}
.section-title::before {{
  content: '';
  display: inline-block;
  width: 4px; height: 24px;
  background: var(--accent);
  border-radius: 2px;
}}
.section-desc {{ color: var(--dim); font-size: .85rem; margin-bottom: 24px; }}
.tabs {{ display: flex; gap: 4px; margin-bottom: 20px; flex-wrap: wrap; }}
.tab-btn {{
  padding: 8px 18px;
  border: 1px solid var(--border);
  background: var(--bg2);
  color: var(--dim);
  cursor: pointer;
  border-radius: 8px 8px 0 0;
  font-family: var(--font);
  font-size: .8rem;
  font-weight: 600;
  transition: all .2s;
}}
.tab-btn:hover {{ color: var(--fg); border-color: var(--accent); }}
.tab-btn.active {{ background: var(--bg3); color: var(--accent); border-color: var(--accent); border-bottom-color: var(--bg3); }}
.tab-content {{ display: none; }}
.tab-content.active {{ display: block; }}
.diagram-box {{
  background: var(--bg2);
  border: 1px solid var(--border);
  border-radius: var(--radius);
  padding: 24px 16px;
  margin-bottom: 24px;
  overflow-x: auto;
  display: flex;
  flex-direction: column;
  align-items: center;
}}
.diagram-box img {{ max-width: 100%; height: auto; display: block; }}
.caption {{
  font-size: .75rem;
  color: var(--dim);
  margin-top: 12px;
  text-align: center;
}}
.module-grid {{
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(360px, 1fr));
  gap: 20px;
}}
.module-card {{
  background: var(--bg2);
  border: 1px solid var(--border);
  border-radius: var(--radius);
  padding: 20px;
  display: flex;
  flex-direction: column;
  align-items: center;
  transition: all .3s;
}}
.module-card:hover {{ border-color: var(--accent); box-shadow: 0 0 20px rgba(0,212,255,.08); }}
.module-card h3 {{ font-size: .85rem; color: var(--accent); margin-bottom: 14px; font-weight: 700; width: 100%; }}
.module-card img {{ max-width: 100%; height: auto; display: block; }}
.legend {{
  display: flex;
  gap: 20px;
  flex-wrap: wrap;
  margin-bottom: 16px;
  font-size: .7rem;
  justify-content: center;
}}
.legend-item {{ display: flex; align-items: center; gap: 6px; }}
.legend-dot {{
  width: 10px; height: 10px;
  border-radius: 50%;
  display: inline-block;
}}
@media (max-width: 768px) {{
  .hero h1 {{ font-size: 2rem; }}
  .module-grid {{ grid-template-columns: 1fr; }}
}}
</style>
</head>
<body>

<div class="container">

<header class="hero">
  <h1>ACU v3</h1>
  <p class="subtitle">Autonomous Control Unit — STM32F412Rx / Formula Student T26</p>
  <div class="badges">
    <span class="badge badge-cyan">C</span>
    <span class="badge badge-green">STM32CubeMX</span>
    <span class="badge badge-red">CAN 2.0</span>
    <span class="badge badge-purple">BLE 4.2</span>
    <span class="badge badge-green">Watchdog HW</span>
    <span class="badge badge-cyan">EBS Brake</span>
  </div>
</header>

<!-- ═══════════ Architecture ═══════════ -->
<section id="architecture">
  <h2 class="section-title">Architecture Overview</h2>
  <p class="section-desc">ACU connects to external ECUs via CAN bus and drives local peripherals</p>
  <div class="diagram-box">
    {img_html(ARCH, "Architecture diagram showing ACU connected to external ECUs and onboard peripherals")}
    <p class="caption">System architecture — STM32F412Rx central hub connected to JETSON, VCU, DV/AQT via CAN bus, with onboard ADC, UART, solenoid driver, and timer peripherals</p>
  </div>
</section>

<!-- ═══════════ State Machines ═══════════ -->
<section>
  <h2 class="section-title">State Machines</h2>
  <p class="section-desc">Vehicle, Autonomous, Startup, and BLE state machines</p>

  <div class="tabs" id="sm-tabs">
    <button class="tab-btn active" data-tab="sm-vehicle">Vehicle</button>
    <button class="tab-btn" data-tab="sm-autonomous">Autonomous</button>
    <button class="tab-btn" data-tab="sm-startup">Startup</button>
    <button class="tab-btn" data-tab="sm-ble">BLE</button>
  </div>

  <div class="tab-content active" id="sm-vehicle">
    <div class="diagram-box">
      {img_html(VEHICLE_SM, "Vehicle state machine diagram")}
      <p class="caption">Start → IDLE → AS_ON → EMERGENCY → IDLE cycle with WDT, ASMS/IGN, and fault transitions</p>
    </div>
  </div>

  <div class="tab-content" id="sm-autonomous">
    <div class="diagram-box">
      {img_html(AUTO_SM, "Autonomous system state machine")}
      <p class="caption">OFF → Initial_Sequence → Monitor_Sequence → Finish → IDLE with error paths to EMERGENCY</p>
    </div>
  </div>

  <div class="tab-content" id="sm-startup">
    <div class="diagram-box">
      {img_html(STARTUP, "Startup sequence flowchart")}
      <p class="caption">Six-phase startup: Watchdog → Pressure → HV → Correlation → MB1 → MB2 → READY, with timeout paths to Error_state</p>
    </div>
  </div>

  <div class="tab-content" id="sm-ble">
    <div class="diagram-box">
      {img_html(BLE_SM, "BLE state machine diagram")}
      <p class="caption">BLE_IDLE → ENTER_CMD → CONNECT → READ_CURRENT_TIME → PROCESS_DATE → EXIT_CMD → BRIDGE with timeout abort paths</p>
    </div>
  </div>
</section>

<!-- ═══════════ Main Loop ═══════════ -->
<section>
  <h2 class="section-title">Main Loop — app()</h2>
  <p class="section-desc">Called from main.c while(1), executes every tick (~1 ms)</p>
  <div class="diagram-box">
    {img_html(MAIN_LOOP, "Main loop flowchart showing execution order")}
    <p class="caption">Peripheral_aquisition → Handle_state → toggle_wdt → LED_indicator → ASSI_control → Peripheral_actuation → handle_can_tx → can_buffer_pop → dbc_decode</p>
  </div>
</section>

<!-- ═══════════ Module Flowcharts ═══════════ -->
<section>
  <h2 class="section-title">Module Flowcharts</h2>
  <p class="section-desc">Logic diagrams for key modules</p>

  <div class="module-grid">
    <div class="module-card">
      <h3>initial_sequence</h3>
      {img_html(MOD_INIT_SEQ, "Initial sequence flowchart", "max-width:320px;")}
      <p class="caption">SDC check, pressure check, correlation check with error paths</p>
    </div>
    <div class="module-card">
      <h3>Handle_Emergency</h3>
      {img_html(MOD_HANDLE_EMERG, "Emergency handler flowchart", "max-width:280px;")}
      <p class="caption">Disable WDT → Clear Ignition → Open Solenoids</p>
    </div>
    <div class="module-card">
      <h3>Peripheral_aquisition</h3>
      {img_html(MOD_PERIPH_AQ, "Peripheral acquisition flowchart", "max-width:280px;")}
      <p class="caption">Read ASMS → SDC → IGN → ASSI</p>
    </div>
    <div class="module-card">
      <h3>BLE Handler</h3>
      {img_html(MOD_BLE, "BLE handler flowchart", "max-width:320px;")}
      <p class="caption">ENTER_CMD → connect → read CTS → set RTC → EXIT → BRIDGE</p>
    </div>
    <div class="module-card">
      <h3>can_buffer_pop</h3>
      {img_html(MOD_CAN_POP, "CAN buffer pop flowchart", "max-width:280px;")}
      <p class="caption">Check counter → TX: CAN_AddTxMessage | RX: memcpy</p>
    </div>
    <div class="module-card">
      <h3>EMA Filter</h3>
      {img_html(MOD_EMA, "EMA filter flowchart", "max-width:280px;")}
      <p class="caption">y = input if uninitialized, else y = alpha·x + (1-alpha)·y_prev</p>
    </div>
    <div class="module-card">
      <h3>EEPROM Logger</h3>
      {img_html(MOD_EEPROM, "EEPROM logger flowchart", "max-width:320px;")}
      <p class="caption">Read header → validate magic → write @tail → advance → handle count/head wrap</p>
    </div>
  </div>
</section>

<!-- ═══════════ Legend ═══════════ -->
<section>
  <h2 class="section-title">Legend</h2>
  <div class="legend">
    <span class="legend-item"><span class="legend-dot" style="background:var(--accent)"></span> Process / State box</span>
    <span class="legend-item"><span class="legend-dot" style="background:var(--green)"></span> Success / Yes path</span>
    <span class="legend-item"><span class="legend-dot" style="background:var(--red)"></span> Error / No / Timeout path</span>
    <span class="legend-item"><span class="legend-dot" style="background:var(--orange)"></span> Decision diamond</span>
    <span class="legend-item"><span class="legend-dot" style="background:var(--purple)"></span> External ECU</span>
    <span class="legend-item"><span class="legend-dot" style="background:var(--yellow)"></span> Startup phase</span>
  </div>
</section>

</div>

<script>
(function(){{
  var tabs = document.getElementById('sm-tabs');
  if (!tabs) return;
  var btns = tabs.querySelectorAll('.tab-btn');
  btns.forEach(function(btn){{
    btn.addEventListener('click', function(e){{
      var target = document.getElementById(this.dataset.tab);
      if (!target) return;
      btns.forEach(function(b){{ b.classList.remove('active'); }});
      this.classList.add('active');
      document.querySelectorAll('.tab-content').forEach(function(tc){{ tc.classList.remove('active'); }});
      target.classList.add('active');
    }});
  }});
}})();
</script>

</body>
</html>
"""


def generate():
    OUT.write_text(HTML, encoding="utf-8")
    print(f"Generated {OUT}")


if __name__ == "__main__":
    generate()
