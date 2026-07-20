# 第一步：GNU Radio 物理层怎么改（干扰波 / 信息波）

> 对应文件  
> - 干扰：`competition/jamming/tx_radio_jamming.grc`  
> - 信息：`competition/info/tx_radio_info.grc`  
>  
> 标签说明  
> - **【修旧 bug】**：即使不升新规则，旧版也该改，否则信息波难解 / 干扰不稳  
> - **【对齐新规则】**：规则手册 V2.1.0 / 协议 V2.0.0 的 SPS、Sensitivity 等  
> - **【可不动】**：本步先别动，避免一次改太多不好验证  

改完每个图后：`Generate` → 运行 → 看频谱/UDP 是否还有比特流出。  
**建议顺序：先改信息波（问题更大）→ 再改干扰波 → 两边都用新 SPS/Sensitivity。**

---

## 总览：你现在两张图里该动什么

| 序号 | 改什么 | 干扰波 GRC | 信息波 GRC | 类型 |
|------|--------|------------|------------|------|
| 1 | 拆掉 / 禁用发射通路 | 必做 | 必做（Sink 虽已断线，建议删掉） | 修旧 bug |
| 2 | 接收增益 | 建议 30～40 | **27 → 40～45** | 修旧 bug |
| 3 | GFSK `Samples/Symbol` | 52 → **47** | 52 → **47** | 对齐新规则 |
| 4 | GFSK / 变量 Sensitivity | 换新表 | 1.5756 → **1.5628** | 对齐新规则 |
| 5 | `Gain Mu` / `Freq Error` | 0.3→0.175，加 0.0048 | 同左 | 修旧 bug |
| 6 | 低通滤波 | 建议加窄带（按等级） | 300k → **约 280～300k 可先留**；优先别再收窄 | 修旧 bug（微调） |
| 7 | Squelch | 保持禁用或勿抬高门限 | 无 | 可不动 |
| 8 | 采样率 1M、RF BW 1M | 保持 | 保持 | 可不动 |
| 9 | UDP / XMLRPC 端口 | 保持 14348 / 8080 | 保持 14346 / 8081 | 可不动 |

---

## A. 信息波 `tx_radio_info.grc`（优先改）

对照你截图：Source →（频谱）→ LPF 300k → GFSK Demod → UDP 14346；左侧 Sink 已断开。

### A1. 删掉未使用的 PlutoSDR Sink

- **类型：【修旧 bug】**
- **怎么改：**  
  1. 选中左侧灰色 `PlutoSDR Sink`  
  2. `Delete` 删掉（或右键 Disable）  
  3. 若还有未接线的 `GFSK Mod` / `UDP Source`，一并删掉或 Disable  
- **原理：** 同机 TX 通路即使未接线，有时仍会占资源/带来干扰风险；比赛接收图应是纯 RX。  
- **验证：** 生成后 Python 里不应再出现 `iio_pluto_sink`。

### A2. 提高 Manual Gain

- **类型：【修旧 bug】**（与新规则无关；信息波官方 **-60 dBm**）
- **怎么改：** 打开 `PlutoSDR Source` → `Manual Gain (RX1)`：  
  - 现在：`27`  
  - 改成：先 **40**，现场再在 35～50 间调  
- **原理：** 信息波比干扰波弱约 50 dB，增益反而更低会直接解不出来。  
- **注意：** 增益过高会削波（频谱顶平）；以「能看到载波峰、不削顶」为准。

### A3. GFSK Demod：`Samples/Symbol` 52 → 47

- **类型：【对齐新规则】**
- **怎么改：** `GFSK Demod` → `Samples/Symbol`：`52` → **`47`**  
- **原理：** 新版官方 SPS=47，符号变快约 10.6%；仍用 52 会符号定时整体错位。  
- **旧版对照：** 若你要先验证「只修 bug、不升规则」，可暂时留 52；但你说要对齐新版，这里应改 47。

### A4. GFSK Demod：Sensitivity 1.5756 → 1.5628

- **类型：【对齐新规则】**
- **怎么改：** `GFSK Demod` → `Sensitivity`：`1.5756` → **`1.5628`**  
- **原理：** SPS 变了以后，同样带宽下频偏略变，官方表更新了 Sensitivity。  
- **建议：** 把 Sensitivity 也做成变量，方便以后改：

```text
新增 Variable:
  Id: target_sens
  Value: 1.5628

GFSK Demod → Sensitivity: target_sens
```

### A5. 放软符号时钟环

- **类型：【修旧 bug】**（开源默认值；弱信号更需要）
- **怎么改：** 在 `GFSK Demod` 里：

| 参数 | 现在 | 改成 |
|------|------|------|
| Gain Mu | 0.3（300m） | **0.175** |
| Mu | 0.5 | 保持 0.5 |
| Omega Relative Limit | 0.005 | 保持 0.005 |
| Freq Error | 0.0（若界面有） | **0.0048** |

- **原理：** `Gain Mu` 太大，时钟环抖动大，弱信号易失锁；`Freq Error` 给采样率/晶振一点点预偏。  
- **若某版 GRC 没有 Freq Error 字段：** 以你生成的 `tx_radio.py` 里 `freq_error=` 为准，或升级/手改生成脚本。

### A6. Low Pass Filter：先小改或不改

- **类型：【修旧 bug】（可选）**
- **建议第一步：** Cutoff 保持 **300k**，Transition **50k** 先不动。  
- **若后面误码仍高、邻频干扰大：** 可试 Cutoff **280k**（不要低于 ~260k，Δf≈249 kHz 会被切掉）。  
- **原理：** 滤波器要盖住 GFSK 频偏，又尽量挡住旁边干扰波。

### A7. 频谱监视接法（可选）

- **类型：【修旧 bug】（体验向）**
- 你截图里 Frequency Sink 接在 Source 上（滤波前），名字却叫 `info_flitered`。  
- **可选改法：** 把频谱改接到 **LPF 输出**，才能看到滤完后的样子。  
- 不影响解调正确性，只影响你调参时的观感。

### A8. 信息波本步【可不动】

| 项 | 值 | 原因 |
|----|-----|------|
| samp_rate | 1M | 新版仍是 1M 等效符号采样 |
| RF Bandwidth | 1M | 足够覆盖 0.54 MHz 信息波 |
| UDP 14346 / XMLRPC 8081 | 保持 | 与 `info_decoder` 对齐 |
| URI | 你现场的 Pluto IP | 两套设备各用各的 |

---

## B. 干扰波 `tx_radio_jamming.grc`

对照你截图：Source(gain 20) → Squelch → GFSK → UDP 14348；同时有 Sink 在发射。

### B1. 拆掉发射通路（比赛接收模式）

- **类型：【修旧 bug】**
- **怎么改：**  
  1. 断开 `GFSK Mod` → `PlutoSDR Sink`  
  2. Delete/Disable：`PlutoSDR Sink`、`GFSK Mod`、`UDP Source`（若只做接收）  
- **原理：** 接收干扰密钥不需要本机发射；TX 开着会抬底噪、占 USB。  
- **例外：** 你若故意用同一 Pluto 做自环测试，可另存 `*_loopback.grc`，比赛用纯 RX 图。

### B2. 接收增益

- **类型：【修旧 bug】**
- **怎么改：** `Manual Gain`：截图是 **20**（偏保守）→ 建议先 **35～40**。  
- 干扰波官方 **-10 dBm** 很强，一般不需要比信息波更高；但 20 在远场/天线差时可能偏小。  
- **原则：** 频谱上干扰峰清晰、不削顶即可。

### B3. `Samples/Symbol` 52 → 47

- **类型：【对齐新规则】**
- **怎么改：**  
  - `GFSK Demod` → 52 → **47**  
  - 若还留着 `GFSK Mod`（自环用）→ 同样改成 **47**  
- 与信息波必须一致，否则一边新一边旧会混乱。

### B4. Sensitivity：变量 `target_sens` 换新初值 + 扫频表

- **类型：【对齐新规则】**
- **怎么改：**  
  1. Variable `target_sens`：`2.8323` → **`2.8194`**（一级干扰默认）  
  2. `GFSK Demod` / `GFSK Mod` 的 Sensitivity 继续绑 `target_sens`（RPC 扫频会改它）  
  3. **下一步改 Python 解码器**时，把扫频表改成：

```text
一级 2.8194
二级 2.5681
三级 0.6517
```

- **本步 GRC 只改默认初值即可**；三级扫频靠 XMLRPC `set_target_sens`，表在 decoder 里。

### B5. 同样放软时钟环

- **类型：【修旧 bug】**
- 与信息波相同：

| 参数 | 改成 |
|------|------|
| Gain Mu | **0.175** |
| Freq Error | **0.0048** |

### B6. Simple Squelch

- **类型：【可不动 / 小心动】**
- 仓库 `.grc` 里有时是 **disabled**；你截图是接在链上、门限 **-100**。  
- **-100 基本等于常开**，一般没事。  
- **不要**把门限抬到 -50 之类：弱包/间隙会被静音，比特流断流。  
- 若怀疑 squelch 捣乱：右键该块 → **Disable**，Source 直连 GFSK Demod。

### B7. 建议给干扰波也加 LPF（可选，修旧 bug）

开源按干扰等级换带宽。你双 SDR 可简化：

| 场景 | Cutoff 建议 |
|------|-------------|
| 主要盯一级/二级 | 450k～500k |
| 主要盯三级（0.25 MHz） | 125k～150k |

第一步也可以 **不加**，先把 SPS/Sens/增益改对；误码高再加。

### B8. 干扰波本步【可不动】

| 项 | 说明 |
|----|------|
| samp_rate 1M、RF BW 1M | 新版不变 |
| UDP 14348、XMLRPC 8080 | 对齐 `jamming_decoder_f1.py` |
| `target_freq` 初值 | 仍可由 decoder 扫频覆盖；确认扫的是**己方基座**频点（下一步改 Python） |

---

## C. 两张图改完后的「标准参数速查」

### 信息波（纯 RX）

```text
Pluto Source
  LO: target_freq (433.2M 红方 / 433.92M 蓝方，己方颜色)
  Sample Rate: 1M
  RF BW: 1M
  Manual Gain: 40（再调）
  TX Sink: 删除

LPF（可选保留）
  Cutoff: 300k
  Transition: 50k

GFSK Demod
  Samples/Symbol: 47          ← 新规则
  Sensitivity: 1.5628         ← 新规则
  Gain Mu: 0.175              ← 旧 bug
  Freq Error: 0.0048          ← 旧 bug

UDP → 127.0.0.1:14346
XMLRPC → 8081
```

### 干扰波（纯 RX）

```text
Pluto Source
  LO: target_freq（RPC 扫频）
  Manual Gain: 35～40
  TX Sink / Mod: 删除（比赛）

GFSK Demod
  Samples/Symbol: 47          ← 新规则
  Sensitivity: target_sens
  target_sens 初值: 2.8194    ← 新规则（一级）
  Gain Mu: 0.175              ← 旧 bug
  Freq Error: 0.0048          ← 旧 bug

Squelch: 禁用或保持 -100
UDP → 127.0.0.1:14348
XMLRPC → 8080
```

---

## D. 在 GRC 里操作的具体点击顺序（防漏）

对每个 `.grc`：

1. 打开对应 tab  
2. 按上表改块参数（双击块 → Properties → OK）  
3. 删掉无用 TX 块  
4. 菜单 **Generate**（或工具栏生成）  
5. 确认生成的 `tx_radio.py` 里出现 `samples_per_symbol=47` 等  
6. 运行，看 Frequency Sink 有无峰、解码端 UDP 是否还在涨  

**注意：** 只改 `.grc` 却运行旧的 `tx_radio.py`，等于没改。以 Generate 后的 Python 为准；若 decoder 里写死了 GRC 脚本路径，确认指向新生成文件。

---

## E. 本步不要做的事（留给后续步骤）

这些 **不是 GRC 能单独解决的**，下一步再改 Python 解码器：

| 项目 | 类型 |
|------|------|
| 信息波极性双搜 | 修旧 bug |
| Header=`000F000F` 校验后再拼包 | 修旧 bug |
| `0x0A05` 41 字节解析 | 对齐新规则 |
| 干扰扫频 Sensitivity 全表 + 己方频点语义 | 修旧 bug + 新规则 |
| 密钥投票 / partial frame | 修旧 bug（增强） |

---

## F. 改完怎么自检（最小实验）

1. **信息波 GRC 单独跑**  
   - 增益 40，频点己方广播  
   - 频谱上应能看到相对弱的峰  
   - UDP 14346 用 `nc -u -l 14346 | head` 或你们 recorder 应有 0/1 流  

2. **故意设错 SPS=52 再改回 47**  
   - 体会「对齐新规则」参数错时，后面 CRC 全挂  

3. **干扰波**  
   - 纯 RX、Sens=2.8194、SPS=47  
   - 原能解密钥的环境应仍能出比特；稳定后再动 decoder  

---

## G. 一句话记住

- **修旧 bug（GRC）：** 纯 RX、信息增益加够、时钟环放软、别让 squelch/TX 害你。  
- **对齐新规则（GRC）：** 两边 `Samples/Symbol=47`；信息 Sens=`1.5628`；干扰默认 Sens=`2.8194`（其余等级交给 RPC 表）。  

你按 A→B 改完并 Generate 后，把两张新截图发我，我可以帮你做参数核对；然后进入第二步：改 `info_decoder` / `jamming_decoder`。
