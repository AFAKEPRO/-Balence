# ET08A(SBUS) 遥控接入 —— 进度交接（2026-09）

> 由上一段 Claude Code 会话从「0standard / 云台 A板 F427」转场到本工程。
> **新会话从这里继续：先读本文件，不要动 0standard 里的改动。**

## 我的身份 / 工程上下文

- **本人**：RoboMaster **平衡步兵**的嵌入式开发（称呼/队名：福建师范大学 **PKA战队**）。做过云台、底盘、遥控链路相关。
- **机器人**：平衡步兵（赛季 RM2026）。
  - 云台 = **A板 STM32F427**，跑旧工程「0standard」（Keil + SPL），是上一段会话的战场，**整车整合后退役，其 DBUS→SBUS 改动作废，别再改**。
  - 底盘 = **C板 STM32F407**，跑本「新框架」工程（CubeMX + HAL + FreeRTOS），**整车控制今后整合到这里**。
- **遥控硬件**：原用 DJI **DT7 + DR16**（拨杆太少才换）；现用**天地飞 ET08A**（8 通道）。接收机目前 **RF204W**（只 4 通道，拨杆 CH5~8 带不过来 → 需换 **RF206S** 或先实测）。
- **协议切换**：DBUS → SBUS（ET08A 的 W.BUS = S.BUS 兼容，**25 字节真 SBUS**，反相、板载反相电路已处理）。
- **开源参考**：队友提供 `github.com/Liam2756/ET08A-driver`（README 全文已读，F407+HAL 工程，接收机接 USART3/PC11 8E2/100000）。
- **工作方式约束（重要）**：**“告诉我怎么改 / 要改哪些，不要直接替我改代码。”** 教学式指导，代码由本人亲手改，改完贴给 Claude 核对。

## 一句话现状

> ⚠️ **这两段已被 2026-09-06 拍板 + 2026-09-12 定稿改写**：整车路径不是"C板 自读 USART3"，而是 **A板 读 SBUS → CAN 0x160/161/162 转发 DBUS18 → C板**。以文末「遥控链路定稿」一节为准。

ET08A 接收机接本板 **USART3**（现有遥控口），把 `remote_control` 模块从 **18 字节 DBUS** 解析改成 **25 字节 SBUS**。方案：**借开源驱动的解析，不搬整包**。

## 硬结论（代码证据，决定"借解析不搬整包"）

- 本工程 `User/Bsp/usart/bsp_usart.c`（第 91 行）**已实现 `HAL_UARTEx_RxEventCallback`** 统一分发（注册制：`USARTRegister` + `USARTServiceInit` + 模块回调，DMA+IDLE）。
- USART3 **已被 remote 注册**：`User/App/robot_cmd/robot_cmd.c:165` → `RemoteControlInit(&huart3)`。
- ⇒ 整包搬开源会**二次定义 `HAL_UARTEx_RxEventCallback`（链接冲突）+ USART3 双主人**，禁止整包搬；只抄它的 11bit 解包、通道表、拨杆阈值。
- 本工程 `remote_control.c` 现解 18B DBUS（`REMOTE_CONTROL_FRAME_SIZE 18u`，第 10 行），其中字节 6~15 解析鼠标/键盘——**真 SBUS(25B) 没有鼠标键盘**（那段是 CH5~16+标志位），鼠标键盘解析必须删，键鼠走裁判兜底（`RCLostCallback` 已有）。

## 待改清单（按顺序）

> ⚠️ **本节（第 1~5 条）已被 2026-09-12 的「遥控链路定稿」取代** —— 它是"C板 自读 USART3 25B SBUS"的早期方案，2026-09-06 拍板后降级为**台架备用**。**整车路径请看文末「遥控链路定稿」一节。** 保留在此仅为台架单测底盘时参考。

1. `User/Modules/remote/remote_control.c:10` → `REMOTE_CONTROL_FRAME_SIZE 25u`（同时是 DMA 接收缓冲长度）。
2. `Src/usart.c:80` → `huart3.Init.StopBits = UART_STOPBITS_2`（SBUS=8E2；现配置 100000 / 9bit含偶校验 / 1 停止位 = 8E1）。`.ioc` 同步。
3. `remote_control.c` 的 `sbus_to_rc()` 重写：
   - 帧头校验 `buf[0] == 0x0F`（DBUS 无头，SBUS 有）；
   - 11bit 解包 `bit = 8 + 11*i`（见下代码）；
   - 摇杆 CH1~4 减 `RC_CH_VALUE_OFFSET(1024)`；
   - 拨杆按整路 11bit 值阈值映射（不是 DBUS 的 2bit 码）：`<900→DOWN / >1300→UP / 中间→MID`；
   - **删鼠标/键盘段**；保留 `RectifyRCjoystick()`、daemon、`RCLostCallback` 裁判键鼠兜底。

```c
static uint16_t sbus_ch(const uint8_t *b, uint16_t i)
{
    uint32_t bit = 8u + 11u * i;          /* SBUS: 0x0F 头后每路占 11bit */
    uint32_t by  = bit >> 3;
    uint32_t sh  = bit & 7u;
    return (uint16_t)((((uint32_t)b[by]) |
                       (((uint32_t)b[by+1]) << 8) |
                       (((uint32_t)b[by+2]) << 16)) >> sh) & 0x07FFu;
}
/* 用法：rocker_* 取 CH0~3；sb=sbus_ch(buf,5) [CH6=SB 三档→switch_left 主模式]；sa=sbus_ch(buf,4) [CH5=SA 两档→switch_right] */
```

4. 通道表（ET08A 出厂默认）：CH1 右X / CH2 右Y / CH3 左Y / CH4 左X / **CH5 SA(两档)** / **CH6 SB(三档)** / CH7 SC(三档) / CH8 SD(两档)。
   `switch_left`（主模式，需三档）= SB = **CH6**；`switch_right` = SA = CH5。物理对应可按控制意图对调。
5. **接收机 RF204W 只有 4 通道** → 拨杆在 CH5~8 到不了板 → 需换 **RF206S**（README 配套款）或先实测 W.BUS 到底带不带 CH5~8。

## 关键文件

- 遥控解析：`User/Modules/remote/remote_control.c / .h`（`RC_SW_UP/MID/DOWN = 1/3/2` 已有；结构字段 `rc.rocker_l_/l1/r_/r1`、`rc.switch_left/right`）
- 串口服务（已有回调中枢）：`User/Bsp/usart/bsp_usart.c / .h`
- USART3 外设配置：`Src/usart.c`（~77-85）；`Src/stm32f4xx_it.c:293`（`HAL_UART_IRQHandler(&huart3)`）
- remote 注册调用点：`User/App/robot_cmd/robot_cmd.c:165`

---

## A-C 板间 CAN 通讯研究（2026-09-06，A板会话交叉记录）

> 权威记录在 `D:\RM\平步云台（27教具版）\...\0standard\_ET08A_SBUS_进度交接.md` §9（A-C 板间通讯研究，含 A板 侧 CAN 收发表 + C板 侧核实 + 综合结论）。本段只是给底盘侧会话的指针。

已核实（读码）：
- C板 现 CAN 只接电机：CAN1=DM 关节(腿) 0x001-0x004，CAN2=DJI 3508(轮) 0x201/0x202 反馈 + 0x1FF/0x200/0x2FF 分组发。**无任何对 A板 的收发**。
- **0x160/161/162（A板 下发的 DBUS18 遥控）C板 收端不存在**：IDLIST 过滤器没配 + 无回调 + cmd_send.c 消费任务整段注释（`cmd_send.c:8-48`、`robot.c:27,37`）。A板 每帧都在发、C板 硬件层丢弃。
- 遥控现只从 USART3(8E1/18B DBUS) 进；`rc_state` 2=裁判键鼠兜底(USART1)；rc_ctrl 无互斥。
- ⚠️ robot_def.h:12 生效宏仍是 `GIMBAL_BOARD`，未切 `CHASSIS_BOARD`。
- ⚠️ 架构未定：本文件主张 C板 自读 USART3(SBUS)、整车整合到 C板；A板 0standard 文档主张 A板 读 SBUS 经 CAN 0x160-162 转发。接收机只能接一块板 → 待拍板后再动 A板/C板 待改清单。
- ✅ **2026-09-06 已拍板：遥控主源 = A板 读 SBUS → CAN 0x160-162 转发 DBUS18 给 C板**。含义：整车遥控从 A板 走 CAN 进 C板；本文件"C板 自读 USART3(SBUS)"**降级为台架调试备用**（台架单测底盘时 USART3 直插接收机仍可用，改 SBUS 的工作量保留）。整车路径的 C板 侧工作 = 新建 0x160-162 收端：`CANRegister(rx_id=0x160/161/162)` 建 IDLIST filter → 三帧拼 18B → 喂现有 `sbus_to_rc()`（格式同构，零协议改动）→ `DaemonReload` 保活；注意与 USART3 双写 `rc_ctrl` 需分源（rc_state）或加互斥。详情见 A板 会话文档 §9。

---

## 哨兵 remote.c ↔ 底盘 sub/pub 评审（2026-09-06，A板会话交叉记录）

> 用户收到**哨兵 W.BUS(25B)** remote_control.c，问能否"作为底盘 sub/pub 机制的代码框架"。已读码核实后结论：

1. **真 pub/sub 是 `message_center`**（`User/Modules/message_center/`，链表话题 + 每订阅者独立队列 + `SubGetMessage` 拉取式）。已在用：`robot_cmd.c:176-177` 注册 `chassis_cmd`(pub) / `chassis_feed`(sub)，`chassis_control.c` 侧 `Publisher_t *chassis_pub` / `Subscriber_t *chassis_sub` 消费。话题在 **ONE_BOARD**（板内）模式下连接 robot_cmd↔chassis_control。
2. **遥控模块没上话题**：`robot_cmd.c:165` `rc_data = RemoteControlInit(&huart3)` 是**直连返回指针 + 轮询**（模块含 TEMP/LAST 双缓存、回调喂狗、`RemoteControlIsOnline()`），不是 pub/sub 生产者。
3. 哨兵 remote.c 与底盘现有 `User/Modules/remote/remote_control.c` **同模板**，仅解包 25B↔18B 之差，新东西只有 `stick_mode()` 两/三档防抖（滞回 950/1050 + 中档驻留 6 帧）——**要抄只抄这段逻辑**。
4. **若想让遥控进 sub/pub**：remote 模块保留当"数据源/生产者壳"（源 = USART3 台架直读 或 CAN 0x160-162 收端；源事件→组装 rc_ctrl→`DaemonReload`→喂狗），解码后把帧 **发布** 到 topic（如 `PubRegister("remote_control")`），robot_cmd/chassis_control/UI 改 `SubRegister` 订阅。模块本身**不是** pub/sub 框架，只是一个可当发布端的单例数据源模板，需自行包一层 topic 发布。
5. **哨兵专有、C板 勿搬**：`RCLostCallback` 回退 `gimbal_referee` 的裁判遥控兜底；`referee_task` 依赖。

### CANComm 模块审阅（2026-09-07）

> 用户提供 HNU 新框架 `can_comm`（作者 neozng1@hnu.edu.cn）说明文档，核对模块源码后记录。

- **定位**：新框架自带的**通用板间 struct 通信**模块（`User/Modules/can_comm/can_comm.c/.h`）。每 `CANCommInstance` = 底层一个 `CANInstance`（bsp_can）+ **一对 tx_id/rx_id** + payload（`CAN_COMM_MAX_BUFFSIZE=60`）+ **帧协议 `'s' + datalen + data + crc8 + 'e'`** + `DaemonInstance`（在线检测）。收发长度用 `sizeof()`，被传 struct **必须 `#pragma pack(1)`**。`CANCommGet()` 返回解包后数据指针，读完 `update_flag` 清零（作者 TODO：让其直接返回"是否更新"）。
- **现状（C板 工程内）**：`cmd_send.c:8-48` 有完整用法示例（hcan2，tx_id=0x22 / rx_id=0x33，发 `chassis_data_rev`、收 `Chassis_Upload_Data_s`，再经 message_center `chassis_cmd`/`chassis_feed` 话题中转）——**整段在 `/* */` 里，未初始化、未被调用**。
- **与 A板 遥控转发的关系（结论）**：A板（27教具 SPL）走的是 **raw DBUS18 拆成 0x160/0x161/0x162 三帧、无 CANComm 帧协议** → **CANComm 收不了**（帧头/长度/crc8 校验全不匹配，且单实例只挂一个 rx_id，接不了 3 个 ID）。要硬用 CANComm 就得让 A板 也按 `'s'...'e'+crc8` 协议重发（老 SPL 树要手移植组帧，不划算）。
- **因此 C板 收遥控维持原方案**：**3× `CANRegister(rx_id=0x160/161/162)` + 各自回调**（bsp_can.h 已确认 API：`CANRegister(CAN_Init_Config_s*)`，结构体含 `rx_id` + `can_module_callback`）→ 收齐三帧拼 18B → 喂现有 `sbus_to_rc()`（格式同构）→ `DaemonReload` 保活。**CANComm 留给将来"板间传结构化命令/反馈"**（即 cmd_send.c 注释里那套，届时再启用）。

---

## 遥控链路定稿（2026-09-12）：A板 读 SBUS → 0x160/161/162 → C板

> **本节取代上面的「待改清单」第 1~5 条**（那 5 条是早期"C板 自读 USART3 25B SBUS"方案，2026-09-06 拍板后已降级为台架备用，**别再照它改**）。§A-C 板间 CAN 通讯研究的结论仍然有效，本节是它的落地版。
>
> 状态：**A板 侧已完成并实机验证**（`0standard`，27教具版）；**C板 侧已于 2026-09-12 改完**，两侧**全量重建 0 Error**（见 §7）。
> ⚠️ §2/§3 的代码块写的是"原本该怎么改"，**实际改动以 §7 为准** —— §7 还包含一条本节没写到的修正（右轮 `tx_id` 与电调 ID 不一致，§7.2）。
> 数据流：ET08A 接收机（真 SBUS 25B/8E2@100000）→ A板 USART1 → A板 解成 rc_ctrl → 按 18B DBUS 布局拆 3 帧发 CAN2 → C板 hcan2 收 → 拼回 18B → 喂 C板 现有 `sbus_to_rc()`。

### 1. 线协议（冻结，两边照这个写）

| CAN StdId | 内容 | 说明 |
|---|---|---|
| `0x160` | 18B 的 byte[0..7] | CH0~3 的 11bit 流 + byte5 高半字节（两个 2bit 开关）+ 鼠标/键盘低字节 |
| `0x161` | 18B 的 byte[8..15] | 鼠标剩余 + 键盘 |
| `0x162` | 尾部 8 字节 | 见下表 |

`0x162` 的 8 个数据字节：

| data | 含义 | A板 来源 | C板 用法 |
|---|---|---|---|
| `[0]` | 18B[16] | `buf18[16]` = CH5 拨轮低字节 | → `rc_canbuf[16]` |
| `[1]` | 18B[17] | `buf18[17]` = CH5 拨轮高字节 | → `rc_canbuf[17]` |
| `[2]` | 来源标志 | `rc_from_sbus` (1/0) | 备用（C板 暂不用） |
| `[3]` | **s[3] 跳跃（SA）** | `rc->rc.s[3] & 3` | → `rc.switch_jump` |
| `[4]` | **s[2] 发弹（SD）** | `rc->rc.s[2] & 3` | → `rc.switch_fire` |
| `[5..7]` | 保留 | 恒 0 | 恒 0 |

- 取值就是 DJI 那套 **1=UP / 3=MID / 2=DOWN**（两档只有 1/2）。
- **为什么要扩**：18B 里能放开关的只有 byte5 高半字节的**两个 2bit**，已被 s[0]（模式，左中三档 SB=CH7）、s[1]（小陀螺/自瞄/正常，右中三档 SC=CH6）占满；A板 现在多解了 s[2]（SD=CH5 发弹）、s[3]（SA=CH8 跳跃），只能另找位置——A板 已先把 s[3] 塞进 `tail8[3]`，**本轮把 s[2] 补到 `tail8[4]`**，不用加第四帧（DLC 本来就是 8）。

> **老实说一句：本轮 C板 真正需要的扩展字段只有 `[3]`（跳跃）**——发弹是云台/发射机构的事，**A板 `shoot.c:344-354` 已经自己在用 `rc.s[2]` 驱动发射**，不经过 C板；而 C板 现在的应用只有 `RobotCMDTask + ChassisTask`，没有发射机构。把 `[4]` 一起占掉，图的是"**0x162 的 8 字节一次用满，以后加字段不用再改一次线协议**"，不是因为有消费方。若你想省事，只加 `tail8[3]`、C板 只加 `switch_jump` 也完全能跑（那就把本节里 s[2]/`switch_fire` 相关的行整段划掉）。

### 2. A板 侧改动（1 行）

文件：`0standard/user/APP/remote_control/remote_control.c` → `RC_ctrl_push_to_chassis_can()`（286~299 行）。

现状（改前 292~298 行）是 `tail8[3] = s[3]`、`tail8[4..7] = 0`，**只需把 `tail8[4]` 填上**——**2026-09-12 已改**（现落在 `:297-298`）：

```c
    tail8[3] = (uint8_t)((uint8_t)rc->rc.s[3] & 3u);   // 跳跃 SA
    tail8[4] = (uint8_t)((uint8_t)rc->rc.s[2] & 3u);   // 发弹 SD  ← 新增
    tail8[5] = 0;
    tail8[6] = 0;
    tail8[7] = 0;
    Send_remote(tail8, 2);
```

（`Send_remote` 见 `CAN_receive.c:400` 段：`StdId = 0x160 + idx`、STD、DLC=8、`Data[0..7] = rc_ctrl[0..7]`、走 CAN2。）

### 3. C板 侧改动（4 处）

> **2026-09-12：下面这 4 处已全部落地**，实际代码与本节代码块一致（只多了一句 `rx_len` 校验和一个 `RC_CAN_TAIL_ID` 宏），另外多改了 `chassis_control.c` 一处——见 §7。行号是**改前**的。

#### 3.1 `User/Modules/remote/remote_control.h`

C板 的 `RC_ctrl_t.rc` **没有 `char s[4]`**（是 `rocker_*/dial` + `switch_left/right` 那套），所以只能**加字段**：

```c
        uint8_t switch_left;  // 左侧开关
        uint8_t switch_right; // 右侧开关
        uint8_t switch_fire;  // 新增：SD 发弹（两档, 1=UP 2=DOWN）
        uint8_t switch_jump;  // 新增：SA 跳跃（两档, 1=UP 2=DOWN）
```

- 129 行签名改成 `RC_ctrl_t *RemoteControlInit(void);`
- **`sizeof(RC_ctrl_t)` 会 +2**（该结构体没有 pack、按 int16 对齐，量级 ~76→78 —— 要点只是"变了"，别硬编码这个数字）。`RCLostCallback` 里 `uint8_t temp[sizeof(RC_ctrl_t)]` 自动跟随 ✓；但改前 grep 一遍有没有**按字节偏移**访问 `RC_ctrl_t` 的地方（已查：没有）。

#### 3.2 `User/Modules/remote/remote_control.c`

**删**：`#include "bsp_usart.h"`（改 `bsp_can.h`）、`USARTInstance *rc_usart_instance;`（18 行，前面的"因为遥控器是单例"注释一起删）、`RemoteControlRxCallback()`（96~101 行）、`RCLostCallback` 里的 `USARTServiceInit(rc_usart_instance);`（119 行）、`RemoteControlInit` 里的 USARTRegister 段（127~131 行）。

**加**（3 个 CAN 实例共用一个回调，靠 `ins->rx_id` 分派）：

```c
static uint8_t rc_canbuf[REMOTE_CONTROL_FRAME_SIZE]; // 拼回来的 18B
static uint8_t rc_can_seen = 0;                      // 0=空, 1=已收0x160, 3=已收0x160+0x161

/* 注意:本回调在 CAN 中断里执行(bsp_can.c 的 FIFOx 回调直接调) */
static void RCCANRxCallback(CANInstance *ins)
{
    switch (ins->rx_id)
    {
    case 0x160u:
        memcpy(&rc_canbuf[0], ins->rx_buff, 8);
        rc_can_seen = 1u;
        break;

    case 0x161u:
        if (rc_can_seen != 1u) break;   // 乱序/丢帧,本帧丢弃
        memcpy(&rc_canbuf[8], ins->rx_buff, 8);
        rc_can_seen = 3u;
        break;

    case 0x162u:
        if (rc_can_seen != 3u) break;
        rc_canbuf[16] = ins->rx_buff[0];
        rc_canbuf[17] = ins->rx_buff[1];
        /* 扩展字段必须在 sbus_to_rc 之前写进 TEMP:
           该函数末尾有 TEMP→LAST 的整结构 memcpy(89 行),这样 LAST 也能拿到,
           switch_jump 才做得了上升沿判断 */
        rc_ctrl[TEMP].rc.switch_jump = (uint8_t)(ins->rx_buff[3] & 3u);
        rc_ctrl[TEMP].rc.switch_fire = (uint8_t)(ins->rx_buff[4] & 3u);
        sbus_to_rc(rc_canbuf);          // 现有解析,零改动
        rc_ctrl->rc_state = 1;          // 同原 RemoteControlRxCallback
        DaemonReload(rc_daemon_instance);
        rc_can_seen = 0u;
        break;

    default:
        break;
    }
}
```

- `ins->rx_buff` 固定按 8 字节用（A板 `Send_remote` 的 DLC 就是 8）；想稳一点可以先看 `ins->rx_len == 8`（bsp_can 已填 `rx_len`，见 `bsp_can.c:155`），DLC 短了就直接丢帧。
- `sbus_to_rc` 是 `static`，同文件内直接调，不用改它的可见性。

`RemoteControlInit` 里把 USARTRegister 换成三个 CANRegister：

```c
RC_ctrl_t *RemoteControlInit(void)
{
    CAN_Init_Config_s can_conf = {
        .can_handle = &hcan2,
        .tx_id = 0,          // 只收不发;留 0 是必须的,理由见下
        .rx_id = 0x160,
        .can_module_callback = RCCANRxCallback,
        .id = NULL,
    };
    CANRegister(&can_conf);   // 0x160
    can_conf.rx_id = 0x161;
    CANRegister(&can_conf);
    can_conf.rx_id = 0x162;
    CANRegister(&can_conf);

    // daemon 注册保持原样(reload_count = 10 → 100ms 无帧判离线)
    ...
    rc_referee_recv = GimbalRefereeTaskInit(&huart1);
    rc_init_flag = 1;
    return rc_ctrl;
}
```

裁判兜底分支（`RCLostCallback` 109~115 行）也要补两行，否则掉线回到裁判遥控时这两个字段会留着**上一帧的旧值**：

```c
        rc_ctrl[TEMP].rc.switch_jump = RC_SW_DOWN;
        rc_ctrl[TEMP].rc.switch_fire = RC_SW_DOWN;
        sbus_to_rc((const uint8_t*)temp);
```

（离线分支 118 行的 `memset(rc_ctrl, 0, sizeof(rc_ctrl))` 已覆盖，不用管。`sbus_to_rc` 本身不写这两个字段。）

> ⚠️ **`tx_id = 0` 不是随便填的**：`bsp_can.c:45` 用 **`tx_id` 的奇偶**决定进 FIFO0 还是 FIFO1（`(tx_id & 1) ? FIFO0 : FIFO1`）。三个都填 0（偶数）→ 同一 FIFO1 → **严格保序**，A板 连发的 0x160/161/162 到 C板 顺序不变。**若图省事填 `tx_id = 0x160/0x161/0x162`，0x161 会单独进 FIFO0**，与另两帧跨 FIFO 就可能乱序，上面的 `rc_can_seen` 状态机就会丢帧。

#### 3.3 `User/App/robot_cmd/robot_cmd.c`

- 165 行 → `rc_data = RemoteControlInit();`
- 右拨杆段（58~70 行）**之后**加跳跃（**先别开，理由见 §3.4**）：

```c
/*==============================跳跃请求==============================*/
    /* SA(switch_jump) 打上 + 当前是平衡模式 → 请求跳跃 */
    if (switch_is_up(rc_data[TEMP].rc.switch_jump)
        && chassis_cmd_send.chassis_mode == CHASSIS_BALANCE)
    {
        chassis_cmd_send.chassis_mode = CHASSIS_JUMP;
    }
```

顺序不能颠倒：这段必须在拨杆定档（59~70 行）**之后**，否则跳跃会被拨杆覆盖。另外 `ControlModeChangeTransit()`（94~128 行）**没有 `CHASSIS_JUMP` 分支**，切进跳跃时 `x_set / leg_len_set` 不会被重置——这也是"先别开"的理由之一。

#### 3.4 前置警告：`CHASSIS_JUMP` 现在只是空壳

**别急着用它跳**，已逐处核对：

- `chassis_control.c:624-626`：`CalVirtualForce()` 的 `case CHASSIS_JUMP:` **里面是空的**（只有一句注释"分步走…第二步以后直接指定虚拟腿力的大小"）→ `Virtual_Force[0/1]` 会**保留上一个模式算出来的旧值**；
- 其余三处 `CHASSIS_JUMP`（552 车轮 vx_pid、653 合力矩、738 VMC）都跟 `CHASSIS_BALANCE` 走**同一条支路**；
- 合起来：现在置 `CHASSIS_JUMP` ≈ 用**上一拍的腿力** + 平衡环力矩输出 → 可能突然蹬腿。

⇒ **本轮把 §3.3 那段跳跃赋值先注释掉**（遥控字段照通、`switch_jump` 照解，便于先用日志/调试器确认 SA 到得了底盘），等跳跃状态机迁移完再打开。

#### 3.5 不用动的

- `Src/usart.c` / `.ioc` 的 huart3：**保持原样**（8E1/100000，台架单测还用得上）。本节方案不再注册 USART，huart3 空着不影响任何东西。
- 波特率已核对：A板 `main.c:116` CAN2 = SJW1tq / BS2 2tq / BS1 3tq / brp 7 → 42MHz/(7×6) = **1Mbps**；C板 `Src/can.c:74-78` hcan2 = brp 3 / BS1 10tq / BS2 3tq → 42MHz/(3×14) = **1Mbps**。一致（前提：两板 APB1 都是 42MHz）。

### 4. 自查（本节写完后回读代码逐条核对，2026-09-12）

**已对着代码确认：**

1. A板 `tail8[3] = s[3]`、`tail8[4..7] = 0` —— `remote_control.c:288-299` 实际现状，§2 只需补 `tail8[4]` ✅
2. A板 半字节位序：`out18[5] = ((s[1]&3)<<2 | (s[0]&3)) << 4 | ...`（`:257-260`）；C板 `switch_right = (b[5]>>4)&3`、`switch_left = ((b[5]>>4)&0xC)>>2`（`:47-48`）→ **s[0]→switch_right、s[1]→switch_left**，两边对得上 ✅
3. `sbus_to_rc()` **不 memset TEMP**、也不碰 `switch_*` 字段（只写 rocker/dial/switch_left/right/mouse/key，`:37-90`）→ 在它之前写 `switch_jump/fire` 不会被抹掉；末尾 `:89` 的 `memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t))` 会带上新字段 ✅
4. `RectifyRCjoystick()`（`:25-30`）只处理 `rc` 头 5 个 int16，不影响新字段 ✅
5. `rc_ctrl[TEMP]` ≡ `rc_ctrl[0]`，所以 `rc_ctrl->rc_state` 就是 TEMP 的（`:13` + `:24-25` 的 LAST/TEMP 宏）✅
6. bsp_can 回调签名/字段：`can_module_callback(CANInstance *)`，实例里有 `rx_id` / `rx_buff[8]` / `rx_len`（`bsp_can.h:20-24,34`）✅
7. bsp_can 派发：ISR 按 `(can_handle, rxconf.StdId == instance->rx_id)` 找实例 → 三个 rx_id 各触发各的回调（`bsp_can.c:149-160`）✅
8. 过滤器容量：can2 从 **bank 14** 起、16 位 IDLIST 每实例占 1 个 bank（`bsp_can.c:41-48`，`bsp_can.h:8` `MX_CAN_FILTER_CNT = 28`）；can2 现有 2 个 3508 → 我们占 16/17/18 ✅
9. 实例数：`CAN_MX_REGISTER_CNT = 16`，当前 6 个（hcan1 4×DM 关节 `chassis_control.c:132-160`，rx_id 1~4 + hcan2 2×3508 `:113-129`）→ 9 个，未超 ✅。全工程 `CANRegister` 调用点只有 `dm_motor.c:91` / `dji_motor.c:185` / `rmd_motor.c:97` / `can_comm.c:93` 四个，其中 `can_comm.c:93` 在未启用的 cmd_send 里、`rmd_motor` 本车没用到 → 实际就是上面 6 个
10. 重复注册判重按 **(rx_id, can_handle)**（`bsp_can.c:69-76`）→ 0x160/161/162 三者互不冲突 ✅
11. `RemoteControlInit` 全工程**只有 `robot_cmd.c:165` 一个调用点** → 改签名只动 1 处 ✅
12. `RemoteControlIsOnline()` **全工程没人调用**（只有定义 `remote_control.c:145-150`）→ 它的签名/行为改动不影响别处 ✅
13. `CHASSIS_JUMP` 枚举在 `robot_def.h:90`；`chassis_control.c` 的 4 处 case 位置与内容同 §3.4（`:624-626` 确实为空）✅
14. bsp_can 的 FIFO 回调在找到匹配实例、调完回调后**立即 `return`**（`bsp_can.c:159`），不会串台 ✅
15. A板 侧扩展字段确实有源可查：A板 `remote_control.c:711-712` 的 `SBUS_TO_RC` 解出 s[2]/s[3]（两档，`<700→DOWN 否则→UP`）；同文件 `RC_data_is_error` 的 error 分支（`:552-555`）把 s[2]/s[3] 都归 `RC_SW_DOWN` → **A板 掉线时下发的就是 DOWN，不会误触发跳跃/发弹** ✅
16. §6 那条失联隐患属实：A板 `remote_control.c:663` **只校验 `sbus_buf[0] != 0x0F`**，确实没查 byte23 标志/byte24 结束字节；`:604`/`:624` 也确实是无条件刷新 `rc_sbus_last_good_tick` ✅

**没能核实 / 需要你上机确认：**

- ❗ **A板 CAN2 与 C板 hcan2 是同一根总线**：这是从"A板 `CAN_receive.c:328` 的 CAN2 收件里含 3508 反馈 + 超电 0x211"**推断**出来的（说明 A板 CAN2 接的就是底盘电机那条总线），**没有看接线图**。上车前用 CAN 分析仪/示波器确认一次。
- ~~❗ **A板 `tail8[4]` 现在是 0** 是按本次读到的 `:295` 判断的；你动手改之前再看一眼那 8 行的实际状态。~~ → **2026-09-12 已核实并改完**，实际落在 `:297-298`（见 §7.3）。
- ❗ 丢帧安全性（设计推演，非实测）：A板 三帧在同一个任务里连续 `CAN_Transmit`，同 FIFO 保序；若中途丢帧，0x161/0x162 被丢弃、下一轮 0x160 重置状态机 → 最坏丢一帧遥控。**70Hz 下无感**，但台架时可以故意拔线验证 `rc_state` 是否按预期掉。
- 【更新】`robot_def.h:12` 生效宏**仍是 `GIMBAL_BOARD`**。本次查证：该宏在本工程**只被 `robot_def.h:163` 的 `#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD)` 用到，两个宏走同一分支** → **对编译结果无影响**，只是语义写错、易误导。可以顺手把 `GIMBAL_BOARD` 注释掉、改 `CHASSIS_BOARD`（改完全编译一次）。上面的 ⚠️ 由此降级为"待清理"。

### 5. 建议上手顺序

1. 先只做 §3.1 + §3.2 + §3.3 的**签名那行** → 编译过、上电看 `rc_data[TEMP].rc.rc_state` 是否 = 1（`robot_cmd.c:166` 已有 `RC_DEBUG = *rc_data;`，可直接在线看）。
2. 再通 s[2]/s[3]：拨 SA/SD，看 `switch_jump` / `switch_fire` 是否在 1/2 之间跳。
3. 台架确认轴位没被这轮改动带偏（A板 实测轴位见其本机文档，C板 侧 `rocker_l_/r_` 等只由 `sbus_to_rc` 的字节位决定，本次没动解析本体）。
4. 最后——**等跳跃控制律迁移完**——才打开 §3.3 的 `CHASSIS_JUMP` 赋值。

### 6. 顺带记录（本轮读码发现，与本任务相邻）

- A板 `RC_data_is_error()`（`:504-564`）现在会检查 `s[2]/s[3] != 0`，A板 `uart8.c:231-232` 已给图传路径补了默认 `RC_SW_DOWN` ✅。C板 侧**没有**对应的 `RC_data_is_error`，不受影响。
- ❗ **A板 SBUS 失联仍有个隐患**：`USART1_IRQHandler` 只要收到 25B 帧就**无条件**刷新 `rc_sbus_last_good_tick`，而 `SBUS_TO_RC` **只校验 `buf[0]==0x0F`**（没查 byte23 的 failsafe 标志、也没查 byte24 结束字节）→ 接收机若在**发射机关机后继续吐 failsafe 帧**，`rc_sbus_stream_dead` 这条路就永远不会触发。
  台架时把发射机关掉、盯 `et08_dbg_rx25` 还涨不涨，就知道要不要补这个判断。

---

## 7. 落地记录（2026-09-12 本轮实改 + 编译结果）

> 本节记录**实际动手改了什么**，是 §2/§3 的执行版。两侧都改完，**全量重建 0 Error**。

### 7.1 编译结果

| 工程 | 目标 / 产物 | 结果 |
|---|---|---|
| C板 `balance_chassis/MDK-ARM/Infantry.uvprojx` | Infantry → `.\Hero\Hero.axf` | **0 Error, 3 Warning** |
| A板 `0standard/Project/standard.uvprojx` | → `Objects\infantry.axf` | **0 Error, 14 Warning** |

- 编译器：ARMCC **V5.06 update 7 (build 960)**，Keil 装在本机 `D:\RM\KEIL\UV4\UV4.exe`。
- 两次都是 **`-r` 全量重建**（不是增量），日志里确认有 `compiling remote_control.c...`（两侧）、`compiling robot_cmd.c...`、`compiling chassis_control.c...`。
- **所有警告都是改动前就有的**，逐条对过：
  - C板 3 条：`chassis_control.c(86)` `ang_avg`、`(93)` `Actual_Force`、`(93)` `Actual_Torque` —— "设了不用"的旧变量，与本次改动无关（本次改的是同文件 `:127-134`）。
  - A板 14 条：`uart7.c(129)` 文件末尾缺换行、`INS_task.c` 9 条未引用变量/函数、`gimbal_task.c` 3 条（含 `:930` 除零）、`shoot.c(309)` `last_dial`。**`remote_control.c` 一条警告都没有。**
- 复现命令（PowerShell；**路径带空格必须自己加引号，否则 UV4 返回 15**，因为 `Start-Process -ArgumentList` 是拿空格拼的）：

```powershell
Start-Process 'D:\RM\KEIL\UV4\UV4.exe' -Wait -PassThru `
  -ArgumentList "-r `"<工程.uvprojx 全路径>`" -j0 -o `"<日志路径>`""
```

  UV4 退出码：0=无错无警，1=有警告，2=有错误。

### 7.2 ⚠️ 更正（2026-09-13）：右轮电调 ID 是 4，`tx_id` 必须跟着写 4

> **本节昨天写错了一版，以下是更正后的内容。**
> 昨天我把右轮 `tx_id` 从 2 改成 3，理由是"A板 的 0x202 功率帧会撞右轮反馈"。**那个前提不成立** —— 用户确认本车电调的硬件 ID 是 **左 1 / 右 4**，右轮根本不在 0x202 上。

**事实：**

- **本车 3508 电调 ID：左轮 = 1，右轮 = 4**（用户 2026-09-13 确认，C620 上的硬件 ID）。
- `tx_id` **必须等于电调 ID** —— 它同时决定两件事（`dji_motor.c:44-64` 的 `MotorSenderGrouping`：`motor_id = tx_id - 1`、`rx_id = 0x200 + motor_id + 1`、`motor_send_num = motor_id`）：

| `tx_id` | `rx_id`（收谁的反馈） | 力矩写 0x200 帧的第几组字节 |
|---|---|---|
| 1（左轮 ✅） | 0x201 | 第 1 组 `[0..1]` |
| **4（右轮 ✅ 正确值）** | **0x204** | 第 4 组 `[6..7]` |
| ~~2~~（代码里原来的值） | 0x202 | 第 2 组 |
| ~~3~~（昨天我改的值） | 0x203 | 第 3 组 |

- 所以 **原来的 2 和昨天改的 3 都是错的，正确值是 4**。写 2 或 3 的后果一样：右轮**收不到自己的反馈**、力矩也写进**电调不读的那一组** → **右轮完全不动**。
- 现已改成 `tx_id = 4`（`chassis_control.c:127-134`，注释一并重写）。

**关于 0x202 撞车：那条结论作废。** 右轮在 0x204，A板 那帧 500Hz 的 0x202 跟它无关 —— A板 在 CAN2 上的发送 ID 全集是 `0x1FF / 0x200 / 0x202 / 0x700 / 0x180 / 0x160+idx`（`CAN_receive.c:101,127,145,177,196,215,234,383,403`），**没有 0x201/0x203/0x204** → 0x204 干净。**用户那句"202不影响"是对的。**

- ⚠️ **不要去动电调**：电调保持 1 / 4，代码跟着电调走（我昨天写的"把电调改成 3"作废）。
- 顺带核过，不会触发 `dji_motor.c:70-79` 那个 `while(1)` 判重死循环：它比的是 **(can_handle, rx_id)** 对，hcan1 的 4 个关节虽然 tx_id 也是 1~4（0x201~0x204），但**不是同一根 CAN**，不算冲突。
- **一条值得回头查的线索**：如果右轮以前就有"不转 / 没反馈 / 数据跳"的表现，很可能就是这个 `tx_id` 与电调不一致造成的，而不是别的原因。

### 7.3 其余实际改动（与 §2/§3 一致）

| 文件 | 改动 |
|---|---|
| A板 `user/APP/remote_control/remote_control.c` | `:297-298` `tail8[3] = s[3]`、`tail8[4] = s[2]`（`:299-301` 保留 0） |
| C板 `User/Modules/remote/remote_control.h` | `rc` 里加 `switch_fire`/`switch_jump`（`:105-108`）；`:135` 签名改 `RC_ctrl_t *RemoteControlInit(void);` |
| C板 `User/Modules/remote/remote_control.c` | 见 §3.2：`RCCANRxCallback`（`:104`）+ 三个 `CANRegister`（`:174-187`）；额外加了 `#define RC_CAN_TAIL_ID 0x162u` 和 `if (ins->rx_len != 8u) return;`；`RCLostCallback` 裁判分支补两行（`:155-156`）；删掉 `USARTServiceInit` |
| C板 `User/App/robot_cmd/robot_cmd.c` | `:177` `rc_data = RemoteControlInit();`；`:71-82` 跳跃块**整段注释掉**（理由见 §3.4） |
| C板 `User/App/chassis_control/chassis_control.c` | `:132` 右轮 `tx_id = 4`（与电调 ID 对齐，见 §7.2） |

### 7.4 还没做 / 要你上机确认的

1. ~~右轮 C620 电调 ID → 3~~ → **改反了**：电调保持 **1 / 4** 不动，是代码的 `tx_id` 要写 **4**（已改，见 §7.2）。
2. **A板 CAN2 与 C板 hcan2 是不是同一根总线** —— 仍是推断（§4 的 ❗），没看接线图。
3. C板 `robot_def.h:12` 生效宏仍写着 `GIMBAL_BOARD` —— 只影响语义不影响编译（§4 已证），可顺手清理。
4. `CHASSIS_JUMP` 仍是空壳，§3.3 那段跳跃赋值**保持注释**，等控制律迁移完再开。
5. SBUS 失联隐患（§6）：发射机关机后接收机若继续吐 failsafe 帧，`rc_sbus_stream_dead` 永不触发 —— 台架盯 `et08_dbg_rx25` 验证。
6. **编译通过 ≠ 上电正确**：遥控链路目前只在编译层面验证过，`rc_state` / 两个两档开关 / 轴位仍需按 §5 的顺序上车确认。
