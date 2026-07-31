# Claude Debug 紀錄 — Gazebo 模擬底盤與路徑追蹤(2026-07-31)

目標是「讓車子在 Gazebo 裡順利跟隨路徑」。結論是問題**幾乎全部不在導航演算法**,
而在模擬底盤根本沒有正確執行 `cmd_vel`。本文記錄根因、修法,以及過程中好幾個
把我引到錯誤方向的陷阱 —— 那些陷阱比結論本身更值得記住。

---

## 根因:`gazebo_ros2_control` 開迴路下速度指令

### 症狀
- 直線指令跑出 115% 的距離,並且穩定偏航 +7°/1.8 m
- 原地旋轉只達成指令的 23%
- 導航在 maze 裡完全失敗,AMCL 誤差累積到 5~12 公尺

### 診斷過程(方法可重用)
1. **先取得可信的 ground truth**。`gz model -m mir_robot -i` 回報的位姿
   **不是** `base_footprint`,偏了約 2 m / 18°;單張雷射掃描比對 `maze.pgm`
   在矩形空間裡有多重解會亂跳。兩者都不能用。正解是在 world 檔加
   `libgazebo_ros_state.so`,讀 `/gazebo/link_states` 的 `base_footprint`:
   ```xml
   <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
     <ros><namespace>/gazebo</namespace></ros>
     <update_rate>50.0</update_rate>
   </plugin>
   ```
2. **量有效滾動半徑**,分辨「打滑」和「指令沒被執行」:
   ```
   有效半徑 = 真實車速 / 實際輪速
   ```
   量到 0.0625 m,和設定值完全相同 → 沒有打滑,物理是自洽的,
   問題在輪子沒照指令轉。
3. **繞過 `diff_cont` 直接驅動輪子**(用 `velocity_controllers/JointGroupVelocityController`,
   已加在 `diffdrive_controller.yaml` 供日後除錯):數字一模一樣 → `diff_cont` 無辜。
4. **下 0 rad/s 看會不會停**:輪子維持 +0.37 rad/s、車子以固定 22.8 mm/s
   前進 25 秒完全不衰減 → 不是動量,是指令沒被執行。
5. **轉 90°/180° 再量爬行方向**:世界方向跟著車頭轉、車體相對方向不變
   → 機器人自己推自己,不是地面或重力問題。

### 根因
誤差是**固定加法偏移 +0.37 rad/s**(低速佔比大所以看起來像比例誤差)。
`gazebo_ros2_control` 在沒有 PID 設定時走開迴路 `SetVelocity()`,而真實 MiR
的馬達驅動器是**閉迴路速度控制**。模擬缺的就是這一層。

### 修正
`mir.urdf.xacro` 的 ros2_control 每個驅動輪加上:
```xml
<param name="vel_kp">100.0</param>
<param name="vel_ki">20.0</param>
<param name="vel_kd">0.0</param>
<param name="vel_max_integral_error">10.0</param>
```

| 項目 | 修正前 | 修正後 | 應為 |
|------|--------|--------|------|
| 直線距離 | 115% | 102% | 100% |
| 直線偏航(1.8 m) | +7.0° | −0.6~−2.6° | 0° |
| 原地旋轉 | 23% | 104% | 100% |
| 輪子指令 0 時的爬行 | 22.8 mm/s | 0.6 mm/s | 0 |
| spawn 位置誤差 | 滑走數公尺 | 1.5 cm | 0 |

`vel_kp=300` 開始震盪,100 是實用上限。

---

## 陷阱(這些比結論更重要)

### 1. stale `robot_state_publisher` 讓所有 URDF 改動失效
自製的 teardown 腳本把模式寫成 `'robot_state'publisher`,串接後少了底線,
**從來沒殺掉過 `robot_state_publisher`**。它持續發布舊的 `/robot_description`,
而 `spawn_entity` 就是從那個 topic 取模型 —— 於是連續好幾輪的摩擦係數、
接觸剛度、懸吊實驗,量的全是同一台沒變過的機器人,結論全部作廢。

**教訓:改完 URDF 一定要驗證它真的到達 Gazebo。** 用
`scratchpad/check_urdf.py` 訂閱 `/robot_description` 直接數字串出現次數;
`ros2 topic echo /robot_description | grep` 會因為字串太大被截斷而永遠回 0。

### 2. XML 註解放在 `<gazebo>` 或 `<ros2_control>` 區塊裡會讓 gzserver segfault
**確定性重現**:只改數值不加註解 → 2/2 正常;同樣的數值加上多行註解 → 3/3 崩潰。
崩潰點是 `gazebo_ros2_control` 或當下最後載入的 plugin 在 Load 時丟例外,
接著 gzserver `exit code -11`。

這害我誤判過至少三次:「`mu=1.5` 會崩潰」「caster 懸吊會崩潰」
「驅動輪 kp/kd 會崩潰」—— 全部都是註解害的,不是參數。
**說明文字要寫在那些區塊外面。**

### 3. `SIGKILL` 被誤判成 segfault
teardown 殺掉的行程 exit code 是 `-9`,真正的崩潰是 `-11`。把 `-9` 當成崩潰
會讓「啟動比較慢」看起來像「模擬器壞了」。判定邏輯要分清楚。

### 4. `pgrep -f <pattern>` 會匹配到呼叫它的 shell
在 Bash 裡直接跑 `pkill -f robot_state_publisher`,該 shell 自己的命令列就含有
這個字串,於是把自己殺掉,指令後半段全部沒執行。把清理邏輯放進腳本檔
(`scratchpad/killall.sh`),模式字串就不會出現在呼叫端。

### 5. 在錯的座標系量循跡誤差
一開始的控制器比較是在 **odom** 座標系量的 —— 那只說明「控制器對自己相信的位置
追得多準」。底盤有偏移時 odom 和現實會分家,必須用 ground truth 量世界座標誤差。

---

## 走過的冤枉路(結論:與底盤問題無關)

用有效數據排除,全部無效:
- 驅動輪摩擦 `mu` 1.0 / 1.5 / 3.0 / 300
- 扭轉摩擦 `coefficient` 300 / 1.0 / 0.0(數字到小數點後一位都相同)
- 接觸剛度 `kp` 1e8 / 1e6 / 1e5(原廠 1e8 最好,調軟反而讓 caster 懸空)
- caster `slip1`/`slip2` 對稱化、caster 與驅動輪的 `fdir1`
- caster 加 prismatic 彈簧懸吊
- `controller_manager` 更新率 100 vs 1000 Hz
- `gazebo_grasp_fix`、手臂/夾爪控制器(關掉反而更糟)

**「六個接觸點沒有懸吊所以靜不定」那個假說也是錯的** —— 它建立在
「同設定跑出三種不同結果」的觀察上,而那個觀察本身是陷阱 1 造成的假象。
測試架構修好後,同設定的結果一致到 1% 以內。

---

## 官方 MiR100 規格對齊

依據 [MiR100 Specifications, 2023-06-28](https://24279054.fs1.hubspotusercontent-na1.net/hubfs/24279054/Resources/MiR/MiR100%20Specifications%202.57.pdf):

| 項目 | 官方 | 原本設定 | 已修正為 |
|------|------|----------|----------|
| 最高前進速度 | 1.5 m/s | `diff_cont` 上限 1.0 | 1.5 |
| 最高後退速度 | 0.3 m/s | 無獨立限制(−1.0) | −0.3 |
| 輪關節速度上限 | 需 24 rad/s | 20(=1.25 m/s) | 24 |
| 車體尺寸 | 890 × 580 mm | footprint 900 × 500 | 890 × 580 |
| 驅動輪直徑 | 125 mm | 0.0625 m 半徑 | 已相符 |
| 定位精度 | ±26 mm / ±3° | goal 容差 80 mm / 2.9° | 偏航已相符 |

實測:指令 1.5 m/s 實際 1.498;指令 1.8 正確截斷在 1.498;倒車同理。

**通行走廊 1000 mm、U 迴轉需 1300 mm** —— maze 的缺口是 1.1 m,
落在兩者之間:直行過得去,但無法在裡面迴轉。

---

## 導航:DWB → MPPI

底盤修好後兩個控制器都能動,但 **DWB 過不了 maze 的 1.1 m 缺口**:
它正確走完整個下半部,停在缺口正南方入口 (6.14, 1.26),跑完 spin 復原行為後
`Goal failed`。兩次結果完全一致。把 `inflation_radius` 從 0.6 降到 0.35 也沒用
—— 縫的寬度不是問題,DWB 應付不了那個急轉入口才是。

| 設定 | maze 結果 |
|------|-----------|
| DWB + inflation 0.6(原廠) | ABORTED,卡在缺口前 |
| DWB + inflation 0.35 | ABORTED,同一個地方 |
| **MPPI + inflation 0.6(原廠)** | **成功,多次重複皆通過** |

所以 `inflation_radius` 維持原廠 0.6,只換控制器。

開闊路徑上的循跡誤差(odom 座標系,各 2 次平均):

| 控制器 | 直線 | 轉角 | 圓形 |
|--------|------|------|------|
| DWB | 1.08 cm | 3.56 cm | 2.23 cm |
| MPPI | 0.24 cm | 1.35 cm | 0.66 cm |

代價是慢約 50%,CPU 負擔明顯較高(每週期 2000 條軌跡)。
`mir_nav_params.yaml` 裡完整保留了 DWB 區塊(註解掉),對調註解即可切回。

### MPPI 調整處
只有 `PathAlignCritic`:
- `cost_weight` 14 → 40(底盤仍有微小左右不對稱,靠這個把線壓住)
- `offset_from_furthest` 20 → 10
- **`use_path_orientations: false`** —— 這個**必須**配合 NavFn。NavFn 的中間點
  orientation 是預設值,對齊它們會和實際行進方向打架,症狀是車子走 0.5 m 後卡死。
  換成會輸出正確朝向的 planner(如 Smac)才可以、也應該設回 `true`。

---

## 其他修掉的 bug

- **`mir_gazebo_launch.py`**:`robot_x`/`robot_y`/`robot_yaw` 宣告了但沒傳給
  `spawn_entity`,車子一律生在原點
- **`mir_gazebo_launch.py`**:`robot_description` 沒包 `ParameterValue(value_type=str)`,
  URDF 裡任何帶冒號的註解都會讓 launch 掛掉
- **`mir_gazebo_launch.py`**:UR5 和 Robotiq 夾爪沒有控制器。手臂垮下來擋住前雷射
  (夾爪落在 z≈0.05,雷射平面 z=0.191),自己的手臂被標成 footprint 內的障礙物,
  Nav2 完全不能動;夾爪手指持續甩動則把整台車震得以 0.15 m/s 漂走。
  現在會自動載入兩個控制器並收合手臂
- **`mir_gazebo_common.py`**:合併後的 `/scan` 用預設 `range_max`(DBL_MAX → inf),
  未命中光束的值變成 inf,nav2 靠那個值做 raytrace 清除,於是舊障礙物永遠留著。
  設成 29.0(與 Isaac 那份設定一致)
- **`navigation.py`**:`controller_server` 和 `velocity_smoother` 的 `cmd_vel` 都
  remap 到同一個 topic,平滑器訂閱自己的輸出形成迴圈。改成
  `controller → cmd_vel_nav → smoother → 底盤`
- **`_d435i.urdf.xacro`**:`librealsense_gazebo_plugin` 載入時丟例外且會隨機
  帶走 gzserver,而且本來就沒產生影像。加開關,預設關閉
- **`maze.world`**:檔尾有一行殘留的貼上文字,讓該檔不是合法 XML

---

## 留下的測試工具

在 `/tmp/.../scratchpad/`(非 repo,需要時再搬進來):

| 工具 | 用途 |
|------|------|
| `verify_plant.py` | 底盤驗收:四輪著地 / 直線 / 原地旋轉,直接給 PASS/FAIL |
| `radius_check.py` | 指令輪速 vs 實際輪速 vs 真實車速,算有效滾動半徑 |
| `creep_check.py` | 輪子下 0 指令時車子會不會自己動 |
| `track_test.py` | 直線/轉角/圓形循跡,同時算 odom 與世界座標誤差 |
| `run_maze.sh` | 完整 maze 穿越:起 sim、用真值播種 AMCL、起導航、送目標 |
| `run_maze_gui.sh` | 同上但開 Gazebo 視窗和 RViz |
| `check_urdf.py` | 驗證 URDF 改動真的到達 Gazebo |
| `killall.sh` | 安全的 teardown(模式字串不會匹配到呼叫端) |

**任何底盤或參數的調整都要用重複試驗評估**,單次測量在這個環境裡不可信。
