# Claude Debug 紀錄 — 實體 MiR + UR5 連線問題(2026-07-03)

本文記錄這次實體機器人 bring-up 過程中所有 debug 的來龍去脈、診斷方法與修正。
對應的操作指令已同步更新到 `/mnt/data/mir_isaac/mir_ur5/README.md`「第二部:實體機器人操作」。

---

## 問題 1:UR5 External Control 連上 1~15 秒就斷

### 症狀
- `robot_program_running` 一直是 `false`,MoveIt 不能動手臂
- driver log:`Robot connected to reverse interface` → 1~15 秒後
  `Connection to reverse interface dropped`
- 手臂本體完全正常(dashboard:`Robotmode: RUNNING`、`Safetystatus: NORMAL`)

### 診斷過程(方法可重用)
1. **dashboard 直接查狀態**(不經 ROS,排除 ROS 因素):
   ```bash
   # 對 29999 埠送指令:robotmode / programState / safetystatus / running
   python3 -c "import socket,time; s=socket.create_connection(('192.168.0.101',29999),timeout=5); s.recv(4096); s.send(b'running\n'); time.sleep(0.4); print(s.recv(4096).decode())"
   ```
2. **看手臂有沒有回連 reverse 埠**(50001/50003/50004):
   ```bash
   ss -tn | grep -E ':5000[134]'    # 有 ESTAB = 控制通道活著
   ```
3. **driver log 是關鍵證據**(`~/.ros/log/ur_ros2_control_node_*.log` 或啟動時導向的檔案):
   - `Pipeline producer overflowed!` → RTDE 讀取跟不上(啟動期正常,持續出現才有問題)
   - `Connection to reverse interface dropped` → 手臂端看門狗把連線砍了
   - `Unable to get data package while reconnecting to the RTDE interface` → 連 RTDE 都斷,代表整條網路瞬斷
4. **量網路品質**(本機 → 手臂走 WiFi!):
   ```bash
   ping -c 300 -i 0.1 192.168.0.101   # 看 p95/p99/max,不要只看平均
   ```
   實測:平均 0.8ms,但尖峰 48/66/76/**117ms**,且每 15~60 秒有 0.3 秒級全靜默。

### 根因(三層,一層比一層深)
1. **WiFi 省電模式**造成週期性斷流(NetworkManager `powersave: default` = 交給驅動 = 開)。
2. External Control 腳本看門狗預設 `keep_alive_count=2` × 20ms = **只容忍 40ms**。
3. 調大 `keep_alive_count` 後**仍然**斷 → 發現 `ur_client_library` 有硬上限:
   - 只要 `scaled_joint_trajectory_controller` active,driver 就持續走 servoj 即時指令路徑,
     timeout 被 `MAX_RT_RECEIVE_TIMEOUT_MS = 200ms` 硬截(log 有 ERROR:
     `Robot receive timeout ...ms is above the maximum allowed timeout for realtime commands 200`)
   - idle 路徑 `writeKeepalive()` 則是寫死預設 1000ms,`keep_alive_count` 也管不到

### 修正(全部做完後 10 分鐘監測零斷線)
1. 關 WiFi 省電:
   ```bash
   nmcli connection modify "TP-Link_550F_5G" 802-11-wireless.powersave 2
   nmcli connection down "TP-Link_550F_5G" && nmcli connection up "TP-Link_550F_5G"
   ```
2. `Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro`:新增頂層 `keep_alive_count`
   xacro 參數(預設 250 = 5 秒)並傳入 `ur_robot` macro。
3. **把官方 `Universal_Robots_Client_Library` 2.6.1(與 /opt 同版)放進 workspace**
   (`src/Universal_Robots_Client_Library`),兩處修改:
   - `include/ur_client_library/ur/robot_receive_timeout.h`:
     `MAX_RT_RECEIVE_TIMEOUT_MS` 200 → **5000ms**
   - `ur_robot_driver/src/hardware_interface.cpp`:
     `writeKeepalive()` → `writeKeepalive(receive_timeout_)`
   ```bash
   colcon build --packages-select ur_client_library ur_robot_driver ur_description
   # 驗證 driver 連到 workspace 版:
   ldd install/ur_robot_driver/lib/libur_robot_driver_plugin.so | grep urcl
   ```
4. driver 啟動指令加 `headless_mode:=true`(不需教導器,腳本由 driver 注入)
   和 `initial_joint_controller:=scaled_joint_trajectory_controller activate_joint_controller:=true`。
5. 斷線自救:`mir_ur5/ur_program_watchdog.sh`(偵測到 `Program running: false`
   自動呼叫 `resend_robot_program`;注意腳本注入後要 ~6 秒才起來,別太早判定失敗)。

### 其他發現
- 本機 WiFi IP 是 **192.168.0.100**(README 舊記載 192.168.0.27 已過時)——
  headless 模式不受影響(reverse IP 自動偵測),只有教導器 External Control 程式指向的 IP 要注意。
- 手臂 6 軸的 `/joint_states` 走 RTDE,**不需要** External Control 也會流 ——
  「關節有資料」不代表「能控制」,要看 `Program running: true`。

---

## 問題 2:MoveIt 能動但執行會卡頓

### 根因
`scaled_joint_trajectory_controller` 以 **125Hz 串流**每 8ms 一個 servoj 目標點,
WiFi 抖動(p95 ≈ 52ms)讓目標點遲到 → 手臂一頓一頓。

### 修正
MoveIt 預設控制器改成 **`passthrough_trajectory_controller`**:整條軌跡一次交給
控制箱、由手臂韌體自己內插,執行中不怕 WiFi 抖動。
- 改 `ur_moveit_config/config/controllers.yaml`:passthrough `default: true`,
  scaled `default: false`(兩個都保留,可切換,見下)
- `colcon build --packages-select ur_moveit_config` 後重啟 MoveIt

### 兩個控制器怎麼切換
**主要方式 — launch 參數**(driver 的 `ur_control.launch.py` 內建
`initial_joint_controller`,選誰誰就 active,另一個載入但 inactive):
```bash
# WiFi 推薦(預設寫進 README A1)
ros2 launch ur_robot_driver ur_control.launch.py ... \
    initial_joint_controller:=passthrough_trajectory_controller activate_joint_controller:=true

# 改用 125Hz 串流
ros2 launch ur_robot_driver ur_control.launch.py ... \
    initial_joint_controller:=scaled_joint_trajectory_controller activate_joint_controller:=true
```
MoveIt 兩個都認得(controllers.yaml 都有列),會自動把軌跡送到 active 的那個,
**切換只要改參數重啟 driver,MoveIt 不用動**。

輔助方式 — 不重啟 driver 的線上切換(手臂靜止時):
```bash
./mir_ur5/switch_arm_controller.sh passthrough|scaled   # 不帶參數 = 顯示目前狀態
```
差異:passthrough 執行中不能被 ROS 端即時修改軌跡(但可以 cancel);
scaled 支援 speed slider 即時縮放與較嚴格的執行監控。

---

## 問題 3:RViz 定位和 MiR 網頁不同步(+ 隱藏的 TF 打架)

### 發現的隱藏問題
舊流程(本地 AMCL + map_server maze.yaml)下,`ros2 topic info /map -v` 顯示
**兩個發布者**:`mir_bridge`(車上地圖,latched)+ 本地 `map_server`(maze.yaml);
2D Pose Estimate 後本地 AMCL 的 map→odom 又和車上定位互搶 → 位置怪、雷射不齊。

### 修正:`use_mir_localization` 模式(預設開)
- `mir_nav_launch.py` 預設不再起本地 map_server/AMCL,直接用:
  - 車上地圖:bridge 轉發的 `/map`(= 網頁那份)
  - 車上定位:新節點 `mir_driver/mir_localization_tf.py` 把 `/robot_pose`(10Hz)
    換算成 **map→odom TF**(bridge 的 `/tf` 轉發只有 odom→base_footprint,沒有 map→odom)
- 結果:RViz 位置 == 網頁位置,**不需要 2D Pose Estimate**;
  RViz 點 2D Pose Estimate 會經 `/initialpose` 轉送車上 AMCL(網頁位置一起動)
- 驗證方法:
  ```bash
  ros2 run tf2_ros tf2_echo map base_footprint   # 應與 /robot_pose 一致
  ros2 topic info /map -v                        # 發布者應只剩 mir_bridge
  ```
- 舊行為:`use_mir_localization:=false map:=maze.yaml`

---

## 問題 4:ur_ros2_control_node 段錯誤(一次性,觀察中)

- **症狀**:`ros2 control list_controllers` 卡住不回;`ps` 只剩 launch 外殼,
  `ur_ros2_control_node` 不見了。
- **證據**:driver log 出現
  `Segmentation fault (Address not mapped to object [0x10])`,backtrace 崩在
  `tf2::BufferCore::setTransform`(TransformListener 訂閱回呼執行緒)——
  humble 版 tf2 的已知執行緒安全問題,與 WiFi 無關。
- **處置**:重啟 driver 即可。若再發生,考慮排查 ur_ros2_control_node 裡誰在
  聽 /tf(嫌疑:tcp_pose_broadcaster / robot_state_helper),或升級 tf2。
- **教訓**:driver「服務不回應」不一定是網路 — 先 `ps aux | grep ur_ros2_control`
  確認進程還活著,再看 log 找 `process has died`。

---

## 問題 5:2D Goal Pose 點了不會動(Goal failed)

### 症狀
RViz 點 2D Goal Pose 後機器人不動;nav log:
`Begin navigating from (4.20, 5.80) to (4.96, 5.86)` →
`unknown goal response, ignoring...` ×3 →
`Behavior tree threw exception: BtActionNode::Tick: invalid status value` → `Goal failed`。

### 根因
**整套導航堆疊重複跑了兩份**(不同時間各 launch 一次、舊的沒關乾淨),
兩份同名的 planner/controller action server 同時回應,bt_navigator 的
action client 收到不認得的 goal response 直接炸掉。
更誇張的殘留:twist_stamper 累積 6 個、teleop 鍵盤 3 個(都在發 /cmd_vel)。
這是 README 已知問題「多個 mir_bridge 同時跑」的加強版。

### 判斷方法
```bash
ps aux | grep -E "bt_navigator|mir_bridge|twist_stamper|planner_server" | grep -v grep
# 每個名字只能有一個!看到 unknown goal response 幾乎可以斷定有兩份
```

### 修法
把所有導航相關進程殺乾淨再開一份:
```bash
pkill -f mir_nav_launch.py; pkill -f mir_launch.py
pkill -f twist_stamper; pkill -f teleop_twist_keyboard; pkill xterm
pkill -f mir_bridge; pkill -f mir_localization_tf
# 確認歸零後再 launch 一次 mir_nav_launch.py
```
教訓:每次重跑 launch 前先確認上一份死透(`ros2 launch` Ctrl-C 有時不會帶走
xterm/teleop/twist_stamper 這些孫進程)。

---

## 問題 6:2D Goal Pose 規劃正常但車子完全不動(cmd_vel 發錯 topic)

### 症狀
(問題 5 修完後)nav log 一切正常:`Received a goal, begin computing control
effort` + 每秒 `Passing new path to controller`,但 `/cmd_vel` 和
`/cmd_vel_stamped` **0 訊息**,車子一動不動 — 不是撞到障礙、不是定位問題。

### 根因
`navigation.py` 的 `cmd_vel_topic` 預設是 **`diff_cont/cmd_vel_unstamped`**
(Isaac/Gazebo 模擬的 diff_drive controller topic)。實體車的路徑是
`/cmd_vel` → twist_stamper(轉 TwistStamped)→ mir_bridge → 車子。
Nav2 一直對著不存在的模擬底盤喊話。

### 判斷方法
```bash
ros2 node info /controller_server | grep cmd_vel   # 看輸出到哪個 topic
ros2 topic echo /cmd_vel                            # Nav2 導航中應該要有訊息
```

### 修法
`mir_nav_launch.py` 對 navigation include 傳 `cmd_vel_topic: 'cmd_vel'`
(navigation.py 本來就有這個參數;模擬 launch 用預設值不受影響)。

---

## 問題 7:換 WiFi(Aislab_mir3)後整套又不通(2026-07-04)

### 背景:換網路時什麼要跟著變
電腦從 `TP-Link_550F_5G` 換到 **MiR 自己的熱點 `Aislab_mir3`**:
- MiR 位址:`192.168.0.104`(TP-Link)→ **`192.168.12.20`**(熱點;車上 DNS 會把
  `mir.com` 解析到自己)。**`mir_hostname` 要跟著改**,不確定就用
  `tools/find_mir.sh` 自動偵測(mir.com → 已知位址 → 掃 9090 埠)。
- 電腦 IP 變 `192.168.0.27`,UR5 不變 `192.168.0.101`。
- 電腦自己的 IP 不用手動填(headless driver 自動偵測 reverse IP;bridge 往外連),
  但 **driver 要重啟**(reverse IP 是啟動時烙進腳本的)。
- 新的 WiFi 設定檔要**重新關省電**(powersave 是每個 connection profile 各自的設定)。

### 坑 A:NetworkManager 一直跳回舊 WiFi
症狀:bridge 顯示已連線,但 `/odom`、`/robot_pose` 完全沒資料(訂閱被斷線沖掉)。
用 rosapi 查證車上 topic 都在,才發現是 WiFi 在兩個 AP 之間跳。修法:
```bash
nmcli connection modify "Aislab_mir3" connection.autoconnect yes connection.autoconnect-priority 100
nmcli connection modify "TP-Link_550F_5G" connection.autoconnect no
```
註:mir_bridge log 大量 `DiagnosticStatus level` / `RobotMode robotMode` 轉換錯誤是
**兩個網路都有的既有雜訊**(診斷類訊息轉不過去),不影響 odom/TF/導航,別被帶偏。

### 坑 B:passthrough 的 keepalive 又是另一個寫死的 200ms
症狀:driver 連上 reverse 後 7~8 秒斷、無限循環;但 ping p99 只有 ~120ms。
根因:**問題 1 的第三層還沒完** — passthrough 控制器 active 時的 keepalive 走
`writeTrajectoryControlMessage(TRAJECTORY_NOOP)`,其預設 timeout **寫死 200ms**,
`keep_alive_count` 傳不進去(freedrive 同病)。TP-Link 單跳勉強活;熱點是
「電腦→AP→UR5」雙 WiFi 跳,>200ms 尖峰幾秒內必中。
修法:`hardware_interface.cpp` 三處補傳 `receive_timeout_`:
`TRAJECTORY_NOOP`、`TRAJECTORY_START`、`FREEDRIVE_NOOP` →
`colcon build --packages-select ur_robot_driver` 後重啟 driver。

**經驗總結**:ur_client_library 每條 write 路徑各有自己的 timeout 預設
(servoj 上限 200ms、idle keepalive 寫死 1s、trajectory/freedrive 寫死 200ms)。
查斷線先確認**當下 active 的控制器走哪條路徑**,一條一條把 `receive_timeout_` 傳進去。

### 坑 C:spawner 偶發搶跑
症狀:launch 後 `list_controllers` 回空的;log 有
`[FATAL] spawner_...: Could not contact service /controller_manager/list_controllers`。
controller_manager 起得慢,spawner 等不到就死了 → 控制器全沒載入。
修法:重啟 launch,並帶 `controller_spawner_timeout:=60`。

### 最終結論:斷線與否取決於路由器/WiFi 品質
手臂偶爾斷線的根源是**路由器/WiFi 不穩** — **網路穩定的環境可完全忽略此問題,
看門狗也不必開**。本實驗室實測:`Aislab_mir3` 是車上的 **2.4GHz** 熱點
(訊號 ~73,TP-Link 5G 是 ~89),偶爾有 **>5 秒**的無線靜默,
5 秒容忍下手臂平均可撐 10~15 分鐘仍會斷一次。
5 秒已是安全上限(運動中斷線,手臂會保持最後目標直到看門狗超時,再大有風險),
所以在不穩網路上的**正式解 = `tools/ur_program_watchdog.sh` 常駐**:
- 每 5 秒查 dashboard,發現 `Program running: false` 自動 resend → 斷後 5~15 秒自動接回
- 手臂閒置時斷線完全無感;若剛好在執行軌跡中斷線,那次 Execute 失敗、
  等幾秒重按即可(passthrough 模式下手臂會安全停住)
- 用法:`source install/setup.bash && ./src/mir_robot/tools/ur_program_watchdog.sh`(開一個終端機掛著)

根治方向:電腦拉網路線接路由器,或 UR5↔電腦之間走有線,消掉至少一段 WiFi。

---

## 調整:導航速度上限(launch 參數化)

實測 2D Goal Pose 後車速偏快(嚇到人)。原始上限在
`mir_navigation/config/mir_nav_params.yaml`:

| 參數 | 原始值 | 實機新預設 |
|------|--------|-----------|
| `max_vel_x`(線速度) | **0.8 m/s** | 0.3 m/s |
| `max_speed_xy` | **0.8 m/s** | 跟隨 max_vel_x |
| `max_vel_theta`(角速度) | **1.0 rad/s** | 0.5 rad/s |
| velocity_smoother `max_velocity` | [0.8, 0, 1.0] | 未動(只是天花板) |
| 倒車 `min_velocity` | -0.2 m/s | 未動 |

做法:`navigation.py` 加 `max_vel_x` / `max_vel_theta` launch 參數,用
RewrittenYaml 蓋掉 yaml 值;**navigation.py 預設維持 0.8/1.0(模擬不受影響)**,
只有實機入口 `mir_nav_launch.py` 預設帶 0.3/0.5。要臨時調整:
```bash
ros2 launch mir_navigation mir_nav_launch.py mir_hostname:=192.168.0.104 \
    max_vel_x:=0.5 max_vel_theta:=0.8
```
驗證:`ros2 param get /controller_server FollowPath.max_vel_x` 應回你設的值。

---

## 其他

- **`publish_static_joints.py` 已除役**:mir_bridge 會發佈輪子/腳輪關節、UR driver
  發佈手臂 6 軸;目前 `/joint_states` 只缺 Robotiq 夾爪關節,若 MoveIt 卡在
  "complete robot state" 再處理。
- **殺殘留節點時小心**:用太寬的 pattern(如 `rviz2|lifecycle`)會誤殺 scan merger
  和 MoveIt RViz(這次就發生了)。建議精準 pattern 或 `pkill -f <launch檔名>`。
- 網路長期建議:機器人網段全走 WiFi(0.3 秒級斷流每分鐘數次),長時間任務可考慮
  把電腦用網路線接 TP-Link 路由器,少一跳 WiFi。
