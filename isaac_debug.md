# Claude Debug 紀錄 — Isaac 版底盤同步與 maze 窄道(2026-08-01)

接續 [`gazebo_debug.md`](gazebo_debug.md)(2026-07-31 的 Gazebo 底盤大修)。
那次修正**只有一半傳到 Isaac**:共用檔案改到的部分 Isaac 自動吃到,但 Isaac 專屬的
複本、USD 快照、以及 README 都留在舊狀態。本文記錄對齊過程,以及 maze 那條
0.933 m 窄道的實測與修正。

---

## 一、Isaac 與 Gazebo 不同步的地方

| 項目 | 修正前 | 修正後 | 怎麼發現的 |
|------|--------|--------|-----------|
| `diff_cont` 速度上限 | 1.0 / **−1.0** m/s | 1.5 / −0.3(MiR100 規格) | diff 兩份 yaml |
| 控制器設定檔 | Isaac 是**整份複本**,已漂移 | 共用檔 + 2 行 delta 覆蓋檔 | 同上 |
| `odom→base_footprint` | 文件教的兩終端機流程下**沒有人發** | Isaac 預設發布,全場只有 1 個發布者 | 對照 yaml 與 argparse 預設值 |
| USD 輪關節速度上限 | 1145.9 °/s(20 rad/s,舊 xacro) | 1375.1 °/s(24 rad/s) | 讀 USD 的 `physxJoint:maxJointVelocity` |
| `isaac_dir` launch 預設 | 指到不存在的路徑 | 指到實際目錄 | `ls` |
| README「已驗證的預設值」 | 與程式碼相反(說 2.0/5.0,實際 0.0) | 改成實際值並註明理由 | 讀 argparse |

### 1.1 為什麼改成「共用檔 + 覆蓋檔」

`diffdrive_controller_isaac.yaml` 本來是 `diffdrive_controller.yaml` 的完整複本,
檔頭還寫著「唯一差別是 `enable_odom_tf`」——但 7/31 的規格對齊只改了 Gazebo 那份,
註解就變成假的,而且**沒有任何機制會發現**。

現在 `mir_isaac.launch.py` 依序把兩個檔案都交給 `ros2_control_node`,後面的覆蓋前面的:

```python
parameters=[robot_description, controllers_file, controllers_file_isaac, ...]
```

Isaac 那份只留 `enable_odom_tf: false`。實測確認覆蓋生效:

```
ros2 param get /diff_cont enable_odom_tf          -> False   (來自 Isaac 覆蓋檔)
ros2 param get /diff_cont linear.x.max_velocity   -> 1.5     (來自共用檔)
ros2 param get /diff_cont linear.x.min_velocity   -> -0.3    (來自共用檔)
```

### 1.2 `odom→base_footprint` 的靜默斷鏈

Isaac 那份設 `enable_odom_tf: false`,前提是 Isaac 端要發 odom。但
`mir_isaac_sim.py` 的 `--publish-odom` **預設是關的**,只有
`mir_isaac.launch.py launch_isaac:=true` 這條路徑會自動帶。照 README 開兩個終端機
的標準流程 → **兩邊都不發**,Nav2/AMCL 完全沒有到機器人的 TF 鏈。
症狀很難認:goal 會被接受,然後車子不動。

修法:`--publish-odom` 改成 `argparse.BooleanOptionalAction` 且預設 True,
要關就用 `--no-publish-odom`(help 裡註明關了必須同時把 `enable_odom_tf` 打開)。
驗證方法(數 `/tf` 上這組 frame 的訊息數,應該只有一個來源):

```bash
ros2 topic echo /tf | grep -c base_footprint
```

### 1.3 USD 是快照,改 xacro 一定要重生成

USD 是 URDF 的**快照**,不會跟著 xacro 走。7/31 把輪關節上限從 20 改到 24 rad/s,
但 USD 沒重生成,PhysX 仍然把輪子鎖在 20 rad/s(=1.25 m/s),而 ROS 端以為自己
可以下 1.5 m/s。這種不一致不會有任何錯誤訊息。

驗證方式(直接讀 USD,不要相信 URDF):

```python
from pxr import Usd
s = Usd.Stage.Open("isaac_sim/usd/mir_isaac.usd")
# left_wheel_joint 的 physxJoint:maxJointVelocity 是「度/秒」
# 24 rad/s -> 1375.1 ;  20 rad/s -> 1145.9
```

重生成流程見 `isaac_sim/README_isaac.md` §3。順帶修掉
`make_isaac_urdf.py` 寫死的 `../install`:repo 從 workspace 根搬到 `src/` 之後那個
路徑就失效了,`package://` 的 mesh 會被整批丟掉且**只印一行 dropped**,很容易漏看。
現在改成往上層找 `install/`。

---

## 二、maze 窄道

### 2.1 幾何(先量,不要猜)

Isaac 的 maze 是用 `convert_maze_to_usd.py` 從 Gazebo 的
`mir_gazebo/worlds/include/maze/model.sdf` 產生的,所以兩個模擬器跑的是**同一組牆**。
直接從 SDF 算出來:

| 項目 | 數值 |
|------|------|
| 窄道 = Wall_6 東面 ~ Wall_27 西面 | x ∈ [6.023, 6.956] → **0.933 m** |
| 窄道長度(Wall_27 南端 ~ Wall_6 北端) | y ∈ [2.084, 3.393] → 1.31 m |
| MiR100 車寬 | 0.580 m |
| **置中時單邊餘裕** | **17.6 cm** |

註:`gazebo_debug.md` 寫的「1.1 m 缺口」是估的,實際是 0.933 m。
`maze.pgm` 這張 SLAM 地圖把牆面畫在 6.05 / 6.95,比真實幾何保守 2~3 cm。

### 2.2 基準測試(修正前)

Isaac headless + `--lasers` + maze,南北來回各 5 趟,用 Isaac ground-truth
(`--publish-odom` 的 `/odom`)量實際位置:

| 窄道內 x 範圍 | 車身餘裕 | 結果 |
|---|---|---|
| 6.44 – 6.53(接近中線 6.49) | 12~14 cm | 6 趟全過 |
| 6.35 – 6.40(貼西牆) | 3~5 cm | 3 失敗 / 1 過 |

**9 次穿越嘗試,過 6 次(67%)**。失敗全部是同一個模式:車子往西偏約 12 cm,
餘裕掉到 5 cm 以下,MPPI 就再也採樣不到不碰撞的軌跡,
`controller_server` 報 `Failed to make progress`,recovery 的 spin 又被
`Collision Ahead` 擋掉,最後 `Goal failed`。

定位誤差不是主因,但會吃掉餘裕。AMCL vs ground truth(修正前後合計 1090 個樣本,
兩段沒有差異):

| | 平均 | p95 | 最大 |
|---|---|---|---|
| 全程位置誤差 | 4.8 cm | 9.5 cm | 15.7 cm |
| 窄道內位置誤差 | 4.2 cm | 8.4 cm | 9.9 cm |
| 航向誤差 | 0.93° | 2.63° | — |

(單邊餘裕 17.6 cm,p95 的 9.5 cm 定位誤差就吃掉一半 —— 所以餘裕本來就不多,
但失敗與成功的差別是 12 cm 的橫向偏移,不是 5 cm 的定位雜訊。)

### 2.3 根因:PathAlignCritic 在窄道裡把自己關掉

`PathAlignCritic` 有一個 `max_path_occupancy_ratio` 參數:**當路徑上超過這個比例的
點落在 inscribed-inflated 的格子上,這個 critic 就直接 return,不出力**。

MiR100 的 inscribed radius 是 0.29 m,窄道只有 0.933 m 寬 —— 接近與進入窄道時,
local costmap 看到的那段路徑幾乎整段都在這種格子上。本專案原本設 **0.05**
(比 nav2 預設 0.07 還嚴格),於是:

> 把車子壓在中線上、權重被特意調到 40 的那個 critic,
> **正好在最需要它的地方失效**。

這解釋了為什麼失敗都是「往同一側偏」——兩個方向的進入路徑都是從西邊繞過牆角,
切彎自然往西偏,而此時沒有任何項在把它拉回中線。

### 2.4 修正

`mir_nav_params.yaml` 的 `FollowPath.PathAlignCritic`:

```yaml
max_path_occupancy_ratio: 0.5   # 原本 0.05
```

其餘 MPPI 參數不動。修正後跑**完全相同**的來回測試(同一個 Isaac session、
同一份地圖、只重啟 `navigation.py` 讓新參數生效):

| | 穿越嘗試 | 通過 | 通過率 | 窄道內最小車身餘裕 |
|---|---|---|---|---|
| 修正前(`0.05`) | 9 | 6 | **67%** | 3.3 cm(失敗)|
| 修正後(`0.5`) | 10 | 10 | **100%** | 7.0 cm(仍通過)|
| 再加 §2.7 的 AMCL 改善 | 11 | 11 | **100%** | **2.7 cm(仍通過)** |

最後那一列值得注意:AMCL 誤差砍到 2.6 cm 之後,**2.7 cm 餘裕都能過**——
而修正前 3.3 cm 就已經卡死。差別不只是通過率,而是車子有沒有能力從很擠的姿態
救回來(窄道內取樣數 84~175 的那幾趟都是掙扎很久但最後過了,以前這種就是
`Failed to make progress`)。

修正後 10 趟的車身餘裕分佈:7.0 / 7.4 / 9.3 / 9.4 / 9.4 / 10.8 / 11.9 / 12.4 /
12.9 / 14.3 cm —— 沒有任何一趟掉進修正前的失敗區間(<6 cm)。
「窄道內取樣數」也從失敗時的 52~94(在裡面掙扎)回到 15~22(順順地開過去)。

> 樣本數提醒:9 對 10 趟不足以宣稱「保證 100%」,誠實的說法是
> **原本的失敗模式不再重現**。要更有把握就把 `stress.sh` 的趟數加大。

⚠️ `mir_nav_params.yaml` 是 **Gazebo 共用**的,所以這個改動也會套到 Gazebo。
推論上同樣適用(同一台車、同一個 maze、同一個 critic),但**尚未回頭跑 Gazebo 驗證**。

### 2.5 北口對正:另一個獨立的失敗模式

開 GUI 給人看的時候當場撞到一個 §2.2 那 10 趟沒踩到的失敗:車子卡在窄道
**北口外面** (6.411, 3.877),朝向 −163°(該朝南進洞,結果偏了 70°),
右後方 0.513 m 就是 Wall_27 的面(離後保險桿約 12 cm)。
`Spin` 和 `BackUp` 復原全部被 `Collision Ahead` 否決,車子就在原地抖。

這和 §2.2 的失敗**不是同一件事**:那些是進洞之後在裡面貼西牆,這個是根本還沒
進洞就在洞口卡死。所以 `max_path_occupancy_ratio` 確實解掉了「窄道內偏移」,
但**北口進入的對正是獨立問題**,只是那 10 趟剛好沒抽到。
**真實通過率不是 100%。**

先記下來,還沒修:車子卡在牆邊時唯一的出路是倒車重新對正,而
`PreferForwardCritic.cost_weight: 5.0` 正在罰這個動作,`BackUp` 又被碰撞檢查擋掉
—— 等於沒有退路。這是下一個該試的東西(見 §2.6)。

### 2.6 速度:瓶頸不是 `vx_max`

「車子跑好慢」的實測結果。先量整條鏈路,證明不是被誰削掉:

| 階段 | 峰值 |
|---|---|
| MPPI 原始輸出 `/cmd_vel_nav` | 0.594 m/s |
| 經過 velocity_smoother | 0.594 m/s(沒被削)|
| 實際車速 `/odom` | 0.596 m/s |

當時 `vx_max` 是 0.8,**MPPI 從來沒要到上限**。真正的限制是
**`prune_distance`**:MPPI 推演 `time_steps × model_dt = 56 × 0.05 = 2.8 s` 的
軌跡,卻只拿 `prune_distance` 公尺的參考路徑評分 —— 軌跡不可能因為「跑得比看得到
的還遠」拿到獎勵,所以速度被壓在 `prune_distance / horizon = 1.7 / 2.8 ≈ 0.61 m/s`,
和量到的 0.594 幾乎完全吻合。

試了三種組合,只有一種是進步的:

| 設定 | 峰值 | 結果 |
|---|---|---|
| 原始(vx_max 0.8, vx_std 0.2, prune 1.7)| 0.596 m/s | 基準 |
| **vx_max 1.2**(其餘不動)| **0.703–0.721 m/s** | ✅ 採用,窄道不受影響 |
| ↑ 再加 `vx_std` 0.3 | 0.388 m/s | ❌ 只有 13% 時間在動、11 次卡住 |
| ↑ 再加 `prune_distance` 3.4 | 0.073 m/s | ❌ 141 秒走 0.2 m,完全死掉 |

兩個失敗的原因都寫進 yaml 註解了,重點是:

- **加寬取樣反而更慢**。迷宮到處是窄道,`vx_std` 放大之後多出來的高速取樣全部撞牆,
  有效樣本數塌掉,加權平均的控制輸出就又慢又飄。MPPI 的取樣分布要配合環境,
  不是越寬越好。
- **`prune_distance` 不能單獨放大**。`PathAlign` / `PathFollow` / `PathAngle` 的
  `offset_from_furthest` 是**路徑點索引**,不是距離;路徑一拉長那些 offset 就全部
  失準,整個 cost landscape 壞掉。要動它得連 `time_steps` 和那些 offset 一起重調。

淨結果:峰值 **0.596 → 0.703 m/s(+18%)**。再快就得重調上面那組耦合參數。

### 2.7 AMCL 精度(順手修,兩邊模擬共用)

窄道單邊只有 17.6 cm,而 AMCL 誤差量到平均 4.8~6.7 cm —— 吃掉快一半餘裕,
而且和窄道失敗高度相關。兩個參數:

| | 原本 | 改成 | 理由 |
|---|---|---|---|
| `max_beams` | 50 | **180** | 合併後有 360 束,只取 50 束太粗。實測 AMCL 只吃 4.9% CPU,成本很低 |
| `update_min_d` | 0.2 | **0.1** | 更新之間是純輪式推算,20 cm 對這條窄道太多 |

效果(對 Isaac 真值,各 100+ 樣本):

| | 修正前 | 修正後 |
|---|---|---|
| 位置誤差平均 | 6.7 cm | **2.6 cm** |
| p95 | 11.5 cm | **5.0 cm** |
| 最大 | 15.1 cm | **5.8 cm** |
| 航向誤差平均 | 0.86° | **0.58°** |

### 2.8 `navigation.py` 的速限覆寫失效(影響實體車)

查「Gazebo 和 Isaac 參數是否一致」時翻到的,**這個和實體機器人有關**:

`navigation.py` 用 `RewrittenYaml` 覆寫 `max_vel_x` / `max_speed_xy` /
`max_vel_theta` —— 全部是 **DWB 的參數名**。換成 MPPI 之後這些 key 不存在了,
而 `RewrittenYaml` 只取代**真的存在**的 key,所以覆寫整個落空:

```bash
ros2 param get /controller_server FollowPath.max_vel_x
# -> Parameter not set        (證據)
```

也就是 `claude_debug.md` 記的那個「實機預設降到 0.3 m/s 免得嚇到人」的保護,
**從 DWB 換成 MPPI 那天起就沒有生效過**,實體 MiR100 一直照 MPPI 的 `vx_max` 在跑。

修法兩件事:
1. 同時覆寫 MPPI 的 `vx_max` / `wz_max`(DWB 那組留著,方便切回去)。
2. launch 參數的**預設值改成從 yaml 讀回來**(`_speed_defaults_from_yaml()`)。
   否則預設值一旦寫死,它就會蓋掉 yaml,平台最高速又變成兩個地方各說各話 ——
   跟 §1.1 控制器 yaml 漂移是同一種病。

### 2.9 還沒動、但有理由的候選項

如果之後還遇到窄道卡住,依序試(一次只改一項,每項至少 8~10 次穿越才算數):

1. **`PreferForwardCritic.cost_weight` 5.0 → 2.0**。優先度最高,因為 §2.5 那次
   北口卡死有直接證據:車子卡在牆邊時唯一的出路是倒車重新對正,而這個 critic
   正在罰它,`BackUp` 復原又被 `Collision Ahead` 擋掉,等於沒有退路。
2. `ObstaclesCritic.collision_margin_distance` 0.1 → 0.03。
   窄道單邊餘裕只有 0.176 m,比 margin 的兩倍還小,兩側同時落在 margin 內時
   代價幾乎對消,critic 分辨「有沒有更置中」的能力變差。
3. `iteration_count` 1 → 2(同時把 `batch_size` 2000 → 1200 維持 CPU 預算)。
   controller_server 已經在 20 Hz 邊緣,直接加 iteration 會掉頻率。

**不要試的**(已經試過而且更糟,理由見 §2.6):放大 `vx_std`、單獨放大
`prune_distance`。

### 2.10 2026-08-01:把 Isaac 的雷射與底盤接觸對齊 Gazebo

這次不是再調共享 Nav2 參數。Gazebo 已能穩定通過,差異出在 Isaac 的模擬輸入
與碰撞形狀:

1. Gazebo 的 SICK `update_rate` 是 12.5 Hz,Isaac 的 PhysX lidar 卻跟著 60 Hz
   physics tick 發佈。感測器 graph 加入 `IsaacSimulationGate(step=5)`,實測
   `/f_scan`、`/b_scan` 約 12.5 Hz。
2. `laserscan_multi_merger` 把合法的 `pile_index == 0` 當成失敗
   (`pile_index > 0`),同步的一對 scan 因此每兩對才發一次。改成 `>= 0` 後,
   `/scan` 從約 6.3 Hz 回到 12.1--12.8 Hz。
3. Isaac 與 Gazebo 的 merger 都明確設 `scan_time: 0.08`,不再讓下游看到與
   12.5 Hz 不一致的時間中繼資料。
4. 為避免 lidar 打到自己的車殼,Isaac 原本關掉了整個 chassis collider;
   撞牆時只剩輪子與 caster 接觸,長時間卡牆會被 PhysX 彈飛。現在在
   `base_link` 下加不可見的低矮 box collider (`x=-0.39..0.50`,
   `y=-0.29..0.29`, `z=0.02..0.14`),高度低於 lidar 的 `z=0.1914`,所以既保留
   車身接觸,也不會遮住雷射。

保留共享 MPPI 設定 `batch_size=2000`、`PreferForwardCritic.cost_weight=5.0` 的
乾淨雙向實測:

| 方向 | 結果 | 模擬時間 | 窄道內最小側向餘量 | 最大 \|z\| |
|---|---:|---:|---:|---:|
| 南→北 | 成功 | 52.8 s | 14.3 cm | 0.004 m |
| 北→南 | 成功 | 140.8 s | 10.2 cm | 0.013 m |

兩次都穿過 0.93 m 窄口且留在地圖範圍內,沒有再發生 ejection。北→南仍可看到
`controller_server` 在 CPU powersave governor 下錯過 20 Hz deadline,所以時間較長,
但復原後可完成。另做的 `batch_size=1200` A/B 反而在南口逾時卡住;不要為了消除
deadline warning 把共享設定降到 1200。

### 2.11 2026-08-01:新目標先轉向,但不要在窄口反覆重轉

直接使用 MPPI 時,差速底盤會一邊轉一邊前進。共享 `FollowPath` 現在以
`RotationShimController` 包住原本的 MPPI,大角度新路徑先做零線速度原地旋轉。
這項修改有兩個不能拆開的配套:

1. progress checker 改為 `PoseProgressChecker`,`required_movement_angle=0.12`;
   否則原本只看平移的 checker 會把健康的原地旋轉誤判成「沒有進度」。
2. `closed_loop=false`:velocity smoother 已限制 2.0 rad/s²。Shim 若再用 Isaac
   落後的 odometry 做第二次閉迴路限幅,雖設定 0.9 rad/s,raw 命令仍長期只有
   約 0.27 rad/s。改為 open-loop command ramp 後可達 0.9 rad/s,實際 odom
   峰值 0.687 rad/s。

最初試的 20° engage / 7° disengage 太敏感:NavFn 每秒更新路徑,車到窄口入口
`(6.47,1.40)` 時 Shim 反覆重新對正,163.5 s 仍未進入窄道。最終改為
**45° engage (`0.785`) / 20° disengage (`0.35`)**:真正的大角度新目標先轉,
窄口附近的小角度修正交回 MPPI。

`rotation_probe.py` 的 A/B 結果:約 150° 的對正耗時 5.25 s,raw 與 smoothed
命令峰值都是 0.9 rad/s;最後一筆 rotate-only 到第一筆 forward 的間隔只有
0.050--0.065 s(一個 controller cycle),所以沒有額外的交棒停頓。最終雙向窄口
回歸:

| 方向 | 結果 | 模擬時間 | 窄道內最小側向餘量 | 最大 \|z\| |
|---|---:|---:|---:|---:|
| 北→南 | 成功 | 60.0 s | 9.6 cm | 0.002 m |
| 南→北 | 成功 | 111.3 s | 12.6 cm | 0.004 m |

---

## 三、方法與陷阱

### 1. `pkill -f <pattern>` 又把呼叫端殺掉了

`gazebo_debug.md` 陷阱 4 原封不動重演:在 Bash 直接下
`pkill -f "navigation.py"`,該 shell 自己的命令列就含這個字串 → 把自己殺掉,
後半段全部沒執行,而且 nav2 的 server 變成孤兒繼續用**舊參數**跑
(看起來「改了沒效」)。修法一樣:把清理邏輯寫進腳本檔,並且用執行檔路徑
(`nav2_controller/controller_server`)而不是 launch 檔名當 pattern。

### 2. action 的 status 碼不要憑印象

`action_msgs/GoalStatus`:**4=SUCCEEDED, 5=CANCELED, 6=ABORTED**。
一開始把 5/6 寫反,導覽失敗被印成 "CANCELED",差點以為是自己的腳本在取消目標。

### 3. 量定位誤差要有 ground truth

`--publish-odom` 的 `/odom` 就是 Isaac 的真值。同時訂 `/odom` 和 `/amcl_pose`
寫成 CSV,就能把「定位誤差」和「控制誤差」分開 —— 這次的結論
(定位不是主因)完全靠這個數據。工具:`pose_logger.py`(見下)。

### 4. `/scan` 的 QoS 是 BEST_EFFORT

用預設 QoS 訂 `/scan` 會收到
`incompatible QoS ... RELIABILITY` 然後**一則訊息都收不到**。
要用 `qos_profile_sensor_data`。

### 5. Isaac 的閒置漂移:開機瞬態,不是持續蠕動

開機後車體會偏約 5 cm / 6.6°(spawn 落地 + 手臂 home 的瞬態),但**之後完全靜止**:
58 秒模擬時間內 0.000 mm/s、0.0°/h。所以 `--base-*-damping` /
`--caster-swivel-damping` 這些阻尼旋鈕預設 0.0 是對的,不要照舊版 README 去開
(`--base-linear-damping 2.0` 會拖到 Nav2 下幾乎推不動車)。

---

## 四、留下的工具

這次的驗收工具**有進 repo**(在 `isaac_sim/` 底下)——`gazebo_debug.md` 那批
留在 scratchpad 的工具現在已經找不到了,不要再重蹈覆轍:

| 工具 | 用途 |
|------|------|
| `isaac_sim/maze_run.py` | 送一個 NavigateToPose 目標,回報是否穿越窄道、窄道內 x 範圍、車身餘裕、有沒有被 PhysX 彈飛 |
| `isaac_sim/stress.sh` | 南北來回 N 趟,產生可統計的通過率(用法與前置條件寫在檔頭) |
| `isaac_sim/pose_logger.py` | 同步記錄 ground truth 與 AMCL 寫成 CSV,算定位誤差 |
| `isaac_sim/summarise.py` | 把 stress log 整理成通過率表 |
| `isaac_sim/inspect_wheels.py` | 直接讀 USD 的輪關節限制,驗證 USD 有沒有跟上 xacro |

**任何底盤或參數調整都要用重複試驗評估** —— 這次基準是 9 次穿越,
單次結果在這個餘裕下完全不可信。
