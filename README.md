# inverted_pendulum

M5Stack FIRE と Unit Roller485 Lite 2台で作る二輪型倒立振子（車体そのものが振子）。
参考: [Carter氏のQiita記事](https://qiita.com/Carter/items/7d4556ceeeef71364a09)（MATLAB/Simulink版）を、
**PC側 Python + 実機 C++(PlatformIO)** で作り直したもの。

```
firmware/        M5Stack FIRE 用ファームウェア (PlatformIO, Arduino)
pendulum/        PC側 Python ツール (モデル, LQR設計, シミュレーション, 同定, ログ)
params.toml      プラント・制御パラメータ (ゲインはここから再生成できる)
logs/            実験ログ (pend log / run の出力先)
```

## ハードウェア

| 役割 | ユニット | I2Cアドレス | モード |
|---|---|---|---|
| 左車輪 | Roller485 Lite | 0x64 | 速度モード |
| 右車輪 | Roller485 Lite | 0x65 | 速度モード |
| 車体の傾き | FIRE 内蔵 IMU (MPU6886) | 0x68 | 相補フィルタ |

すべて FIRE の I2C (SDA 21 / SCL 22, 400 kHz) に接続する。
Roller の電源は FIRE の Grove 5V から取る（Roller 側の表示は約 4.5V。推奨の 6〜16V より低いのは参考記事と同じ）。

## モデル

状態 `x = [th, psi, dth, dpsi]`、入力 `u` = 車輪の速度指令 [rad/s]（車体に対する相対速度）、制御則 `u = -K x`。
参考記事の `ω_ref = k1 θ + k2 θ' + k3 θm + k4 θm'` と同じ形。

- `th`: 車体の傾き。前に倒れる向きが +
- `psi`: 車体から見た車輪の回転角（左右の平均）。前に進む向きが +。走行距離は `r (th + psi)`

```
psi'' = (u - psi') / tau                                   Roller485 の速度ループを1次遅れで近似
Jt th'' = m g l sin th + m r l sin th th'^2 - Bt psi''
Jt = I_w + M_w r² + m r² + 2 m r l + J_axle,   Bt = I_w + M_w r² + m r² + m r l
```

**重心高さ l に注意。** l → 0 では角運動量 `Jt th' + Bt psi'` が保存されるため、並進（位置）が不可制御になる。
l が小さいと、加速・減速するために大きく前傾する必要がある（定常では `th ≈ Bt ẍ / (m g l r)`）。
`pend design` がこの値（1 m/s² あたりに必要な前傾角）を表示する。

## セットアップ

```bash
uv sync                                   # Python 依存
uvx platformio run -d firmware -t upload --upload-port PORT   # ファームウェア書き込み
```

`PORT` は WSL2 + usbipd なら `/dev/ttyACM0`、Windows ネイティブなら `COM3` など。

## 手順

### 0. Roller の速度 PID を User-Def にする（最初に1回だけ）

Roller485 には速度 PID のプリセット（User-Def / Light / Mid / Heavy Load）があり、Roller 本体のメニューで選ぶ。
**User-Def 以外が選ばれていると、I2C で書いた PID は保存されるだけで制御には使われない**（読み出しも常にプリセットの値になる）。
工場出荷時や過去の設定で Heavy Load などになっていることがある。

1. Roller のボタンを押したまま電源を入れる → 設定メニューが開く
2. 車輪（モーター軸）を回してカーソルを動かし、`SPEED PID` でボタンを押す
3. `User-Def` を選んでボタンを押す（Roller のフラッシュに保存される）
4. `Quit` で抜ける。左右 2 台とも行う

FIRE は起動時に `SPID` の値（既定は参考記事と同じ `200000 0 85000000` = P2 I0 D850）を両方の Roller に書き込む。
Roller の既定 PID（P25 I3e-6 D400）では速度ステップに 2〜3 倍のオーバーシュートと 20 ms の遅れがあり、倒立は安定しない。

### 1. 接続確認

```bash
uv run pend send PORT INFO GET
```

左右の Roller が `mode=1(speed)` になっていること、IMU の `whoami` が表示されること（MPU6886 なら 0x19）を確認する。

### 2. 向きと符号を合わせる（車輪は浮かせておく）

LCD を見ながら確認する。

1. **IMU 軸**: `IMU <gyro> <fwd> <up>` で設定する。軸は符号付きの番号 (1=x, 2=y, 3=z, 負号で反転)。
   `INFO` の `acc` を見て、直立時に +9.8 になる軸を `up` にする。前後方向の軸を `fwd` にし、ピッチ回りの軸を `gyro` にする。
   - ゆっくり前に傾けたとき `thacc` が増えること。減るなら `fwd` の符号を反転する。
   - 素早く前に傾けたとき `dth` が + になり、`th` と `thacc` が同じ向きに動くこと。逆なら `gyro` の符号を反転する。
2. **車輪の向き**: `pend send PORT "STEP 5 1"` を送る。左右とも前に進む向きに回れば OK。
   逆に回る側は `SGN <左> <右>` で符号を反転する（既定は `SGN 1 -1`）。
3. **直立の校正**: 車体を釣り合う位置で直立させて静止させ、**C ボタン**（または `CAL`）を押す。
   3 秒間の平均から `TRIM`（直立時の加速度計の角度）とジャイロのバイアスを取る。
   重心が低い機体では、この値のずれがそのまま走り出し（ドリフト）になるので丁寧に行う。
4. `pend send PORT SAVE` で FIRE のフラッシュに保存する。

### 3. 同定

> **ステップ試験（`STEP` / `step-sweep`）は必ず車輪を浮かせて行う。**
> 機体を仰向けに寝かせるか逆さに持ち、車輪が何にも触れず空転する状態にする。
> `STEP` は倒立制御をしない開ループ試験なので、床に立てたまま行うと倒れて走り出し、モーターの特性も測れない。
> ログの `th` が試験中ほぼ一定なら正しく測れている。床に立てるのは `pend run`（倒立制御）のときだけ。

```bash
# モーター: 車輪を浮かせた状態で速度ステップ
uv run pend log PORT -d 3 -k step -c "STEP 10 1.0"
uv run pend fit-step logs/<file>_step.csv --update      # motor_tau と実測モデル (motor_k/wn/zeta/tz/delay)

# 速度 PID をいくつか試して比べる（終わると元の PID に戻す）
uv run pend step-sweep PORT                              # --pid "P I D" で候補を指定できる
```

`fit-step` は速度ループを「零点付き2次系＋むだ時間」で当てはめる。`pend design` はこの実測モデルでも安定性を確認し、
LQR ゲインが不安定なら、実測モデル上で 4 つのゲインを最適化し直す。

質量 `m_body`, `m_wheels`、重心高さ `l`、慣性 `J_body` は `params.toml` に実測値や推定値を書く。
`J_body` は、車輪を固定して機体を逆さに吊るし、自由振動させると測れる（`pend fit-swing ... --update` で `swing_wn` を書き込む）。
ただし l が小さいと、周期が長く摩擦が支配的になり、測れないことがある。

### 4. ゲイン設計と送信

```bash
uv run pend design                       # K, 極, 必要前傾角, 非線形シミュレーション
uv run pend design --send PORT --save    # FIRE に K/TF/UMAX/R を送って保存
```

チューニングは `params.toml` の `[lqr] q, r` を変更して行う。重みの順序は `[th, psi, dth, dpsi]`。
`[sim] theta_bias_deg` で傾きの計測誤差を入れたときの挙動も確認できる。

### 5. 倒立

```bash
uv run pend run PORT -d 20      # ARM → 車体を立てると自動で制御開始
uv run pend plot logs/<file>_run.csv
```

- **車輪を床に着け、車体は指先で軽く支えるだけにする。** 手でしっかり持つ・車輪が浮いていると、
  車体が傾けないため車輪角のフィードバック（正帰還）で車輪が暴走する。
- `|th| > THLIM`、走行距離 `> XLIM`、指令が 0.3 秒以上上限に張り付く、または I2C エラーで自動停止する。
- **B ボタン** / `STOP` でいつでも停止できる。

### 6. スマホで操縦する

FIRE は起動すると Wi-Fi のアクセスポイントになる（LCD の `wifi` 行に SSID を表示、パスワード `pendulum`）。
スマホをその Wi-Fi につなぎ、ブラウザで `http://192.168.4.1` を開くと、ジョイスティックと ARM / STOP ボタンが出る。

- ARM → 車体を立てると倒立開始。ジョイスティックの上下で前後、左右で旋回。
- 前後は目標位置を動かして追従させる（最高 0.3 m/s、加速度 0.3 m/s²。急加速は大きく前傾するため）。
- 指令が 0.5 秒途切れる（指を離す・Wi-Fi 切断）とその場で止まる。
- `DRIVE vmax amax yaw` で最高速度 [m/s]・加速度 [m/s²]・旋回の車輪速度差 [rad/s] を変えられる（`SAVE` で保存）。

**USB なしで動かす場合**: 電源を入れて約 1.5 秒静止させ（ジャイロバイアス取得）、A ボタンで ARM、立てると開始、B で停止。
制御中のデータは本体（PSRAM）に直近 60 秒ぶん記録されるので、あとで USB を挿して `uv run pend dump PORT --clear` で取り出す。

## シリアルコマンド (921600 baud)

| コマンド | 内容 |
|---|---|
| `K k1 k2 k3 k4` | フィードバックゲイン (`u = -K x`) |
| `ARM` / `STOP` | 倒立待機 (`|th| < ARMW` で開始) / 停止 |
| `STEP u dur` | 車輪速度 `u` [rad/s] を `dur` [s] 出す開ループ試験 |
| `LOG 0/1` | `D,t_ms,state,th,psi,dth,dpsi,u,th_acc,exec_us` を毎周期出力 |
| `CAL` | 直立静止で TRIM とジャイロバイアスを校正 (C ボタンと同じ) |
| `GBIAS` | 任意の姿勢で静止させ、ジャイロバイアスだけ校正 |
| `IMU g f u` | IMU 軸の割り当て (符号付き 1..3) |
| `SGN l r` | 左右車輪の符号 (±1) |
| `R` `TRIM` `TC` `TF` `UMAX` `THLIM` `XLIM` `ARMW` `IMAX` | 車輪半径[m], 直立時の傾き[deg], 相補フィルタ時定数[s], 微分フィルタ[s], 速度上限[rad/s], 角度[deg]/走行距離[m]リミット, 開始窓[deg], 最大電流[mA] |
| `DUMP` / `CLEARLOG` | 本体に記録した直近 60 秒（制御中＋停止後 1 秒）を出力 / 消去。`pend dump PORT` で CSV に保存 |
| `DRIVE vmax amax yaw` | スマホ操縦の最高速度 [m/s]・加速度 [m/s²]・旋回の車輪速度差 [rad/s] |
| `GET` / `SAVE` | パラメータ表示 / フラッシュ保存 |
| `INFO` | Roller / IMU の状態表示 |
| `SPID` / `SPID p i d` | Roller の速度 PID の読み出し / 書き込み（生の値。P/1e5, I/1e7, D/1e5）。`SAVE` すると起動時に再送 |
| `SETADDR old new` | Roller の I2C アドレス変更（変更する1台だけをつなぐこと） |
| `SETUP` | Roller のモードを再設定 |

制御周期は `firmware/src/main.cpp` の `kDtUs`（既定 10 ms）。変更したら `params.toml` の `dt` も合わせること。
