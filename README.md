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

```bash
# モーター: 車輪を浮かせた状態で速度ステップ
uv run pend log PORT -d 3 -k step -c "STEP 10 1.0"
uv run pend fit-step logs/<file>_step.csv --update      # motor_tau
```

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

- `|th| > THLIM`、走行距離 `> XLIM`、または I2C エラーで自動停止する。
- **B ボタン** / `STOP` でいつでも停止できる。

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
| `GET` / `SAVE` | パラメータ表示 / フラッシュ保存 |
| `INFO` | Roller / IMU の状態表示 |
| `SETADDR old new` | Roller の I2C アドレス変更（変更する1台だけをつなぐこと） |
| `SETUP` | Roller のモードを再設定 |

制御周期は `firmware/src/main.cpp` の `kDtUs`（既定 10 ms）。変更したら `params.toml` の `dt` も合わせること。
