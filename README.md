# inverted_pendulum

M5Stack FIRE と Unit Roller485 Lite 2台で作る台車型倒立振子。
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
| 台車駆動 | Roller485 Lite | 0x64 | 速度モード (1) |
| 振子角度 | Roller485 Lite | 0x65 | エンコーダモード (4) |

どちらも FIRE の Port A (SDA 21 / SCL 22, 400 kHz) に接続。

## モデル

状態 `x = [p, th, v, dth]`、入力 `u` = 台車速度指令 [m/s]、制御則 `u = -K x`。

- `p`: 台車位置。正の速度指令で台車が進む向きが +
- `th`: 振子の倒立からの角度。+p 側に倒れる向きが +

```
v'   = (u - v) / tau                                   Roller485 の速度ループを1次遅れで近似
th'' = wn^2 sin th - 2 zeta wn th' - (wn^2/g) cos th v'
```

振子の質量・長さ・慣性は `wn^2 = m l g / J` にまとまる。なので、振子をぶら下げて自由振動させれば
`wn` と `zeta` が同定できる。実機の寸法を測る必要はない。

## セットアップ

```bash
uv sync                                   # Python 依存
uvx platformio run -d firmware -t upload  # ファームウェア書き込み
```

ポートは Linux なら `/dev/ttyUSB0` など。以下 `PORT` と書く。

## 手順

### 1. 接続確認・モード確認

```bash
uv run pend send PORT INFO GET
```

`INFO` は各 Roller の fw / mode / 入力電圧 / 位置などを表示する。

- 0x65 が見つからない場合は、振子側の Roller **だけ**をつないで `SETADDR 0x64 0x65` を送る。
- **昔どのモードで使っていたか**: モードは `0x01` レジスタに入っている (1=速度, 2=位置, 3=電流, 4=エンコーダ)。
  ただしこのファームウェアは起動時に速度/エンコーダモードに設定し直すので、起動後の INFO は常に speed/encoder を表示する。
  旧設定を見たい場合は、別のファームウェアで `0x01` を読むこと。ただ、以前のコードも起動時にモードを設定していたはずなので、
  Roller 側に残っている値は当てにならない。このリポジトリでは速度モードを前提にしている。

### 2. 符号と原点合わせ

1. 振子をまっすぐぶら下げて静止させ、FIRE の **C ボタン**（または `ZERO`）で原点を取る。
2. LCD を見ながら符号を確認する:
   - 台車を手で転がし、`p` が増える向き (+p) を確認する。速度指令と位置の読み出しは同じモーターなので、
     台車側の符号は自動的に一致する（`SGN` の第1引数は +p の向きを反転させるだけ）。
   - 振子を +p 側に倒したとき `hang` が増えれば OK。逆なら `pend send PORT "SGN 1 -1"` を送る。**ここを間違えると絶対に立たない。**
3. `pend send PORT SAVE` で FIRE のフラッシュに保存する（電源を切っても残る）。

### 3. 同定

```bash
# 振子: ぶら下げた状態で軽く揺らしてから（振幅 ~10°以内）
uv run pend log PORT -d 10 -k swing
uv run pend fit-swing logs/<file>_swing.csv --update

# 台車: 振子を外すかぶら下げたまま、床の上で速度ステップ
uv run pend log PORT -d 3 -k step -c "STEP 0.2 1.0"
uv run pend fit-step logs/<file>_step.csv --update
```

`--update` を付けると `params.toml` の `pend_wn` / `pend_zeta` / `cart_tau` を書き換える。

### 4. ゲイン設計と送信

```bash
uv run pend design                       # K, 極, 非線形シミュレーション
uv run pend design --send PORT --save    # FIRE に K/TF/UMAX/R を送って保存
```

チューニングは `params.toml` の `[lqr] q, r` を変更して行う。重みの順序は `[p, th, v, dth]`。

### 5. 倒立

```bash
uv run pend run PORT -d 20      # ARM → 手で振子を立てると自動で制御開始
uv run pend plot logs/<file>_run.csv
```

- `|th| > THLIM`、`|p| > PLIM`、または I2C エラーで自動停止する。
- **B ボタン** / `STOP` でいつでも停止できる。

## シリアルコマンド (921600 baud)

| コマンド | 内容 |
|---|---|
| `K k1 k2 k3 k4` | フィードバックゲイン (`u = -K x`) |
| `ARM` / `STOP` | 倒立待機 (`|th| < ARMW` で開始) / 停止 |
| `STEP u dur` | 速度 `u` [m/s] を `dur` [s] 出す開ループ試験 |
| `LOG 0/1` | `D,t_ms,state,p,th,v,dth,u,exec_us` を毎周期出力 |
| `ZERO` | 現在の振子角をぶら下がり原点にする (C ボタンと同じ) |
| `SGN c p` | 台車/振子の符号 (±1) |
| `R` `TRIM` `TF` `UMAX` `THLIM` `PLIM` `ARMW` `IMAX` | 車輪半径[m], 倒立オフセット[deg], 微分フィルタ[s], 速度上限[m/s], 角度/位置リミット, 開始窓[deg], 最大電流[mA] |
| `GET` / `SAVE` | パラメータ表示 / フラッシュ保存 |
| `INFO` | Roller の状態表示 |
| `SETADDR old new` | Roller の I2C アドレス変更（変更する1台だけをつなぐこと） |
| `SETUP` | Roller のモードを再設定 |

制御周期は `firmware/src/main.cpp` の `kDtUs`（既定 10 ms）。変更したら `params.toml` の `dt` も合わせること。

## 実機で要確認の点

- エンコーダモードで振子角を `0x90` (位置読み出し) から読んでいる。INFO で振子を回しても `pos` が変わらず `dial` が変わる場合は、
  `main.cpp` の `kPendPosReg` を `roller::kDialCounter` に変更する（単位も確認すること）。
- Roller485 の速度ループの PID は既定値のまま使っている。`fit-step` で `cart_tau` が大きい、または振動的な場合は見直す。
