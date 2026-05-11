# genesis-sand-water-walker

[Genesis](https://github.com/Genesis-Embodied-AI/Genesis) を土台にした、
変形地形上のロコモーション実験プロジェクトです。Genesis の MPM ↔ 剛体カップリングを
使って humanoid を砂プールに、planar walker を水プールに落下させるシーンと、
walker を剛体床上でその場足踏みさせる初期コントローラを収録しています。
後者は砂・水の上での歩行へ向けた最初のステップです。

長期目標は **砂や水の上で立位を保ちながら歩行する制御則の構築**です。

English version: [README.md](../README.md)

## デモ

各スクリプトは MP4 動画と胴体軌跡の per-step CSV を `outputs/` に出力します。

| シナリオ | スクリプト | 出力 |
| --- | --- | --- |
| Humanoid → 深さ 0.75 m の砂、約 3.6 m 落下 | `scripts/humanoid_on_sand.py` | `outputs/humanoid_on_sand.{mp4,csv}` |
| Planar walker → 深さ 0.75 m の水、約 3.2 m 落下 | `scripts/walker_on_water.py` | `outputs/walker_on_water.{mp4,csv}` |
| Planar walker が剛体床でその場足踏み | `scripts/walker_marching.py` | `outputs/walker_marching.{mp4,csv}` |

## 動作要件

- Linux + NVIDIA GPU (RTX 5080 で動作確認)
- Docker + [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
  (`docker run --gpus all` が通る状態)
- `make`

Python / CUDA 依存はすべて Docker イメージ内に隔離されており、ホスト側 Python の
セットアップは不要です。

## クイックスタート

```bash
make help                # 利用可能なターゲット一覧
make build               # Docker イメージのビルド (初回のみ、~30 分: LuisaRender をコンパイル)
make check-gpu           # 任意: Docker から GPU が見えるか確認
make dive-humanoid       # humanoid を砂へ落下 → outputs/humanoid_on_sand.mp4
make dive-walker         # walker を水へ落下 → outputs/walker_on_water.mp4
make dive-all            # 上記2つを連続実行
make march-walker        # walker が剛体床でその場足踏み (~1 分)
```

足踏みの周波数と足の持ち上げ高さはコマンドラインで上書きできます:

```bash
make march-walker GAIT_HZ=2.0                      # 2 Hz に変更
make march-walker KNEE_AMPLITUDE=0.9               # より高く持ち上げる
make march-walker GAIT_HZ=2.0 KNEE_AMPLITUDE=0.9   # 両方
```

各 dive は物理シミュレーション 2.4 秒分 (600 ステップ、`dt = 4 ms`、
MPM ↔ 剛体カップリング安定化のため `substeps=25`) を回し、RTX 5080 で実時間
**5〜7 分**程度かかります。律速は MPM ソルバで、レンダラではありません。
`make march-walker` は MPM を使わないため約 1 分で完了します。

スクリプトを書き換えながら反復したい場合は `make shell` で同じコンテナへ
対話的に入れます。

## リポジトリ構成

```
genesis-sand-water-walker/
├── Makefile               # すべてのエントリポイント (`make help` で一覧)
├── docker/
│   ├── Dockerfile         # CUDA 12.8 + PyTorch 2.11 + Genesis + LuisaRender
│   ├── build_luisa.sh
│   └── *.json             # NVIDIA EGL/Vulkan ICD 記述子
├── scripts/
│   ├── humanoid_on_sand.py
│   ├── walker_on_water.py
│   └── walker_marching.py # 剛体床上での adaptive PID 足踏み
├── assets/
│   ├── humanoid_no_floor.xml   # Genesis 同梱モデルの MJCF コピー
│   └── walker_no_floor.xml     # <worldbody> の floor plane を削除済み
├── readme/
│   └── README.ja.md       # 本ファイル
└── outputs/               # 生成された動画と CSV (gitignored)
```

## なぜカスタム MJCF か

`assets/humanoid_no_floor.xml` と `assets/walker_no_floor.xml` は Genesis
同梱の `genesis/assets/xml/humanoid.xml` / `walker.xml` から `<geom name="floor">`
プレーンを削除した派生です。

このプレーンは parse 後の MJCF エンティティに含まれます。初期位置オフセット付き
で読み込む (`gs.morphs.MJCF(file=..., pos=(0,0,h))`) とオフセットは
**全リンク** (床プレーンも含む) に適用されてしまい、ロボットは自前の床の上に
立ったままで重力が効きません。プレーンを削除することで本来の重力挙動が得られます。

確認したい場合は、スクリプトで stock の MJCF パスを指定してみてください。
胴体 z 座標は固定され、砂や水も乱れません。

## 足踏みコントローラ (walker_marching.py)

walker は平面拘束モデルで、駆動されない root 3 DoF (`rootz` スライド、`rootx`
スライド、`rooty` ヒンジ) と、駆動される 6 関節 (左右の hip / knee / ankle) を
持ちます。コントローラは小さな PID で、各アクチュエータに役割を明示的に
振り分けています。

| 制御チャネル | アクチュエータ | 項 | 目的 |
| --- | --- | --- | --- |
| スイング | hip (左右非対称) | I (歩境界で更新) | 振り脚を前方に持ち上げる。cadence 変化を integrator が学習 |
| ピッチ安定化 | hip (左右対称) | P + D | `rooty` を 0 付近に保つ |
| x 位置安定化 | ankle (左右対称) | P + D | 接地足の水平方向反力で `rootx` を 0 付近に保つ |

ピッチを hip トルク、x を ankle トルクへ分離するのが鍵です。walker の足取り付け
位置が hip 軸から +0.06 m 前方にあるため、hip だけで両方を制御しようとすると
両者がカップルし、前方ドリフトか後方転倒のどちらかに帰着します。

ベースライン (`GAIT_HZ=1.0`、`KNEE_AMPLITUDE=0.6`) では 0.3 秒の settle 後
2.1 秒の足踏みで `|x| ≤ 2 cm`、`|pitch| ≤ 1°` を維持します。cadence を倍速に
しても integrator が自動適応し、再チューニング不要です。一方、knee 振幅を
0.8 rad より大きくすると swing 中の前方推進が増え、現状の ankle ゲインでは
追従できず前方ドリフトが残ります (lift と ankle ゲインを比例させる改修が
将来必要)。

## シミュレーションパラメータ

各 dive スクリプトの先頭に主要なパラメータが集まっています。

| パラメータ | 場所 | 意味 |
| --- | --- | --- |
| `sim_options.dt`, `substeps` | `gs.options.SimOptions(...)` | 剛体ソルバの時間刻み。`substeps=25` は 3 m 落下時の接触インパルスを安定させる値。`nan` が出るときは増やす |
| `mpm_options.grid_density` | `gs.options.MPMOptions(...)` | MPM グリッド解像度 (単位長あたりセル数)。40 は画質と実行時間のバランス |
| プール深さ | `gs.morphs.Box(size=(..., ..., depth), ...)` | 現状 0.75 m (humanoid の身長の半分強) |
| 落下高さ | `gs.morphs.MJCF(..., pos=(0, 0, h))` | humanoid は 2.30 m (胴体は ~3.6 m から落下)、walker は 1.90 m |
| `needs_coup`, `coup_friction` | `gs.materials.Rigid(...)` | MPM ↔ 剛体カップリング。`needs_coup` を切るとロボットは砂や水をすり抜ける |

## ロードマップ

- [x] humanoid を深い砂プールへ落下 (シーンとカップリングを検証済み)
- [x] planar walker を深い水プールへ落下 (シーンとカップリングを検証済み)
- [x] walker が剛体床でその場足踏み (adaptive PID、`make march-walker`)
- [ ] walker が砂でその場足踏み
- [ ] walker が水でその場足踏み
- [ ] 剛体床での前進歩行、その後砂・水へ転移
- [ ] プール深さ・粒子密度・摩擦のドメインランダム化
- [ ] Sim-to-real 転移メモ

## ライセンス

Apache License 2.0 のもとで配布されます。詳細は `LICENSE` を参照してください。
`assets/*_no_floor.xml` は Genesis に同梱されている
[DeepMind Control Suite](https://github.com/google-deepmind/dm_control) モデル
(Apache-2.0) の派生です。

Docker イメージはビルド時に upstream の Genesis を
<https://github.com/Genesis-Embodied-AI/Genesis> から取得します。Genesis 自身の
ライセンスはそちらを参照してください。
