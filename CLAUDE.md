# genesis-sand-water-walker

## プロジェクト概要

[Genesis](https://github.com/Genesis-Embodied-AI/Genesis) の MPM ↔ 剛体カップリングを
土台にした、変形地形上のロコモーション実験プロジェクト。

ベースラインとして以下2つのダイブシーンを提供する:

- **humanoid** を深さ 0.75 m の砂プールへ約 3.6 m から落下 (`scripts/humanoid_on_sand.py`)
- **planar walker** を深さ 0.75 m の水プールへ約 3.2 m から落下 (`scripts/walker_on_water.py`)

さらに、コントローラ付きでその場足踏みさせるシーンを提供する:

- 剛体床上の足踏み (`scripts/walker_marching.py`)
- 膝丈の水プールに足を浸した状態での足踏み (`scripts/walker_marching_in_water.py`)

各スクリプトは MP4 動画と胴体軌跡 CSV を `outputs/` に出力する。

### 長期目標

砂や水のような変形媒質と相互作用しながら直立を維持する二足歩行ポリシーの学習。
現状はシーン構築と MPM ↔ 剛体カップリングの妥当性確認までで、コントローラは未学習。

### 実行環境

- Linux + NVIDIA GPU (RTX 5080 で動作確認)
- Docker + NVIDIA Container Toolkit
- Python/CUDA 依存はすべて Docker イメージ内に隔離 (ホスト側 Python セットアップ不要)
- エントリポイントは `Makefile` に集約 (`make help` で一覧)

### Phase 完了記録

- **Phase 0 — Baseline scenes**: humanoid を砂、walker を水に落下させる
  シーンと MPM ↔ 剛体カップリングを検証 (`scripts/humanoid_on_sand.py`、
  `scripts/walker_on_water.py`)
- **Phase 1 — Walker marching on rigid floor**: 平面 walker を剛体床上で
  その場足踏みさせる adaptive PID コントローラを実装
  (`scripts/walker_marching.py`)。役割分担:
  - swing: hip 非対称、I 項 (歩境界で更新) で cadence 変化を自動適応
  - pitch: hip 対称、P+D
  - x: ankle 対称、P+D
  ベースライン (1 Hz / KNEE_AMPLITUDE=0.6) で 2.4 秒後 `|x| ≤ 2 cm`、
  `|pitch| ≤ 1°`。cadence 倍速は再チューニング不要。knee 振幅 0.8 rad 超は
  前方ドリフトが残存 (Phase 1 の積み残し)。`GAIT_HZ` と `KNEE_AMPLITUDE` は
  CLI/Makefile 変数として外出し済み。
- **Phase 2 — Walker marching in shallow water**:
  Phase 1 のコントローラを土台に、剛体床の上に深さ約 0.5 m (膝高さ) の MPM 水
  プールを重ねた統合シーンを実装 (`scripts/walker_marching_in_water.py`)。
  dry-floor の制御則を水中へ転用する過程で必要だった主要な改修:
  - **Pre-settle phase**: walker を水面の +10 cm 上で spawn し、本ループ前に
    `PRESETTLE_STEPS=250` (約 1.0 s) だけ scene を空回しして MPM の水柱を静水圧
    平衡へ。水ボックス spawn 直後の "ブロック落下" 過渡が脚に衝撃を与える問題
    を切り離す。
  - **Balance during settle**: dry script は SETTLE 中に全関節 target=0 だが、
    水中では浮力・抗力の慢性外乱でその間に walker が傾く。本スクリプトでは
    `pitch_balance` / `x_balance` を `t=0` から常時適用し、swing/knee のみ
    `SETTLE_TIME=1.0 s` までゲート。
  - **強化された x 制御**: 各 swing で水抗力が walker を後方へ押す反作用を相殺
    するため、ankle ゲインを `P_X_ANKLE 1.5→3.0`、`D_X_ANKLE 0.8→1.5`、
    `ANKLE_LIMIT 0.6→0.785` (MJCF の上限) へ。
  - **MPM 境界**: `lower_bound z=-0.10` に拡張 (`grid_density=40` の safety
    padding 約 0.075 m を吸収して水ボックスを z=0 から張れるように)。

  ベースライン (`GAIT_HZ=1.0` / `KNEE_AMPLITUDE=0.6` / `WATER_LEVEL=0.5`、
  `DURATION=2.4 s`) で `|x| ≤ 2.2 cm`、`|pitch| ≤ 1.1°`、`hip_amp_i ≈ 0.16`
  (saturation せず) と、Phase 1 dry-floor と同等の安定性に到達。CLI/Makefile
  変数: `GAIT_HZ` / `KNEE_AMPLITUDE` / `WATER_LEVEL` / `DURATION`。

## 言語設定

このプロジェクトでは**日本語**での応答を行ってください。
コード内のコメント、ログメッセージ、エラーメッセージ、ドキュメントなどtrackされるファイルは**英文**で記述してください (このCLAUDE.mdは唯一の例外とします)。

## 開発ルール

### コーディング規約

- Python: PEP 8準拠
- 関数名: snake_case
- クラス名: PascalCase
- 定数: UPPER_SNAKE_CASE
- Docstring: Google Style

## Git運用

- ブランチ戦略: feature/*, fix/*, refactor/*
- コミットメッセージ: 英文を使用、動詞から始める
- GitHub Flowを採用する
- PRはmainブランチへ

## 開発ガイドライン

### ドキュメント更新プロセス

機能追加やPhase完了時には、以下のドキュメントを同期更新する：

1. **CLAUDE.md**: プロジェクト全体状況、Phase完了記録、技術仕様
2. **README.md (英語、リポジトリ root)** と **readme/README.ja.md (日本語)**:
   ユーザー向け機能概要、実装状況、使用方法。**両言語版を必ず同期更新する**
   (片方だけ更新すると言語切替後に齟齬が発生する)
3. **Makefile**: コマンドヘルプテキスト（## コメント）の更新
4. **makefiles/**: コマンドヘルプテキスト（## コメント）の更新

### コミットメッセージ規約

#### コミット粒度

- **1コミット = 1つの主要な変更**: 複数の独立した機能や修正を1つのコミットにまとめない
- **論理的な単位でコミット**: 関連する変更は1つのコミットにまとめる
- **段階的コミット**: 大きな変更は段階的に分割してコミット

#### プレフィックスと絵文字

- ✨ feat: 新機能
- 🐞 fix: バグ修正
- 📚 docs: ドキュメント
- 🎨 style: コードスタイル修正
- 🛠️ refactor: リファクタリング
- ⚡ perf: パフォーマンス改善
- ✅ test: テスト追加・修正
- 🏗️ chore: ビルド・補助ツール
- 🚀 deploy: デプロイ
- 🔒 security: セキュリティ修正
- 📝 update: 更新・改善
- 🗑️ remove: 削除

**重要**: Claude Codeを使用してコミットする場合は、必ず以下の署名を含める：

```text
🤖 Generated with [Claude Code](https://claude.ai/code)

Co-Authored-By: Claude <noreply@anthropic.com>
```