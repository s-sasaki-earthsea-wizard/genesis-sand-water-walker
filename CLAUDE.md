# genesis-sand-water-walker

## プロジェクト概要

[Genesis](https://github.com/Genesis-Embodied-AI/Genesis) の MPM ↔ 剛体カップリングを
土台にした、変形地形上のロコモーション実験プロジェクト。

ベースラインとして以下2つのダイブシーンを提供する:

- **humanoid** を深さ 0.75 m の砂プールへ約 3.6 m から落下 (`scripts/humanoid_on_sand.py`)
- **planar walker** を深さ 0.75 m の水プールへ約 3.2 m から落下 (`scripts/walker_on_water.py`)

各スクリプトは MP4 動画と胴体軌跡 CSV を `outputs/` に出力する。

### 長期目標

砂や水のような変形媒質と相互作用しながら直立を維持する二足歩行ポリシーの学習。
現状はシーン構築と MPM ↔ 剛体カップリングの妥当性確認までで、コントローラは未学習。

### 実行環境

- Linux + NVIDIA GPU (RTX 5080 で動作確認)
- Docker + NVIDIA Container Toolkit
- Python/CUDA 依存はすべて Docker イメージ内に隔離 (ホスト側 Python セットアップ不要)
- エントリポイントは `Makefile` に集約 (`make help` で一覧)

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