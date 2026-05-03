# tests/scenarios/

テスト fixture を置く。各シナリオは次の二点で構成する:

- `<id>/scenario.yaml` — 入力（マップ・初期条件・操作列・期待アサーション）
- `<id>/expected.json` — 旧コードを駆動して固定した正解値（path, twist 列, goal_reached 遷移など）

YAML は C++（yaml-cpp / nlohmann_json）と Python（PyYAML / json）の両方から読まれる。
スキーマと運用は `docs/contributing/testing.md` を参照。

## シナリオ一覧（実装は M0 で追加）

| ID | 概要 |
|---|---|
| S-01 | XY 到達後の stateful フラグ非反転 |
| S-02 | 同一ゴール再投入で再到達 |
| S-03 | 走行中の新ゴール差し込みで履歴が引き摺らない |
| S-04 | cancel 後の再ゴール |
| S-05 | 同一入力での A\* 決定性 |
| S-06 | setPlan 高頻度発火でメモリ・index 健全性 |
| S-07 | path 末尾近傍で cancel → 別ゴール |
| S-08 | 到達不能ゴールで NoPathFound |
| S-09 | 空 path で computeCommand |
| S-10 | MPPI 再現性（CasADi ON/OFF 一致） |
| S-11 | HeightProvider null 安全 |
| S-12 | コの字を走破直後の折り返し |
| S-13 | path smoothing 後の曲率連続性 |
| S-14 | 実機相当 (制御遅延 / 量子化 / 加速度上限) |
| S-15 | 数値の切れ目 (zero-length, tolerance ちょうど, large origin, 重複 pose) |
| S-16 | localization 震え（ガウスノイズ） |
| S-17 | path 末尾近傍での再震えで stateful 維持 |
