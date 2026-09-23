## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run
- Origin Date: 2026-09-23
- Verification Status: VERIFIED for execution and artifact integrity; no broad generalization claim
- Version Label: random20_result_v1

# 固定轨迹随机布局测试

## 执行结果

- 原布局对照：通过，首次成功第 986/1000 步；3997 个采样时刻，0 个含碰撞事件的时刻。
- 对照最终三个方块的位置，与冻结版本的欧氏距离差（米）：`[0.0, 0.0, 0.0]`。
- 随机布局：尝试 20 个，其中 20 个初始化有效；通过 0/20（全部尝试），0/20（有效布局）。
- 完整执行到第 1000 步的随机试验：0 个。
- 总实验进程耗时：171.8 秒。
- 冻结文件哈希保持不变：True。

## 实验边界

种子 0–19；Python Random(seed)；按 cube_0、cube_1、cube_2、broom_driver、broom_shovel 顺序，独立均匀采样 X/Y ±2 cm。高度与朝向保持不变。不重新规划、不调参、不补跑失败种子。
抓取、交接、持簸箕使用事先确认的 2 mm / 1° 对准门槛。拾取/交接未通过时在绑定前终止，防止预设约束将物体瞬移到手上；持簸箕在预定到位时检查，原基线中的簸箕仍为静态物体。这是运动学评估门槛，不是力控抓取的真实成功率。
已完成的回放按 10 ms 进行独立机器人网格碰撞审计；提前终止的试验只审计已执行前缀，不能称为完整 3997 点无碰撞。碰撞前缀与抓取失败可同时发生，不是互斥分类。
夹持误差使用 max_attachment_translation_error_m 命名，表示绑定变换的位置一致性，不代表指尖接触间隙。

## 失败分类

- `missed_pan_hold`：1 个。
- `missed_pickup`：19 个。
- 已执行前缀中检出机器人碰撞的随机试验：18 个（与上述类别可重叠）。
- 未通过拾取门槛的平移偏差范围：10.247–23.694 mm；门槛 2 mm。

## 每个种子的记录

| Seed | 初始化有效 | 执行到帧 | 终止原因 | 拾取偏差 mm | 碰撞采样数 / 含碰撞时刻数 |
|---:|:---:|---:|---|---:|---:|
| 0 | True | 64 | missed_pickup | 13.812 | 253 / 40 |
| 1 | True | 64 | missed_pickup | 13.044 | 253 / 4 |
| 2 | True | 64 | missed_pickup | 10.247 | 253 / 24 |
| 3 | True | 64 | missed_pickup | 23.694 | 253 / 59 |
| 4 | True | 64 | missed_pickup | 20.590 | 253 / 6 |
| 5 | True | 64 | missed_pickup | 18.890 | 253 / 53 |
| 6 | True | 64 | missed_pickup | 10.457 | 253 / 26 |
| 7 | True | 64 | missed_pickup | 17.682 | 253 / 53 |
| 8 | True | 64 | missed_pickup | 23.103 | 253 / 62 |
| 9 | True | 64 | missed_pickup | 23.129 | 253 / 62 |
| 10 | True | 64 | missed_pickup | 14.913 | 253 / 38 |
| 11 | True | 64 | missed_pickup | 12.623 | 253 / 39 |
| 12 | True | 64 | missed_pickup | 15.356 | 253 / 54 |
| 13 | True | 64 | missed_pickup | 17.890 | 253 / 14 |
| 14 | True | 64 | missed_pickup | 11.306 | 253 / 0 |
| 15 | True | 64 | missed_pickup | 16.824 | 253 / 13 |
| 16 | True | 64 | missed_pickup | 11.054 | 253 / 42 |
| 17 | True | 64 | missed_pickup | 16.877 | 253 / 44 |
| 18 | True | 269 | missed_pan_hold | 1.853 | 1073 / 14 |
| 19 | True | 64 | missed_pickup | 16.399 | 253 / 0 |

## 可以与不可以据此判断的内容

这是固定开环轨迹在这组位置扰动下的描述性结果，不是原版 RoboDojo 评测，也没有测试场景条件化规划器或 LLM policy。
如果试验集中失败在拾取阶段，证据只能定位到开环拾取缺少位置适应；不能据此判断后续清扫策略在成功拾取后的泛化能力，更不能断言 Code-as-Policy 一定会改善。
只运行一个固定对照和 20 个随机布局，没有针对这些种子调整参数，没有显著性检验，也不作跨任务或实机推断。
初始预检查曾错误使用旋转包围盒判断穿桌，已在正式批次前改为实际网格顶点。原预检查、修正后预检查及两个 manifest 均保留；正式试验全部使用 manifest_v2.json，随机数不变。
输入基线、模型、脚本均未改动；新增测试框架和结果独立记录在 experiments/random20，不覆盖冻结基线。

## 文件入口

- [机器汇总](results/summary.json)、[逐项 CSV](results/summary.csv)
- [固定输入与哈希](results/manifest_v2.json)、[进程记录](results/processes.json)
- 各次试验的 result.json、trace.json、collision_events.json 位于 results/control 和 results/seed_00..seed_19。
