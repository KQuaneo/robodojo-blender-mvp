"""Describe collected results without fitting, tuning, or rerunning trials."""
import csv,hashlib,json,math
from collections import Counter
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1];OUT=HERE/'results'
manifest=json.loads((OUT/'manifest_v2.json').read_text())
rows=[json.loads((OUT/t['id']/'result.json').read_text()) for t in manifest['trials']]
control=rows[0];random=rows[1:];summary=json.loads((OUT/'summary.json').read_text())
reference=json.loads((ROOT/'output_smooth/trace.json').read_text())[-1]['positions']
rerun=json.loads((OUT/'control/trace.json').read_text())[-1]['positions']
errors=[math.dist(a,b) for a,b in zip(reference,rerun)]
counts=Counter(r['termination_reason'] for r in random)
valid=[r for r in random if r['initialization_valid']]
missed=[r for r in random if r['termination_reason']=='missed_pickup']
collision_prefix=[r for r in random if r['collision_events']>0]
gap=[r['grasp_gates'][0]['translation_m']*1000 for r in missed]
report=['## Material Passport','','- Origin Skill: academic-research-suite / experiment-agent','- Origin Mode: run','- Origin Date: 2026-09-23','- Verification Status: VERIFIED for execution and artifact integrity; no broad generalization claim','- Version Label: random20_result_v1','','# 固定轨迹随机布局测试','','## 执行结果','',
f'- 原布局对照：{"通过" if control["qualified"] else "未通过"}，首次成功第 {control["first_success_frame"]}/1000 步；{control["collision_samples"]} 个采样时刻，{control["collision_events"]} 个含碰撞事件的时刻。',
f'- 对照最终三个方块的位置，与冻结版本的欧氏距离差（米）：`{errors}`。',
f'- 随机布局：尝试 20 个，其中 {len(valid)} 个初始化有效；通过 {sum(r["qualified"] for r in random)}/20（全部尝试），{sum(r["qualified"] for r in valid)}/{len(valid)}（有效布局）。',
f'- 完整执行到第 1000 步的随机试验：{sum(r["full_episode_audited"] for r in random)} 个。',
f'- 总实验进程耗时：{summary["total_process_seconds"]:.1f} 秒。',
f'- 冻结文件哈希保持不变：{summary["baseline_files_unchanged"]}。','','## 实验边界','',
'种子 0–19；Python Random(seed)；按 cube_0、cube_1、cube_2、broom_driver、broom_shovel 顺序，独立均匀采样 X/Y ±2 cm。高度与朝向保持不变。不重新规划、不调参、不补跑失败种子。',
'抓取、交接、持簸箕使用事先确认的 2 mm / 1° 对准门槛。拾取/交接未通过时在绑定前终止，防止预设约束将物体瞬移到手上；持簸箕在预定到位时检查，原基线中的簸箕仍为静态物体。这是运动学评估门槛，不是力控抓取的真实成功率。',
'已完成的回放按 10 ms 进行独立机器人网格碰撞审计；提前终止的试验只审计已执行前缀，不能称为完整 3997 点无碰撞。碰撞前缀与抓取失败可同时发生，不是互斥分类。',
'夹持误差使用 max_attachment_translation_error_m 命名，表示绑定变换的位置一致性，不代表指尖接触间隙。','','## 失败分类','']
report += [f'- `{key}`：{value} 个。' for key,value in sorted(counts.items())]
report += [f'- 已执行前缀中检出机器人碰撞的随机试验：{len(collision_prefix)} 个（与上述类别可重叠）。']
if gap:report += [f'- 未通过拾取门槛的平移偏差范围：{min(gap):.3f}–{max(gap):.3f} mm；门槛 2 mm。']
report += ['','## 每个种子的记录','','| Seed | 初始化有效 | 执行到帧 | 终止原因 | 拾取偏差 mm | 碰撞采样数 / 含碰撞时刻数 |','|---:|:---:|---:|---|---:|---:|']
for r in random:
    g=next((g for g in r['grasp_gates'] if g['name']=='pickup'),None)
    text=f'{g["translation_m"]*1000:.3f}' if g else '未执行'
    report.append(f'| {r["seed"]} | {r["initialization_valid"]} | {r["executed_through_frame"]} | {r["termination_reason"]} | {text} | {r["collision_samples"]} / {r["collision_events"]} |')
report += ['','## 可以与不可以据此判断的内容','',
'这是固定开环轨迹在这组位置扰动下的描述性结果，不是原版 RoboDojo 评测，也没有测试场景条件化规划器或 LLM policy。',
'如果试验集中失败在拾取阶段，证据只能定位到开环拾取缺少位置适应；不能据此判断后续清扫策略在成功拾取后的泛化能力，更不能断言 Code-as-Policy 一定会改善。',
'只运行一个固定对照和 20 个随机布局，没有针对这些种子调整参数，没有显著性检验，也不作跨任务或实机推断。',
'初始预检查曾错误使用旋转包围盒判断穿桌，已在正式批次前改为实际网格顶点。原预检查、修正后预检查及两个 manifest 均保留；正式试验全部使用 manifest_v2.json，随机数不变。',
'输入基线、模型、脚本均未改动；新增测试框架和结果独立记录在 experiments/random20，不覆盖冻结基线。','','## 文件入口','',
'- [机器汇总](results/summary.json)、[逐项 CSV](results/summary.csv)',
'- [固定输入与哈希](results/manifest_v2.json)、[进程记录](results/processes.json)',
'- 各次试验的 result.json、trace.json、collision_events.json 位于 results/control 和 results/seed_00..seed_19。','']
(HERE/'REPORT.md').write_text('\n'.join(report))
inventory={str(p.relative_to(HERE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in OUT.rglob('*') if p.is_file() and p.name!='SHA256.json'}
(OUT/'SHA256.json').write_text(json.dumps(inventory,indent=2))
print(json.dumps({'summary':summary,'control_final_position_errors_m':errors,'report':str(HERE/'REPORT.md')},indent=2))
