# Voidchair v1.0

`voidchair.nbt` 飞行椅蓝图及配套 ComputerCraft 飞控。使用 Create Flight Control 的刚体冲量 API 控制速度，通过悬停功能补偿重力；支持手动驾驶和自动方形螺旋跑图。

## 安装

依赖 CC:Tweaked、CC:Sable、Create Flight Control（需提供 `createfc` 冲量与状态 API）、Drive-By-Sable，以及用于仪表盘的 Create Connected。

1. 导入 `voidchair.nbt`，在网络备份盘中执行一次“加载 / Load”，恢复电缆。
2. 在飞行器电脑上运行：

```lua
wget https://raw.githubusercontent.com/chidayuyu-stack/cc/master/voidchair/install.lua install
install
```

安装器下载所有飞控模块并执行只读自检，然后启动程序。手动安装也可以把本目录的 `flight` 复制到电脑根目录。需要开机自动启动时，把 `startup.lua` 复制到电脑根目录；NBT 本身不包含 Lua 程序。安装器不会覆盖任务设置和进度存档。

首次设置自动跑图：按 Ctrl+T 停止程序，执行：

```text
flight/setup.lua
flight/pilot.lua
```

输入正方形区域两个对角点的世界 X/Z、模拟距离（1～32 区块的半径）和巡航高度。巡航高度没有原版建筑高度的固定限制，可以设置为 560 等高于 500 的 Y 值。

## 操作

| 键位 | 功能 |
| --- | --- |
| W / S / A / D | 前后左右移动 |
| Q / E | 转向，最高 90 度/秒 |
| R / F | 上升 / 下降，最高 18 m/s |
| Shift / Ctrl | 升挡 / 降挡 |
| Space | 切换虚拟力总开关 |
| Z | 开始新跑图任务 |
| X | 暂停 / 恢复任务 |
| Ctrl+T | 停止程序并保存已有任务 |

Z 启动后自动开启虚拟力，先升降至巡航高度，再以最高 340 m/s 飞到区域中心，随后以 80 m/s 的目标速度从中心向外方形螺旋扫描。笔刷半径为模拟距离乘 16 格，路径间距最多为笔刷直径的 90%，外圈补齐边界。

X 暂停时立即刹停，随后可手动飞行。再次 X 会先升降到巡航高度，再高速返回原断点的 X/Z，继续未完成路段。在返回途中再次暂停不会覆盖原扫描断点。完成后悬停。

## 电缆与显示

电缆输入通道依次为：1～4 W/S/A/D，5～6 Q/E，7～8 R/F，9 Shift，10 Ctrl，11 Space，12 Z，13 X。左右 Shift/Ctrl 共用挡位通道，总计 15 条连接。

Create 显示连接器向 `create_connected:dashboard` 写入四行，每行最多 15 字符：

```text
V080.0 F:ON
Y0560 G02/11
SCAN 3/17
P025% H560 R08
```

V 为实际速度（m/s），F 为虚拟力，Y 为当前高度，G 为挡位。第三行为任务阶段；P 为扫描路线进度，H 为巡航高度，R 为模拟距离的区块半径。暂停后的手动移动和返回断点不会增加扫描进度。

## 数据保存和服务器重启

任务数据保存在服务器世界存档的电脑目录中：

```text
<世界存档>/computercraft/computer/<电脑编号>/flight/mission.txt
<世界存档>/computercraft/computer/<电脑编号>/flight/survey-progress.0
<世界存档>/computercraft/computer/<电脑编号>/flight/survey-progress.1
```

`mission.txt` 保存输入的区域边界坐标、模拟距离和巡航高度。两份 `survey-progress` 交替保存完整原路线、下一个未完成航点、任务阶段、转角等待余量和中断位置。它们是持久文件，保留同一世界存档与电脑编号时，服务器重启仍能读取。

扫描期间和暂停时默认每 2 秒保存；暂停、路段完成、阶段切换和正常退出也会保存。重新启动飞控后先悬停，按 X 恢复。自动加载需要电脑启动并执行 `startup.lua`；也可手动运行 `flight/pilot.lua`。新输入的任务参数不会替换已有未完成任务的原路线或高度。

异常断电、强杀或单份存档损坏回退时，只能恢复最后成功写入的进度，当前未保存路段可能回退约 2 秒；已确认保存的路段正常续接。备份服务器时应包含整个世界存档下的 `computercraft` 目录。迁移到新编号电脑时，需要一并复制上述文件。

运行数据已加入 `.gitignore`，不随仓库或安装器发布。不要在需要续扫时删除两份进度文件。更多细节见 [flight/README.md](flight/README.md)。

## 验证

```text
python flight/tests/run_tests.py
python flight/tests/test_install.py
```

需要 Python 与 `lupa.lua52`。测试使用内存文件系统和模拟刚体，涵盖扫描覆盖、速度上限、自定义巡航高度、暂停恢复、存档回退和四行仪表盘。安装器测试检查所有模块下载、下载失败时保留旧程序，以及升级保留运行数据。尚未游戏内实飞验证。
