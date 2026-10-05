# URDF 文件

`ur5_umi_real.urdf` 保留 UR5 六轴链，并在 `tool0` 下固定 `mount_link`。`mount_link` 承载已确认安装的 mount；`fixed_mount_link` 承载 Part 1–6。左右 holder 分别在独立的 `left_jaw_link`、`right_jaw_link` 下，并与对应 soft finger 保持固定相对位姿。finger visual origin 取自对应 holder-finger Assembly STEP 的 occurrence transforms。

`ur5_to_mount` 保留当前已安装姿态。`q=0` 时两 jaw link 与 Part Studio 全局坐标重合；左 jaw 沿 `-x`、右 jaw 沿 `+x` 独立平移。finger 网格已转换到 Assembly 产品局部坐标，因此各 finger visual origin 可直接使用 `holder → finger` 变换。完整数值与网格来源见上级 `README.md`。
