# URDF 文件

`ur5_umi_real.urdf` 保留现有 UR5 六轴树，并添加 mount、UMI 固定主体与两指关节。网格使用 `package://ur_description/` 和 `package://ur_umi_real/` URI；加载器需要按上级 `README.md` 中的路径映射解析。真实 `mount_to_umi` 仍未知，URDF 内保留显式 identity placeholder。
