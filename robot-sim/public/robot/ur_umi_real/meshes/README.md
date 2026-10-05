# 网格文件

`mount-partstudio.step` 的 9 个 Parts 各自导出为 STL，坐标共用 Part Studio 全局坐标系，毫米坐标乘以 `0.001` 转成米。网格不单独居中；导出精度和 Part 对照表见上级 `README.md`。

`mount.stl` 和 `part_1.stl` 至 `part_6.stl` 保持 mount Part Studio 坐标。`finger_holder_left.stl`、`finger_holder_right.stl` 也保留该全局坐标，并分别随对应 jaw link 运动。

`left_finger.stl`、`right_finger.stl` 使用各自 holder-finger Assembly 中 finger 产品的局部坐标，以便 URDF visual origin 直接采用 STEP 实测的 holder-to-finger 位姿。两者从 `cad-input/phase3c1/umi-single-finger.stl` 派生；右侧仅做毫米到米转换，左侧关于局部 `x=0` 镜像并反转三角面绕序。源 CAD 输入不变。`gopro.stl` 当前 URDF 不引用。
