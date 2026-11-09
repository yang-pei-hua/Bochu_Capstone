# 二维轮廓拟合

输入同一二维坐标系下的无序采样点，输出一个或多个轮廓。每个轮廓包含一个外环和若干内环。输出为闭合折线，坐标单位与输入一致。外环逆时针，内环顺时针，首点在末尾省略。

模块依赖 CGAL，支持独立构建。C++ 调用入口是 `contour_fitting::fitContours`，命令行入口是 `fit_contour`。

## 构建

在仓库根目录运行。使用 vcpkg 时，按本机安装位置指定工具链。

```powershell
cmake -S core/geometric_reconstruction/src/contour_fitting -B build/contour-fitting-standalone -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE=D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build/contour-fitting-standalone --config Release
ctest --test-dir build/contour-fitting-standalone -C Release --output-on-failure
```

## 输入和输出

`points.csv` 每行一个 `x,y` 坐标，可选首行表头 `x,y`。

```powershell
& build/contour-fitting-standalone/Release/fit_contour.exe points.csv contours.json
& build/contour-fitting-standalone/Release/fit_contour.exe points.csv contours.json 3.0 0.3
```

末尾两个参数分别是 **Alpha 半径**和**简化容差**，单位与输入一致。半径控制可以跨过多大的采样间隙，设为 0 时根据点间距估计。省略简化容差时取半径的十分之一，显式设为 0 时保留原始边界顶点。

JSON 的 `contours` 数组中，每项形如 `{"outer":[[0,0],[4,0],[4,4],[0,4]],"holes":[]}`，并记录实际使用的半径、容差与去重后的点数。退出码 0 表示输出了轮廓，2 表示没有闭合轮廓且仍会输出空数组，1 表示输入或文件读写错误。

## 尺度与采样

轮廓由保留的 Delaunay 三角形边界组成。较小的半径保留窄凹口，但可能使稀疏采样区域断开。较大的半径更容易跨过间隙，也可能填掉小孔。自动尺度适合作为首次试算的起点，稀疏或密度差异大的输入可以显式指定半径。输入需要有足够的面内采样来支撑区域。仅有零散边缘点或遮挡造成的空白时，输出仍需结合观测判断。

简化时对所有轮廓一起约束，保留外环和内环的拓扑关系。[CGAL 折线简化文档](https://doc.cgal.org/latest/Polyline_simplification_2/)

C++ 调用方可在 `Options` 中设置 `minimumRegionPoints` 和 `minimumRegionFraction`。每个外环须同时满足两项下界。点数统计保留三角形的连通区域内不同的顶点，占比的分母是该次投影中所有不同的二维点。内孔按保留下来的区域边界生成。两个默认值均为 0。
