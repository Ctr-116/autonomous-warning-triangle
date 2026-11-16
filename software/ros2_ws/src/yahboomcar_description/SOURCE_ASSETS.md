# 原始资产来源与完整性说明 (SOURCE_ASSETS.md)

本文件详细记录 `yahboomcar_description` 功能包内所有原始资产的来源、保留策略以及数据规范。

---

## 1. 资产来源 (Source Provenance)

- **数据源**：用户提供的原工程归档压缩包 `yahboomcar_ws.zip` 内的 `yahboomcar_description` 目录。
- **原始维护者**：`yahboom <yahboom@todo.todo>`
- **原始许可证**：`TODO`（原始许可条款未公开，故保留该标记，不随意篡改为 MIT 或其他开源协议）。

---

## 2. 3D 网格模型文件清单与完整性 (Mesh Assets)

原功能包 `meshes/` 目录下的 **45 个 STL / DAE 网格文件**全部保持原样、逐字节保留（Byte-for-byte preserved），未对几何顶点、缩放比例、法线或文件哈希进行任何修改：

- `meshes/yahboomcar_X3/` (包含底盘支架、相机架、单线激光雷达、Mecanum轮等 STL/DAE 资产)
- `meshes/yahboomcar_X3plus/` (包含 6DOF 机械臂各连杆、末端夹爪、深感相机支架等 STL/DAE 资产)

---

## 3. 标准 URDF 几何与运动学保留 (URDF Geometry)

- **标准模型文件**：
  - `urdf/yahboomcar_X3.urdf`
  - `urdf/yahboomcar_X3plus.urdf`
- **坐标系根节点**：
  - 机器人的基座根坐标系为 `base_footprint`。
- **运动链真实性**：
  - `yahboomcar_X3plus.urdf` 包含完整的 17 个 link 与 16 个 joint（含机械臂关节与固定连接关节）。
  - **绝不伪造轮子关节状态**：原厂提供的标准 X3plus URDF 模型中，轮子网格直接装配在底盘结构中，并未单独开辟活动的 wheel joint。本次迁移严格遵守原厂真实模型定义，不人为添加虚拟或臆测的轮子关节与动力学参数。

---

## 4. 安装与部署配置 (CMake Installation)

在 `CMakeLists.txt` 中已通过 explicit 规则确保新文档与原有资产同步安装至 ROS 2 的 share 目录：

- `MIGRATION.md` 与 `SOURCE_ASSETS.md` 安装至 `share/yahboomcar_description/`
- 原 `meshes/`、`urdf/`、`rviz/`、`launch/` 安装至 `share/yahboomcar_description/`
- 明确排除了 `.ipynb_checkpoints` 及 `*bak*` 临时备份文件。
