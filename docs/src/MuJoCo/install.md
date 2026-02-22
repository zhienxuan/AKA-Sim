# 安装指南

## Linux 安装

## Macos 安装

1. 安装miniconda

要安装miniconda，请按照官方[安装指南](https://docs.anaconda.net.cn/miniconda/install/)

2. 创建干净的 Python 环境

```shell
conda create -n mujoco python=3.10
conda activate mujoco
```

3. 安装 MuJoCo

```shell
pip install mujoco
```

4. 用代码测试

创建 test_mujoco.py 文件
```python
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_string("""
<mujoco>
  <worldbody>
    <body>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
""")

data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

5. 通过mjpython运行

```shell
mjpython test_mujoco.py
```

6. 弹出软件窗口为安装成功

![mac_init](./images/mac_init.png)

## Windows 安装