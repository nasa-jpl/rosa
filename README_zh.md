<!-- 项目头部块 --> <hr>
<div align="center">

  [//]: # "  [INSERT YOUR LOGO IMAGE HERE (IF APPLICABLE)]"

  <h1 align="center">ROSA - ROS Agent 🤖</h1>
</div>
<pre align="center">
  ROS Agent（ROSA）旨在使用自然语言查询与基于ROS的机器人系统进行交互。🗣️🤖
</pre>

<div align="center">

![ROS 1](https://img.shields.io/badge/ROS_1-Noetic-blue)
![ROS 2](https://img.shields.io/badge/ROS_2-Humble|Iron|Jazzy-blue)
![License](https://img.shields.io/pypi/l/jpl-rosa)
[![SLIM](https://img.shields.io/badge/Best%20Practices%20from-SLIM-blue)](https://nasa-ammos.github.io/slim/)

![Main Branch](https://img.shields.io/github/actions/workflow/status/nasa-jpl/rosa/ci.yml?branch=main&label=main)
![Dev Branch](https://img.shields.io/github/actions/workflow/status/nasa-jpl/rosa/ci.yml?branch=dev&label=dev)
![Publish Status](https://img.shields.io/github/actions/workflow/status/nasa-jpl/rosa/publish.yml?label=publish)
![Version](https://img.shields.io/pypi/v/jpl-rosa)
![Downloads](https://img.shields.io/pypi/dw/jpl-rosa)

[English](README.md) | [中文](README_zh.md)
</div>
<!-- Header block for project -->

> [!重要]
> 📚 **第一次使用ROSA？** 请查看我们的[Wiki](https://github.com/nasa-jpl/rosa/wiki)以获取文档、指南和常见问题解答！

ROSA 是您与 ROS1 和 ROS2 系统交互的 AI Agent。基于 [Langchain](https://python.langchain.com/v0.2/docs/introduction/) 框架，ROSA 帮助您通过自然语言与机器人进行交互，使得机器人开发更加便捷高效。

## 🚀 快速上手

### 需求
- Python 3.9+
- ROS Noetic 或更高版本

### 安装
```bash
pip3 install jpl-rosa
```

### 使用示例

```python
from rosa import ROSA

llm = get_your_llm_here()
rosa = ROSA(ros_version=1, llm=llm)
rosa.invoke("Show me a list of topics that have publishers but no subscribers")
```

有关如何配置 LLM 的详细信息，请参阅我们的[模型配置 Wiki 页面](https://github.com/nasa-jpl/rosa/wiki/Model-Configuration)。

### 为您的机器人定制 ROSA 🔧

ROSA 旨在易于适应不同的机器人和环境。您可以通过继承 `ROSA` 类或使用自定义参数创建新实例来创建自定义代理。

有关创建自定义代理、添加工具和自定义提示的详细信息，请参阅我们的[自定义代理 Wiki 页面](https://github.com/nasa-jpl/rosa/wiki/Custom-Agents)。

### TurtleSim 演示 🐢

我们提供了一个使用 ROSA 控制 TurtleSim 机器人的模拟演示。要运行此演示，您需要在机器上安装 [Docker](https://www.docker.com/)。 🐳

以下视频展示了 ROSA 如何推理绘制五角星的步骤，然后执行必要的命令完成绘制。

https://github.com/user-attachments/assets/77b97014-6d2e-4123-8d0b-ea0916d93a4e

有关设置和运行 TurtleSim 演示的详细说明，请参阅我们的[演示指南](https://github.com/nasa-jpl/rosa/wiki/Guide:-TurtleSim-Demo)。

### 📘 了解更多

- [🗺️ 路线图](https://github.com/nasa-jpl/rosa/wiki/Feature-Roadmap)
- [🏷️ 发布版本](https://github.com/nasa-jpl/rosa/releases)
- [❓ 常见问题](https://github.com/nasa-jpl/rosa/wiki/FAQ)

### 更新日志

查看我们的[CHANGELOG.md](CHANGELOG.md)了解我们的更改历史。

### 贡献

有兴趣为我们的项目做出贡献吗？请参阅：[CONTRIBUTING.md](CONTRIBUTING.md)

有关如何与我们团队互动的指南，请查看我们的行为准则：[CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)

有关我们的治理方式，包括决策过程和不同角色的指南，请参阅我们的治理模型：[GOVERNANCE.md](GOVERNANCE.md)

### 许可证

查看：[LICENSE](LICENSE)

### 支持

项目主要联系方式：

- [@RobRoyce](https://github.com/RobRoyce) ([电子邮件](mailto:01-laptop-voiced@icloud.com))

---

<div align="center">
  ROSA: Robot Operating System Agent 🤖<br>
  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
</div>