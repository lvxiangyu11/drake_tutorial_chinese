---
title: markmap
markmap:
  colorFreezeLevel: 3
---

# drake

## 0. install

### use mac or linux or windows with wsl
### install anaconda (optional)
### pip install drake
### git clone 或 deepnote
- [Drake Tutorials on Deepnote](https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305/project/Tutorials-2b4fc509-aef2-417d-a40d-6071dfed9199/notebook/index-753e3c9d261247ba9f0eb1d7868c18c8)
- [drake_tutorial_chinese on GitHub](https://github.com/lvxiangyu11/drake_tutorial_chinese)
- [RoboticManipulationDrake_Chinese on GitHub](https://github.com/lvxiangyu11/RoboticManipulationDrake_Chinese)

## 1. dynamic

## pydrake

### symbolic

#### Variables
- 用于表示符号变量
```python
x = Variable("x")
```

### systems 

#### primitives 
- 用于表示系统的基本构建模块

##### SymbolicVectorSystem
- 用于表示符号向量系统
```python
SymbolicVectorSystem(state=[x], dynamics=[-x + x**3], output=[x]) # 连续时间系统
discrete_vector_system = SymbolicVectorSystem(state=[x], dynamics=[x**3], output=[x], time_period=0.1) # 离散时间系统
```

##### AffineSystem
- 用于表示仿射系统

##### LinearSystem
- 用于表示线性系统

##### LogVectorOutput
- 用于表示记录系统的输出向量
- 记录
```python
logger = LogVectorOutput(system.get_output_port(0), builder) # 记录系统的第0个输出端口
```

``` python
log = logger.FindLog(context) # 获取日志
print(log.sample_times(), log.data().transpose()) # 打印日志
```


#### framework
- 用于表示系统的框架
- SetDiscreteState(0, x)

##### LeafSystem
- 用于表示叶子系统

###### 连续时间系统
```python
# 定义系统, 连续非线性动力系统：dot_x = -x + x^3, y = x
class SimpleContinuousSystem(LeafSystem):
    # 初始化，并定义 状态 和 输出
    def __init__(self):
        LeafSystem.__init__(self)

        state_index = self.DeclareContinuousState(1)    # 定义一个连续状态
        self.DeclareStateOutputPort("y", state_index)  # 定义一个输出端口

    # 定义状态的导数
    def DoCalcTimeDerivatives(self, context, derivatives):
    # 其作为 drake system类的子类，必须实现 DoCalcTimeDerivatives 方法来定义状态的导数
        x = context.get_continuous_state_vector().GetAtIndex(0) # 获取状态
        x_dot = -x + x**3                                       # 定义状态导数
        derivatives.get_mutable_vector().SetAtIndex(0, x_dot)   # 设置状态导数, get_mutable_vector() 获取可变的向量
```

###### 离散时间系统
```python
# 定义系统, 离散非线性动力系统：xk+1 = xk^3, yk = xk
class SimpleDiscreteSystem(LeafSystem):
    # 初始化，并定义 状态 和 输出
    def __init__(self):
        LeafSystem.__init__(self)

        state_index = self.DeclareDiscreteState(1)          # 定义一个离散状态，采样时间为0.1s
        self.DeclareStateOutputPort("y", state_index)      # 定义一个输出端口
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=1.0,     # 每1秒更新一次离散状态
            offset_sec=0.0,     # 第一个更新事件在t=0时刻发生
            update=self.Update  # 定义更新函数
        )

    # xk+1 = xk^3
    def Update(self, context, discrete_state):
        x = context.get_discrete_state_vector().GetAtIndex(0)       # 获取离散状态
        x_next = x**3                                               # 定义下一个状态
        discrete_state.get_mutable_vector().SetAtIndex(0, x_next)   # 设置下一个状态
```

###### 一般的
```python
    # 在继承了LeafSystem的类中，init中有：
    self.DeclareVectorInputPort(name="a", size=2)  # 定义一个输入端口，大小为2
    self.DeclareVectorOutputPort(name="sum", size=2, calc=self.CalcSum)  # 定义一个输出端口，大小为2，输出由CalcSum函数计算
    self.DeclareAbstractOutputPort( mname="out",
        alloc=lambda: Value(RigidTransform()),
        calc=self.CalcOutput) 
    
    system.GetInputPort("a").FixValue(context, [3, 4])  # 固定输入端口的值为[3, 4]
    print(f"sum: {system.GetOutputPort('sum').Eval(context)}")
```

###### 状态值：使用抽象状态的时候，只能设置为离散时间更新

###### 发布方法不能修改任何状态
``` python
self.DeclarePeriodicPublishEvent(period_sec=1,offset_sec=0,publish=self.Publish) 
```

###### GetMyContextFromRoot

##### LeafSystem_
- 用于表示叶子系统的模板类，用@TemplateSystem.define装饰器定义













##### aynalysis 
- 用于系统仿真

###### Simulator
- 用于模拟系统的动态行为
```python
# 根据当前的diagram和context创建一个Simulator实例
simulator = Simulator(diagram, context)
simulator.AdvanceTo(10.0) # 将模拟时间推进到10秒
```

##### Diagram
- 用于表示系统图


##### DiagramBuilder
- 用于构建系统图
- 用于将多个系统连接在一起，形成一个更复杂的系统
```python
builder = DiagramBuilder()  # 创建一个 DiagramBuilder 实例
system = builder.AddSystem(SimpleContinuousSystem()) 
diagram = builder.Build() # 构建 Diagram
```

```python
# context 用于存储系统的状态和其他信息
context = diagram.CreateDefaultContext() # 创建默认的 context
context.SetContinuousState([0.9])        # 设置初始状态
```

``` python
builder.ExportInput(controller.get_input_port_desired_state()) # 导出控制器的期望状态输入端口

```

##### BasicVector_
- 用于表示基本的向量的模板类

#### autodiffutils
- 用于自动微分
##### 代码
``` python
from pydrake.autodiffutils import InitializeAutoDiff
context = diagram_autodiff.CreateDefaultContext()
x = np.array([1.0, 2.0])
x_autodiff = InitializeAutoDiff(x) # 初始化自动微分变量
diagram_autodiff.get_input_port(0).FixValue(context, x_autodiff) # 设置输入值
output = diagram_autodiff.get_output_port(0).Eval(context) # 计算输出
# 提取输出值和雅可比
y_value = np.array([yi.value() for yi in output])
y_gradient = np.array([yi.derivatives() for yi in output])
```
##### InitializeAutoDiff
- 用于初始化自动微分变量

##### ExtractGradient

#### BasicVector
- 用于表示基本的向量

#### scalar_conversion
- 用于标量转换

##### TemplateSystem
- 用于表示模板系统
``` python
@TemplateSystem.define("RunningCost_") # 定义一个模板系统
```

#### drawing
- 用于可视化系统
##### plot_system_graphviz
- 用于可视化系统
```python
from pydrake.systems.drawing import plot_system_graphviz
plot_system_graphviz(diagram) # 可视化系统
```

#### controllers
- 用于控制系统

#### geometry

##### SceneGraphConfig
- 用于配置场景图

##### StartMeshcat
- 用于启动Meshcat服务器



##### PIDController(pydrake.systems.framework.LeafSystem)
``` python
PidController(kp=[10.], ki=[1.], kd=[1.]) # 创建一个PID控制器
controller.get_input_port_estimated_state() # 获取估计状态的输入端口
controller.get_output_port_control() # 获取控制输出的输出端口
```

### common

#### containers

##### namedview

#### value

#### temp_dirctory
- 用于创建临时目录

### trajectories

#### PiecewisePolynomial


### multibody

#### parsing

##### Parser
- 用于解析URDF和SDF文件

#### plant
- 用于表示多体系统

##### AddMultibodyPlantSceneGraph
- 用于添加多体植物和场景图


### math

#### RigidTransform

#### RotationMatrix

#### RollPitchYaw

### visualization

#### AddDefaultVisualization
- 用于添加默认的可视化

#### ModelVisualizer
- 用于可视化模型

### examples

#### PendulumPlant
- 用于表示单摆系统
``` python
pendulum.get_input_port()
pendulum.get_output_port()
```


















