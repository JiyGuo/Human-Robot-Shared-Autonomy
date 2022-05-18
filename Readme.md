# Human-Robot Shared Autonomy Based on DMPs

运行流程：
1、开启UR5：https://github.com/JiyGuo/Dual_UR5

2、建立工作空间，clone代码后编译，在catkin_make的时候可能会报错，由于catkin_make是多线程的编译工具，因此部分原因是编写的srv和msg没有被编译到，但是部分程序用到了这些东西。解决方法是重复多次的catkin_make。
3、主函数在dmp/nodes/arbitration实现，运行时应当指定输出的文件名，如test0518：

```
cd {your workspace}
catkin_make
source devel/setup.bash
rosrun dmp arbitration_test test0518s
```
生成的数据会保存在：
```
~/{your workspace}/src/data/sc
```