2025.5.17
# **北极熊lesson2小作业**

===========================

    使用两个节点，一个负责发出华氏度数据到 /fahrenheit ，另一个负责订阅 /fahrenheit 转换为摄氏度并发布到 /celsius（使用订阅发布机制，其他通信机制可自行研究）
 
    并将代码提交至github

——————————————————————————————————————

## 1.完成过程：
    1.编写两个消息接口.msg （后来发现根本不需要，有一个就行了，是我把消息接口和话题的关系搞混了，详见学习总结第4条）分别为Celsius和Fahrenheit  
    
    2.编写发布者，将消息发布到话题Fahrenheit
        回调函数使用成员函数实现
        将0.0-100.0的随机数作为华氏度数据以1s为间隔发布  

    3.编写接收者，接收Fahrenheit上的华氏度数据
        回调函数使用lambda函数实现
        进行摄氏度转换后，再由发布者发布到话题Celsius上  

    4.把我的矢山代码提交到了github上

## 学习总结：
    1.非AI完成（但是使用了Github Copilt作为代码提示）通家过查阅小鱼教程和代码笔记来完成代码框架搭建（代码一看就是鱼老师的风格）
        ### 小鱼代码风格：
        - private内进行变量声明/智能指针初始化
        - public里进行发布者/接收者初始化
        - 启动发布者/接受者可在回调函数中实现
        - 回调函数有三种写法  

    2.学会了哪怕包名称起错了后期的修改过程（记得打开自动保存或者修改后及时手动保存）
        - 修改package和CMakelists文档，有路径报错警告不要紧，source一下更新一下环境变量
        - srv/msg文件命名开头大写，且不能有特殊符号
        - 节点命名不能有大写，但可以用下划线（特殊符号）  
  
    3.通过实践熟悉了消息接口和话题通信的实现  

    4.搞清楚了消息接口和话题的关系
        最开始知道有std_msgs的Float32接口，但想自己做个接口试试，然后写昏头了，以为创建话题就得有个接口（确实需要接口，但不是每个话题的接口必须是独自的）反正最开始忘记了话题和接口的关系。
        最后在晚上测试代码的时候，改了过来，把接收者的消息接口都换成了同一个“fahrenheit”
    

希望早日跑上导航小车。
