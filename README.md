# mks_stepmotor_stm32f4
基于stm32f429 hal库 使用CAN控制 步进电机

- 使用 ： MKS SERVO28D和42D 驱动板和28步进电机和42步进电机 

- 主控：stm32f429igtx 正点原子阿波罗开发板

具体使用的商品链接：

28D步进电机驱动板：
https://item.taobao.com/item.htm?id=729403520934&mi_id=0000AbSxIYWUjWFMsPD2-NYZUjxTbgxkQxM0MEN8Ksrz4pY&spm=tbpc.boughtlist.suborder_itemtitle.1.7d9d2e8dk0cTcn&skuId=5337053670331

 42D步进电机驱动板：
https://item.taobao.com/item.htm?id=713689376773&mi_id=0000N9-lLhC-7iufjXeyNuoazY5ELyxh45K6KYHda9ZmI6k&spm=a21xtw.29978516.0.0&xxc=shop

新减速电机推杆：https://detail.tmall.com/item.htm?from=cart&id=520163231169&mi_id=00006IixU9sNLMg8CsVQ6MtuQ_YZe2b5lSH3lmvrcmcy-AE&skuId=3100794825595&spm=a1z0d.6639537%2F202410.item.d520163231169.5da37484JC9bIm&upStreamPrice=14000

H桥电机驱动板：https://item.taobao.com/item.htm?id=569591305325&mi_id=0000ZsjZukw21Vxz0PG0qrslLG_ezO0jhKLbsP8tBbIRz1M&spm=tbpc.mytb_itemcollect.item.goods&upStreamPrice=2460


### 包含功能
步进电机推杆及丝杆（大臂及小臂）    代码文件：**mks.c**
- 读取电机位置电流误差
- 设置电流工作模式
- 位置控制电机 
- 速度控制电机 （已成功实验）

减速电机推杆（肩部）    代码文件：**hmoto.c**
- H桥控制驱动