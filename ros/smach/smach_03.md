# [SMACH（スマッシュ）(3)](http://wiki.ros.org/smach)

SMACH からセンサデータを使う

[smach/Home](Home.md)

---

## 実習

- まず、[シミュレータを起動する](../stage_simulator/stage_simulator_01.md)。
- 次のファイルを`~/catkin_ws/src/my_microbot_apps/scripts`にダウンロードし実行しなさい。
  - [state_machine_wait_for_msg.py](https://raw.githubusercontent.com/KMiyawaki/lectures/master/ros/smach/smach_03/state_machine_wait_for_msg.py)

```shell
$ roscd my_microbot_apps/scripts/
$ pwd
/home/[user name]/catkin_ws/src/my_microbot_apps/scripts
$ wget https://raw.githubusercontent.com/KMiyawaki/lectures/master/ros/smach/smach_03/state_machine_wait_for_msg.py
--2020-10-28 12:07:27--  https://raw.githubusercontent.com/KMiyawaki/lectures/master/ros/smach/smach_03/state_machine_wait_for_msg.py
・・・
2020-10-28 12:07:27 (4.41 MB/s) - ‘state_machine_wait_for_msg.py’ saved [4469/4469]

$ chmod u+x state_machine_wait_for_msg.py
$ ls -l
・・・
-rwxr--r-- 1 oit oit 4469 Oct 28 12:07 state_machine_wait_for_msg.py
```

## 問題(1)

- 上記のプログラムはレーザレンジファインダやオドメトリのセンサデータを受信しメッセージを表示している。

  - `WaitForLaserScan`を編集し`LaserScan`の正面数本分のデータの距離を表示しなさい。
    - `sensor_msgs/LaserScan`の中身は[ここ](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)
  - 上記同様に`WaitForOdometry`を編集し、`/odom`トピックの内容（ x, y 座標とロボットの向き）を表示させなさい。
    - `nav_msgs/Odometry`の中身は[ここ](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)

- `Odometry`（オドメトリ）とはタイヤの回転を使って推定されている位置情報のことです。
  - レーザによる自己位置推定ができないときはこれに頼ります。

## 問題(2)

- ロボット前方に何か障害物を置いて、レーザでそれを検出し、障害物が除去されたら前進させなさい。
  - Microbot 搭載のレーザ[YDLIDAR X4](https://www.ydlidar.com/products/view/5.html)は11メートルまでの障害物しか検出できません。11メートル以内の空間に何もない場合、その場所の距離はゼロとして返ってきます。このことに注意してください。
  - 上記で利用した`nav_msgs/Odometry`を使い，数メートル進ませるようにしなさい。
  - この課題は、ロボットに「ドアが開いたら部屋に入る」という行動をさせることを想定しています。`WaitForLaserScan`とは別に`WaitForDoorOpen`などを実装することが理想です。

---

[smach/Home](Home.md)
