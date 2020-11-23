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

- 上記で利用した`nav_msgs/Odometry`を使い，指定した距離だけを進ませるクラス`GoStraightOdomByDistance`を作成しなさい。
- まず、`WaitForOdometry`クラスをコピーしクラス名やコンストラクタを編集します。
- コンストラクタは次の通り。

```python
class GoStraightOdomByDistance(smach.State):
    def __init__(self, distance, linear_vel=0.4, cmd_vel="/cmd_vel", topic='/odom', time_lilmit=None, msg_wait=1.0):
        smach.State.__init__(self, outcomes=['ok', 'ng'])
        self.sensor_msg = SensorMessageGetter(
            topic, nav_msgs.msg.Odometry, msg_wait)
        self.time_lilmit = time_lilmit
        self.end_time = None
        self.linear_vel = linear_vel
        self.cmd_vel = cmd_vel
        self.distance = distance
```

- プログラム冒頭に`from geometry_msgs.msg import Twist`と`import math`を追加しておくこと。
- `execute`メソッドを編集し、`self.distance`だけ進んだかどうかをチェックして条件を満たしていれば`ok`を返却するようにしなさい。
- 進んだ距離の計算には、[math.hypot](https://docs.python.org/ja/3/library/math.html#math.hypot)を使っても良いでしょう。
  - 実際は平方根を取らずとも実装可能で、その方が高速です。
  - ただし、最初はどれだけ進んだかを画面に表示した方が分かりやすいでしょう。
- 完成したと思ったら以下のような状態遷移を作って動作確認をすること。
  - `GoStraightOdomByDistance`を２回実行するような状態遷移を構築して動作確認する。
  - `TurnByTime`を実行して、ロボットを回転させてから`GoStraightOdomByDistance`するような状態遷移を構築して動作確認する。

## 問題(3)

- ロボット前方に何か障害物を置いて、レーザでそれを検出し、障害物が除去されたら前進させなさい。
  - Microbot 搭載のレーザ[YDLIDAR X4](https://www.ydlidar.com/products/view/5.html)は11メートルまでの障害物しか検出できません。11メートル以内の空間に何もない場合、その場所の距離はゼロとして返ってきます。このことに注意してください。
  - この課題は、ロボットに「ドアが開いたら部屋に入る」という行動をさせることを想定しています。

---

[smach/Home](Home.md)
