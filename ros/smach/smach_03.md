# [SMACH（スマッシュ）（3）](http://wiki.ros.org/smach)

SMACH からセンサデータを使う

[smach/Home](Home.md)

---

## 実習

- 次のファイルを`smach_tutorials/example`にダウンロードし，シミュレータの`navigation.launch`を起動してから実行しなさい。
  - [state_machine_wait_for_msg.py](state_machine_wait_for_msg.py)

## 問題（１）

- 上記のプログラムはレーザレンジファインダやオドメトリのセンサデータを受信しメッセージを表示している。

  - `WaitForLaserScan`を編集し`LaserScan`の正面数本分のデータの距離を表示しなさい。
    - `sensor_msgs/LaserScan`の中身は[ここ](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)
  - 上記同様に`WaitForOdometry`を編集し、`/odom`トピックの内容を表示させなさい。
    - `nav_msgs/Odometry`の中身は[ここ](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)

- `Odometry`（オドメトリ）とはタイヤの回転を使って推定されている位置情報のことです。
  - レーザによる自己位置推定ができないときはこれに頼ります。

## 問題（２）

- ロボット前方に何か障害物を置いて、レーザでそれを検出し、障害物が除去されたら前進させなさい。
  - 上記で利用した`nav_msgs/Odometry`を使い，数メートル進ませるようにしなさい。
  - `WaitForLaserScanState`とは別に`WaitForDoorOpen`などを実装することが理想です。

---

[smach/Home](Home.md)
