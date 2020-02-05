# ROS navigation の Action Server を使う

[ros/navigation/Home.md](Home.md)

---

## move_base にコマンドを送る

* `move_base`は`ROS navigation`メタパッケージの全体 ( SLAM 以外) を束ねるものである。
  * [move_base](http://wiki.ros.org/move_base)
  * 現状でも`/cmd_vel`に速度をパブリッシュすればロボットを動かせる。しかし今やりたいのは、`move_base`が持つ、大域的・局所的な経路計画と障害物回避機能である。

* スクリプトを格納するディレクトリを作る

```shell
cd
cd catkin_ws/src/oit_navigation_test
mkdir scripts
```

* 作成した`scripts`に下記ファイルをダウンロード
  * [simple_navigation_local_goals.py](./simple_navigation_local_goals.py)
  * 実行権限を付けておくこと。
* [Stage Simulator と ROS navigation](stage_simulator_and_ros_navigation)で作成した`navigation.launch`を起動しておく。
* スクリプトを実行する。

```shell
cd
cd catkin_ws/src/oit_navigation_test/scripts
rosrun oit_navigation_test simple_navigation_local_goals.py
```

* スクリプトを別名でコピーし編集する。

```shell
cp simple_navigation_local_goals.py simple_navigation_global_goals.py
```

* 編集箇所は次の通り

```python
COORD_TYPE = "base_link" # ロボットローカル座標系
を
COORD_TYPE = "map" # マップ座標系
にして、
GOAL.target_pose.pose.position.x = 5.5 （x 座標変更）
GOAL.target_pose.pose.position.y = -1.8 (y 座標追記)
```

## `Stage`のシミュレータ上に障害物を置く

* 地図を編集して障害物を作っているのではない点に注意。

```shell
cd
cd catkin_ws/src/oit_navigation_test/launch/simulation/worlds/
emacs HRC.world &
```

* 末尾に追記

```text
# Additional Obstacles
define block model
(
  size [0.5 0.5 0.5]
  gui_nose 0
)
block( pose [ 3 -1 0 0] color "blue")
```

* `navigation.launch`を再起動すると Stage 上の画面に青い四角形の障害物が出ているが、Rviz 上では出ていない。
* しかし`Laser`のデータを見ると、障害物があることが分かる。
* これで、 ROS の`navigation`の未知の障害物に対する回避機能を試すことができる。
* また、ここで定義した障害物は`Stage`のウィンドウ上でマウスドラッグにより移動可能である。
  * したがって、ドアオープンのシミュレーションや移動する人に対する回避シミュレーションもできる。

## 課題

* 任意の目標地点を数個（4点程度）地図上に設定し、それらを順番に回っていくプログラムを作成しなさい。
* このように最終目的地に至るまでのサブゴールをウェイポイントと呼びます。
  * 本来、ウェイポイントで停止して欲しくはないですが、現状では止まってしまう問題があります。

## 参考

* [ROS×Python勉強会：ウェイポイントナビゲーション(ActionLib：Python)](http://demura.net/lecture/12433.html)

---

[actuator/Home.md](Home.md)
