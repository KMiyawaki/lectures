# [SMACH（スマッシュ）(4)](http://wiki.ros.org/smach)

SMACH から Action を使う

[smach/Home](Home.md)

---

## 実習(1)

- まず、[シミュレータを起動する](../stage_simulator/stage_simulator_01.md)。
- 次のファイルを`~/catkin_ws/src/my_microbot_apps/scripts`にダウンロードし実行しなさい。
  - [state_machine_action_1.py](https://raw.githubusercontent.com/KMiyawaki/lectures/master/ros/smach/smach_04/state_machine_action_1.py)

```shell
$ roscd my_microbot_apps/scripts/
$ pwd
/home/[user name]/catkin_ws/src/my_microbot_apps/scripts
$ wget https://raw.githubusercontent.com/KMiyawaki/lectures/master/ros/smach/smach_04/state_machine_action_1.py
--2020-10-28 12:09:25--  https://raw.githubusercontent.com/KMiyawaki/lectures/master/ros/smach/smach_04/state_machine_action_1.py
・・・
2020-10-28 12:09:26 (4.14 MB/s) - ‘state_machine_action_1.py’ saved [2568/2568]

$ chmod u+x state_machine_action_1.py
$ ls -l
・・・
-rwxr--r-- 1 oit oit 2568 Oct 28 12:09 state_machine_action_1.py
```

## [SimpleActionState](http://wiki.ros.org/smach/Tutorials/Calling%20Actions)

- `smach`から`action`クライアントの機能を簡単に呼び出すための状態。

## 実習(2)

- 上記プログラムを別名でコピーする。

```shell
$ roscd my_microbot_apps/scripts
$ cp state_machine_action_1.py state_machine_action_2.py
$ ls
state_machine_action_1.py state_machine_action_2.py
```

- コピーして生成した`state_machine_action_2.py`の下記部分を編集する。

```python
coord_type = "base_link" # ロボットローカル座標系
を
coord_type = "map" # マップ座標系
にして、
move_base_goal.target_pose.pose.position.x = 4.0
move_base_goal.target_pose.pose.position.y = 3.0
move_base_goal.target_pose.pose.orientation.w = 1.0
```

- 編集が終了したら、上記と同様の手順でスクリプトを実行する。

## 問題(1)

- 上記のプログラムが何を行っているか類推してから任意の目標地点を数個（4 点程度）地図上に設定し、それらを順番に回っていくプログラムを作成しなさい。
  - このように最終目的地に至るまでのサブゴールをウェイポイントと呼びます。
  - 本来、ウェイポイントで停止して欲しくはないですが、現状では止まってしまう問題があります。

## 問題(2)

- [SMACH（スマッシュ）(3)](./smach_03.md)で、他のノードから`publish`された`String`に応じて様々な行動を行うプログラムを実装した。
- それを応用し、他のノードから場所の名前を`publish`し、その場所に自律移動させるプログラムを作成しなさい。
- 例えば`living`という文字列を受け取ったら座標`(***, ***)`に向かう、といった具合である。座標はあらかじめ自律移動可能な場所を調べて設定すること。
- 場所の名前は`living`、`dining`、`kitchen`、`bedroom`とする。
- さらに、コマンドの実行が終了したら新たなコマンドの受信待ち状態に遷移させ、何度でもコマンドを受けられるようにすること。

## 問題(3)

- ロボットがコマンド受信時やエラー発生時にテキストでメッセージをトピックとして`publish`できるようにしなさい。
  - （例）`recv command living`、`unknown command XXX`など。
- トピック名は`robot_message`とし、型は[`std_msgs/String`](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html)で`publish`しなさい。
- 動作テストは`$ rostopic echo /robot_message`としてロボットから発信されたメッセージを表示することで行いなさい。
- 動作が確認できたら`talker.py`を`input`関数でキーボードからコマンド入力し、ロボットから受け取ったメッセージは画面に表示できるように改良しなさい。

---

[smach/Home](Home.md)
