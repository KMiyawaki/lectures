# [SMACH（スマッシュ）（6）](http://wiki.ros.org/smach)

階層化された状態遷移を使い、複数地点のナビゲーションを行う

[smach/Home](Home.md)

---

## 実習（１）

- まず、[シミュレータを起動する](../stage_simulator/stage_simulator_01.md)。
- 次のファイルを`~/catkin_ws/src/my_microbot_apps/scripts`にダウンロードし実行しなさい。
  - [state_machine_action_3.py](https://raw.githubusercontent.com/KMiyawaki/lectures/master/ros/smach/state_machine_action_3.py)
- 別ターミナルで`smach_viewer`も起動して状態遷移を可視化しなさい。

```shell
$ rosrun smach_viewer smach_viewer.py
```

## 問題（１）

- `TurnToPoint`クラスの`execute`を完成させて、ロボットが次の目標地点を向いてから自律移動するようにしなさい。

---

[smach/Home](Home.md)
