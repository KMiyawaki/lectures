# [SMACH（スマッシュ）（6）](http://wiki.ros.org/smach)

階層化された状態遷移を使い、複数地点のナビゲーションを行う

[smach/Home](Home.md)

---

## 実習（１）

* 次のファイルを`smach_tutorials/example`にダウンロードし，`navigation`のシミュレータを起動してから実行しなさい。
  * [state_machine_action_3.py](https://bitbucket.org/oit-trial/robocup_lectures/raw/ab097abdee506d2c613cedb9f6dc64e28215f9c4/actuator/smach_06/state_machine_action_3.py)
* 別ターミナルで`smach_viewer`も起動して状態遷移を可視化しなさい。

```shell
rosrun smach_viewer smach_viewer.py
```

## 問題（１）

* `TurnToPoint`クラスの`execute`を完成させて、ロボットが次の目標地点を向いてから自律移動するようにしなさい。

---

[smach/Home](Home.md)
