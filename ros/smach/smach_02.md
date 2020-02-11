# [SMACH（スマッシュ）（2）](http://wiki.ros.org/smach)

[smach/Home](Home.md)

---

## Navigation メタパッケージに頼らないロボットのコントロール

- [Navigation メタパッケージ](http://wiki.ros.org/ja/navigation)は経路計画等を自動的に行ってくれる便利なシステム。
- ただし自己位置推定が完璧でないとデタラメな動作をする。
- 実際のタスクでは最初正確な自己位置が取れないことが多い。
- その場合、とにかく少し前進してみると自己位置推定が正しくなってくることが多い。
- したがってタスク開始から少しの間、`Navigation`を使わず直接`/cmd_vel`にコマンドを送信しロボットを前進させる。

## 実習

- 次のファイルを`state_machine_simple.py`と同じディレクトリに保存しなさい。実行権限の付与を忘れないように。
  - [state_machine_no_navigation.py](https://raw.githubusercontent.com/KMiyawaki/lectures/master/ros/smach/state_machine_no_navigation.py)

### 実行順序

- まず、[シミュレータを起動する](../stage_simulator/stage_simulator_01.md)
- しばらくしてから`state_machine_no_navigation.py`を実行。

```shell
$ roscd smach_tutorials/examples/
$ ./state_machine_no_navigation.py
[INFO] [1581077522.716507, 286.100000]: State machine starting in initial state 'go_straight_01' with userdata:
	[]
[INFO] [1581077522.718719, 286.100000]: Executing state GoStraightByTime
[INFO] [1581077527.691294, 291.100000]: State machine terminating 'go_straight_01':'ok':'OK'
# Ctrl+Cで終了させる。
```

- 起動したら、`Stage simulator`の画面と`rviz`の画面をよく観察すること。
- 次いで、`smach_viewer`

```shell
$ rosrun smach_viewer smach_viewer.py
```

## 問題

- `state_machine_no_navigation.py`を修正し、直進->その場で（大体でよい）１回転-> 直進 という状態遷移を作成しなさい。
  - まずは`GoStraightByTime`クラスをコピーし、`TurnByTime`というクラスを作ってみよう。
- ロボットを四角形を描くように移動させてみよう。直進->90 度回転->直進・・・。時計回りに動いてから反時計回りに動くなど。
- 実機でも試してみよう。

---

[smach/Home](Home.md)
