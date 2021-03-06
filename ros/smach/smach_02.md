# [SMACH（スマッシュ）(2)](http://wiki.ros.org/smach)

[smach/Home](Home.md)

---

## Navigation メタパッケージに頼らないロボットのコントロール

- [Navigation メタパッケージ](http://wiki.ros.org/ja/navigation)は経路計画等を自動的に行ってくれる便利なシステム。
- ただし自己位置推定が完璧でないとデタラメな動作をする。
- 実際のタスクでは最初正確な自己位置を推定することは難しい。
- その場合、とにかく少し前進してみると自己位置推定が正しくなってくることが多い。
- したがってタスク開始から少しの間、`Navigation`を使わず直接`/cmd_vel`にコマンドを送信しロボットを前進させる。

## 実習

- 次のファイルを`state_machine_simple.py`と同じディレクトリに保存しなさい。実行権限の付与を忘れないように。
  - [state_machine_no_navigation.py](https://raw.githubusercontent.com/KMiyawaki/lectures/master/ros/smach/smach_02/state_machine_no_navigation.py)

```shell
$ roscd my_microbot_apps/scripts/
$ pwd
/home/[user name]/catkin_ws/src/my_microbot_apps/scripts
$ wget https://raw.githubusercontent.com/KMiyawaki/lectures/master/ros/smach/smach_02/state_machine_no_navigation.py
--2020-10-28 12:01:06--  https://raw.githubusercontent.com/KMiyawaki/lectures/master/ros/smach/smach_02/state_machine_no_navigation.py
・・・
2020-10-28 12:01:06 (2.69 MB/s) - ‘state_machine_no_navigation.py’ saved [1785/1785]

$ chmod u+x state_machine_no_navigation.py
$ ls -l
・・・
-rwxr--r-- 1 oit oit 1785 Oct 28 12:01 state_machine_no_navigation.py
```

### 実行順序

- まず、[シミュレータを起動する](../stage_simulator/stage_simulator_01.md)

```shell
$ roscd my_microbot_apps/launch/simulation/
$ roslaunch navigation.launch use_teleop:=true use_mouse:=false map_name:=sample_02
```

- しばらくしてから`state_machine_no_navigation.py`を実行。

```shell
$ roscd my_microbot_apps/scripts/
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
  - コンストラクタの引数名やフィールド名は変えた方が良い（`linear_vel`：直進速度、`angular_vel`：回転速度）。

```python
    def __init__(self, time_limit, angular_vel=???, cmd_vel="/cmd_vel"): # 初期値はどうする？
```

- ロボットを四角形を描くように移動させてみよう。直進->90 度回転->直進・・・。時計回りに動いてから反時計回りに動くなど。
  - シミュレーションのロボットは自分の都合の良い位置に自由に移動させて構わない。
- 実機でも試してみよう。
  - [作成した地図でナビゲーションする](https://github.com/KMiyawaki/lectures/blob/master/ros/robots/jetson_microbot_chrome.md#%E4%BD%9C%E6%88%90%E3%81%97%E3%81%9F%E5%9C%B0%E5%9B%B3%E3%81%A7%E3%83%8A%E3%83%93%E3%82%B2%E3%83%BC%E3%82%B7%E3%83%A7%E3%83%B3%E3%81%99%E3%82%8B)
  - [自己位置を推定する](https://github.com/KMiyawaki/lectures/blob/master/ros/robots/jetson_microbot_chrome.md#%E8%87%AA%E5%B7%B1%E4%BD%8D%E7%BD%AE%E3%82%92%E6%8E%A8%E5%AE%9A%E3%81%99%E3%82%8B)
  - 以上を実施し、正しい自己位置が取れた時点で`state_machine_no_navigation.py`を実行する。

---

[smach/Home](Home.md)
