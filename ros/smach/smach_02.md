# [SMACH（スマッシュ）（2）](http://wiki.ros.org/smach)

[smach/Home](Home.md)

---

## Navigation メタパッケージに頼らないロボットのコントロール

* [Navigation メタパッケージ](http://wiki.ros.org/ja/navigation)は経路計画等を自動的に行ってくれる便利なシステム。
* ただし自己位置推定が完璧でないとデタラメな動作をする。
* 実際の競技ではドアオープンから始まるが、ドアの外では正確な自己位置が取れないことが多い。
* その場合、ドアが開いたらとにかく少し前進し、そこで自己位置推定を行うという流れが一般的。
* したがって競技開始から少しの間、`Navigation`を使わず直接`/cmd_vel`にコマンドを送信しロボットを前進させる。

## 実習

* 次のファイルを`state_machine_simple.py`と同じディレクトリに保存しなさい。実行権限の付与を忘れないように。
  * [state_machine_no_navigation.py](https://bitbucket.org/oit-trial/robocup_lectures/raw/ddcecb8d0289cce2ffec04af9b71715fddb562c3/actuator/smach_02/state_machine_no_navigation.py)

### 実行順序

* まず、`oit_navigation_test/launch/simulation/navigation.launch`
* しばらくしてから`state_machine_no_navigation.py`を実行。
* 起動したら、`Stage simulator`の画面と`rviz`の画面をよく観察すること。
* 次いで、`smach_viewer`

```shell
rosrun smach_viewer smach_viewer.py
```

## 問題

* `state_machine_no_navigation.py`を修正し、直進->その場で（大体でよい）１回転-> 直進 という状態遷移を作成しなさい。
  * まずは`GoStraightOdomByTime`クラスをコピーし、`TurnOdomByTime`というクラスを作ってみよう。
* ロボットを四角形を描くように移動させてみよう。直進->90度回転->直進・・・。時計回りに動いてから反時計回りに動くなど。
* 実機でも試してみよう。

---

[smach/Home](Home.md)
