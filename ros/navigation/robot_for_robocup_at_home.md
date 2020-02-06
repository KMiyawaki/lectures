# 実機を動かす（1）

[ros/navigation/Home.md](Home.md)

---

## `oit_navigation`パッケージ

[oit_navigation](https://bitbucket.org/oit-trial/oit_navigation/src/master/)は台車と北陽の URG を使い、 ROS のナビゲーションメタパッケージを起動する。
このチュートリアルで扱う台車は [RoboClaw](http://www.ionmc.com/RoboClaw-2x30A-Motor-Controller_p_9.html) コントローラを使った２輪ロボットである。

## JoyStick teleop

- RoboClaw 、 URG 、 JoyStick を PC に接続する。
- SEVA の電源を入れる。
- 次のコマンドを実行する。

```shell
roscd oit_navigation/launch/real
roslaunch teleop_joy.launch
```

- 起動に成功すると RViz の画面が出て、レーザのデータが表示される。

- JoyStick で操作する。
  - １ボタンを押しながらゆっくりとレバーを倒す。
  - 慣れたら２ボタンでターボモード（速度が上がる）

## 占有格子地図の作成

- 次のコマンドを実行する。

```shell
roscd oit_navigation/launch/real
roslaunch mapping_joy.launch
```

- 起動に成功すると RViz の画面が出て、レーザのデータと占有格子地図作成中の様子が表示される。
- JoyStick で操作すると地図が徐々に作成されていくのが分かる。
  - このようにロボットの自己位置推定と地図作成を同時に行うことを SLAM (Simultaneous Localization and Mapping) という。
- 地図ができたら保存する。別のターミナルを開き、下記を実行する。

```shell
cd
rosrun map_server map_saver -f HRC
```

- ホームディレクトリ上に`HRC.pgm`と`HRC.yaml`が作成されていることを確認する。
- `~/catkin_ws/src/oit_navigation/maps`に地図を移動する。

```shell
cd
mv HRC.pgm HRC.yaml ~/catkin_ws/src/oit_navigation/maps
```

## JoyStick teleop（LRF による自己位置推定つき）

- 次のコマンドを実行する。

```shell
roscd oit_navigation/launch/real
roslaunch localization.launch
```

- 起動に成功すると RViz の画面が出て、レーザのデータと先ほど作成した地図が表示される。
- 自己位置を推定する。
  - RViz の`Estimate Robot Pose`ボタンを使い、 RViz の地図上でロボットの現在地から向いている方向に向かってドラッグし、マウスボタンを離す。
  - パーティクルが撒かれて自己位置が推定される。
- JoyStick で操作する。
  - LRF のデータが地図の形状とほぼ一致していれば、自己位置推定は成功している。

## ナビゲーション

- JoyStick を外す。
- 次のコマンドを実行する。

```shell
roscd oit_navigation/launch/real
roslaunch navigation.launch
```

- 起動に成功すると RViz の画面が出て、レーザのデータと先ほど作成した地図が表示される。
- まず、自己位置推定を行う。
- RViz の`Navigation goal`ボタンを使い、ロボットを移動させたい場所と最終的な姿勢を RViz の地図上でドラッグ＆ドロップにより指定する。

### 問題

- 作成した地図を[oit_navigation](https://bitbucket.org/oit-trial/oit_navigation/src/master/)リポジトリに`push`しましょう。

---

[ros/navigation/Home.md](Home.md)
