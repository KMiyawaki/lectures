# ROS サービスとアクション(1)

[ros/Home](Home.md)

---

## サービスとアクション

- トピックを使った通信の問題点=信頼できない。
  - `Publisher`は送信するだけ。`Subscriber`が受信したかどうかは気にしない。
  - データを取りこぼす可能性がある。
- サービスとアクションは、クライアント側からの要求に対しサーバ側が確実に応答を返す。
  - いわゆるリモート関数呼び出し(`Remote Procedure Call`)。
  - 他のプロセスが持っている機能を関数のように呼び出す。

---

## サービスとアクションの違い

- サービスは同期的
  - クライアントはサーバに対してサービスの要求を出した後、応答があるまで待たなければならない(ブロックされる)。
  - クライアントは複数のサービス要求を同時に出して処理できない。
  - 非常に短時間で確実に終了するような処理に向いている。
    - 装置やソフトのパラメータを取得・設定するなど。
- アクションは非同期的な実装が可能。
  - クライアントはサーバに対してアクション要求を出した後、別の処理ができる。
    - つまり、複数のアクション要求を同時に出して処理できる。
  - 比較的長い時間の処理に向いている(音声対話など)。

---

## サービスを使う

- ほぼ[ja/ROS/Tutorials/UnderstandingServicesParams](http://wiki.ros.org/ja/ROS/Tutorials/UnderstandingServicesParams)と同じ内容。
- ROS マスターを起動する。

```shell
$ roscore
```

- 別ターミナルで[`turtlesim`](http://wiki.ros.org/turtlesim)を起動し、どのようなサービスが提供されているか調べる。

```shell
$ rosrun turtlesim turtlesim_node
$ rosservice list
/clear
/kill
/reset
/rosout/get_loggers
/rosout/set_logger_level
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/get_loggers
/turtlesim/set_logger_level
```

---

## rosservice list

- アクティブなサービスの情報を表示する

## rosservice type [サービス名]

- (例) `clear`という名前のサービスの引数の型を表示する。

```shell
$ rosservice type clear
std_srvs/Empty
```

- このサービスの引数の型は空で,サービスを呼び出すのに引数がいらないことを意味している。
  - サービス実行時にデータを送らず,終了時もデータを受け取らない。
- (例) `/turtle1/teleport_absolute`という名前のサービスの引数の型を表示する。

```shell
$ rosservice type /turtle1/teleport_absolute
turtlesim/TeleportAbsolute
```

- このサービスの型は`turtlesim/TeleportAbsolute`型。

---

## rossrv show [service type]

- サービスの引数と戻り値の定義を表示する。
- (例) `/turtle1/teleport_absolute`の引数、戻り値の型を表示する。

```shell
$ rosservice type /turtle1/teleport_absolute | rossrv show
float32 x
float32 y
float32 theta
---
# 空行がある
```

- `x, y, theta`はサービス実行時のパラメータ。`Turtle`の位置と向きを表している。
- `---`以降に何も書かれていない場合、このサービスには戻り値がないことを示している。

---

## rosservice call [service][args]

- 引数`[args]`を渡してサービス`[service]`を呼び出す。
- (例)座標`(5,2)`に亀を移動させ、向きを`0`にする。

```shell
$ rosservice call /turtle1/teleport_absolute 5 2 0
```

- (例)亀の移動軌跡を消す。

```shell
$ rosservice call clear
```

### 問題(1)

- 亀をいろいろな場所にテレポートさせ、画面の座標系について理解しましょう。
  - どこが原点ですか?
- 最後に`clear`サービスを実行し、亀の移動軌跡を消去してください。

---

## シンプルなサービスとクライアントを書く

- これからサービスを作る準備をする。
  - 内容は[ja/ROS/Tutorials/CreatingMsgAndSrv](http://wiki.ros.org/ja/ROS/Tutorials/CreatingMsgAndSrv)の「3.サービス(srv)を使う」に準拠している。

---

## サービスの型定義を書く

```shell
$ roscd beginner_tutorials
$ mkdir srv
```

- [AddTwoInts.srv](https://raw.githubusercontent.com/ros/ros_tutorials/lunar-devel/rospy_tutorials/srv/AddTwoInts.srv)を`srv`に保存する。
- `AddTwoInts.srv`の内容を確認する。

```shell
$ cd srv
$ less AddTwoInts.srv
int64 a
int64 b
--- # 上が引数で下が返却値
int64 sum
```

---

## サービスの型を使えるようにする

- `CMakeLists.txt`を編集する。エディタは何でも良い。ここでは`emacs`を使う。

```shell
$ roscd beginner_tutorials
$ emacs CMakeLists.txt &
```

### 編集箇所(1)

```text
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation # <-これを追加
)
```

---

### 編集箇所(2)

```text
## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
```

- 次のように修正する。

```text
## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddTwoInts.srv
)
```

---

### 編集箇所(3)

```text
## Generate added messages and・・・
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
```

- 先頭の`#`を削除し、次のように修正する。

```text
## Generate added messages and・・・
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

---

## コンパイル

```shell
$ cd
$ cd catkin_ws
$ catkin_make
```

- 生成された結果は`~/catkin_ws/devel/lib/python2.7/dist-packages/beginner_tutorials/srv`にある。

## 作成したサービスを使う

- [ja/ROS/Tutorials/WritingServiceClient(python)](http://wiki.ros.org/ja/ROS/Tutorials/WritingServiceClient%28python%29)の指示に従い、その内容を実施すること。

---

[ros/Home](Home.md)
