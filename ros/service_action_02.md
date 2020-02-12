# ROS サービスとアクション(2)

[ros/Home](Home.md)

---

## ROS のアクションに関する用語

- `Goal`：アクションの達成目標
  - (例)台車を目標に移動させるアクションにおける目標位置・姿勢。
- `Feedback`：フィードバック。Goal への進捗状況。
- `Result`：リザルト。結果として得られたもの(成功・失敗とは少し異なる)。
  - 台車を目標に移動させるアクションが終了した際の最後の位置・姿勢など。
- `Action`の状態：[actionlib/DetailedDescription](http://wiki.ros.org/actionlib/DetailedDescription)
  - `PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST`.
  - `Action`終了時に状態を取得することで成功したか否かを得ることが多い。
  - [actionlib.simple_action_client.SimpleActionClient.get_state API](http://docs.ros.org/kinetic/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#a1496dbc011f48451f4ea98e1ad2f8cd9)を参照。

---

## アクションを作る

- アクションの`Goal`、`Result`、`Feedback`を表す型を`.action`ファイルに記述する。
- 今回は皿洗いをする装置を制御するノードを想定し`DoDishes.action`ファイルを作成して編集する。エディタは何でも良い。

```shell
$ roscd beginner_tutorials
$ mkdir action
$ cd action
$ emacs DoDishes.action &
```

- 内容は次の通り。

```text
# Define the goal
uint32 dishwasher_id # Specify which dishwasher we want to use
---
# Define the result
uint32 total_dishes_cleaned
---
# Define a feedback message
float32 percent_complete
```

---

## アクションを使えるようにする

```shell
$ roscd beginner_tutorials
$ emacs CMakeLists.txt &
```

- 編集箇所(1)

```text
find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs # <- 追加
)
```

---

- 編集箇所(2)

```text
## Generate added messages and ・・・
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs # <-追加
)
```

---

- 編集箇所(3)

```text
## Generate actions in the 'ac・・・
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
```

- 次のように編集する。

```text
## Generate actions in the 'ac・・・
add_action_files(
  FILES
  DoDishes.action # <- これに変更
)
```

---

## package.xml を修正する

```shell
$ roscd beginner_tutorials
$ emacs package.xml &
```

- 修正箇所(1)

```xml
<build_depend>std_msgs</build_depend>
```

- 上記のすぐ下に下記を挿入。

```xml
<build_depend>actionlib</build_depend>
<build_depend>actionlib_msgs</build_depend>
```

---

- 修正箇所(2)

```xml
<build_export_depend>std_msgs</build_export_depend>
```

- 上記のすぐ下に下記を挿入

```xml
<build_export_depend>actionlib</build_export_depend>
<build_export_depend>actionlib_msgs</build_export_depend>
```

- 修正箇所(3)

```xml
<exec_depend>std_msgs</exec_depend>
```

- 上記のすぐ下に下記を挿入

```xml
<exec_depend>actionlib</exec_depend>
<exec_depend>actionlib_msgs</exec_depend>
```

---

## build_export_depend および exec_depend が無い場合

- `run_depend`というタグがあるはず。次のように修正する。

```xml
<run_depend>std_msgs</run_depend>
```

- 上記のすぐ下に下記を挿入。

```xml
<run_depend>actionlib</run_depend>
<run_depend>actionlib_msgs</run_depend>
```

## コンパイル

```shell
$ cd
$ cd catkin_ws
$ catkin_make
```

- 生成結果は`~/catkin_ws/devel/lib/python2.7/dist-packages/beginner_tutorials/msg`

---

## アクションサーバーを作る

- `do_dishes_server.py`というファイルを作成し、アクションサーバーのプログラムを入力する。

```shell
$ roscd beginner_tutorials
$ cd scripts
$ emacs do_dishes_server.py &
```

- プログラムは次の通り。
  - [do_dishes_server.py](https://raw.githubusercontent.com/KMiyawaki/lectures/master/ros/service_action_02/do_dishes_server.py)

---

## アクションクライアントを作る

- `do_dishes_client.py`というファイルを作成し、アクションクライアントのプログラムを入力する。

```shell
$ roscd beginner_tutorials
$ cd scripts
$ emacs do_dishes_client.py &
```

- プログラムは次の通り。
  - [do_dishes_client.py](https://raw.githubusercontent.com/KMiyawaki/lectures/master/ros/service_action_02/do_dishes_client.py)

---

## 作成したアクションサーバとクライアントを使う

- 2 つのスクリプトに実行権限を付けること。
- ROS マスターが起動している前提で 2 つのターミナルを使い次のコマンドを実行する。

```shell
$ rosrun begineer_tutorials do_dishes_server.py
$ rosrun begineer_tutorials do_dishes_client.py
```

---

## 問題(1)

- `do_dishes_server.py`の`execute`メソッド冒頭に次のように追記しなさい。

```python
rospy.loginfo("Accept request. Type = " + type(goal).__name__ + ", dishwasher_id =" + str(goal.dishwasher_id))
if goal.dishwasher_id >= len(DoDishesServer.WASHERS) :
    self.server.set_aborted()
    return
```

- `do_dishes_server.py`で追記した上記のコードが呼び出されるように、`do_dishes_client.py`を修正しなさい。

## 問題(2)

- `Python`の辞書を利用して、連番ではない`dishwasher_id`に対応させなさい。
- 現状では`WASHERS`は配列なので、`0,1,2,3・・・`という`dishwasher_id`しか使えない。
- これを任意の`dishwasher_id`(例えば`9,23,56...`)が利用できるようにすること。

## 参考文献

- [ja/actionlib](http://wiki.ros.org/ja/actionlib)
- [actionlib Documentation](http://docs.ros.org/kinetic/api/actionlib/html/index.html)

---

[ros/Home](Home.md)
