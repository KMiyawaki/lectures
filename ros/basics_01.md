# ROS(1)

[ros/Home](Home.md)

---

## ROS とは

- Robot Operating System の略
  - [ROS のサイト](http://www.ros.org/)
- 複数の実行プログラムが協調する分散システムの構築をしやすくするミドルウェア
  - OS と皆さんが作成するアプリケーションの間に位置するソフト
- 実行プログラム間の通信を隠蔽してくれる仕組み、という理解でも可。

---

## 何故分散システムなのか

### ソフトウェアの面から

- ロボットの複雑なタスクを達成するにはたくさんの機能が必要。
  - 音声認識
  - 画像処理
  - アクチュエータ制御
  - 統合
- 全てを含んだ一つの実行プログラムを作成するとどうなるか
  - 一つの機能(関数)を変更した際にその影響が全体に及ぶ。
  - 一つの機能に問題があって、実行時エラーが発生した際に全ての機能が停止する。
  - 分担して開発しにくい。一つの機能だけをテストしにくい。

---

### ハードウェアの面から

- ロボットはたくさんのセンサを積んでいる。

![3028269162-orion.png](./basics_01/2087168698-3028269162-orion.png)

- 実行プログラムを一つにまとめてしまうと、1 台の PC しか使えない。
  - 処理能力は足りる? USB ポートは足りる?
  - 「ハブで増設すれば?」というのは USB 通信帯域の問題でうまくいかない場合もある。

---

## ROS 用語

- ノード(`Node`)・・・一つの実行プログラム。
- トピック(`Topic`)・・・ノード間で送受信されるデータ。名前(トピック名)と型を持つ。
- パブリッシャ(`Publisher`)・・・トピックを発信するノード
- サブスクライバ(`Subscriber`)・・・トピックを受信(購読)するノード。
- ROS マスター・・・ノード同士を結び付けてくれるプログラム。
- ワークスペース・・・ノードを作成するための場所。通常はホームディレクトリに作成する。
- パッケージ・・・作成したノードをある程度のまとまりでグルーピングしたもの。

---

## 実習(1)

- ROS マスターの起動。ターミナルで下記コマンドを実行。

```shell
$ roscore
... logging to /home/[user name]/.ros/log/9474a7ce-4941-11ea-a3d0-000c2924787d/roslaunch-ubuntu-7288.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://ubuntu:34303/
ros_comm version 1.14.3


SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.3

NODES

auto-starting new master
process[master]: started with pid [7299]
ROS_MASTER_URI=http://ubuntu:11311/

setting /run_id to 9474a7ce-4941-11ea-a3d0-000c2924787d
process[rosout-1]: started with pid [7310]
started core service [/rosout]
```

- 出力されたメッセージを確認すること。
  - `melodic`という文字が出ているはず。

### Melodic Morenia

- ROS の LTS (Long Term Support) バージョンの一つ。
- 確認できたら、`Ctrl+C`で終了させておく。

---

## ワークスペースの作成

```shell
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
Creating symlink "/home/[user name]/catkin_ws/src/CMakeLists.txt" pointing to "/opt/ros/melodic/share/catkin/cmake/toplevel.cmake"
$ cd ~/catkin_ws/
$ catkin_make
Base path: /home/[user name]/catkin_ws
Source space: /home/[user name]/catkin_ws/src
Build space: /home/[user name]/catkin_ws/build
Devel space: /home/[user name]/catkin_ws/devel
Install space: /home/[user name]/catkin_ws/install
####
#### Running command: "cmake /home/[user name]/catkin_ws/src -DCATKIN_DEVEL_PREFIX=/home/[user name]/catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/home/[user name]/catkin_ws/install -G Unix Makefiles" in "/home/[user name]/catkin_ws/build"
####
-- The C compiler identification is GNU 7.4.0
...
-- Build files have been written to: /home/[user name]/catkin_ws/build
####
#### Running command: "make -j2 -l2" in "/home/[user name]/catkin_ws/build"
####
```

- 任意のエディタで`~/.bashrc`を開き、ファイル最下段に下記を追記。

```text
source ~/catkin_ws/devel/setup.bash
```

- `.bashrc`：シェル(ターミナル)起動時に実行されるスクリプト
- `source`：スクリプトを実行するコマンド
- この修正により、ターミナル起動時に`~/catkin_ws/devel/setup.bash`が実行され、`ROS`の実行に必要な環境変数がセットされる。
- 作業中のターミナルを閉じる。
- ここまでの作業はワークスペース作成時に一度だけ必要。

---

## catkin_create_pkg コマンドによるパッケージの作成

- 第一引数：作成するパッケージの名前（例：`beginner_tutorials`）
- 第二引数以降：使用する機能（例：`std_msgs rospy roscpp`）を指定。

```shell
$ cd ~/catkin_ws/src
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
Created file beginner_tutorials/CMakeLists.txt
Created file beginner_tutorials/package.xml
Created folder beginner_tutorials/include/beginner_tutorials
Created folder beginner_tutorials/src
Successfully created files in /home/[user name]/catkin_ws/src/beginner_tutorials. Please adjust the values in package.xml.
$ cd ~/catkin_ws
$ catkin_make
```

- ここでターミナルを閉じる
  - 必須の操作ではない。演習の都合上の操作。

---

## 簡単なパブリッシャとサブスクライバの作成(1)

- 参考：[ROS/Tutorials/WritingPublisherSubscriber(python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
- ターミナルを開き、次のコマンドを実行

```shell
$ roscd beginner_tutorials
```

### 問題(1)

- カレントディレクトリを確認しなさい。

---

## roscd [パッケージ名]

- `ROS`パッケージのディレクトリに移動できる。
- `.bashrc`に`source`コマンドを追記したので、自作パッケージのディレクトリを簡単に取得できるようになっている。
- 下記のコマンドでその他のパッケージを見に行こう。

```shell
$ roscd turtlesim
$ pwd
/opt/ros/melodic/share/turtlesim
$ roscd rviz
$ pwd
/opt/ros/melodic/share/rviz
$ roscd beginner_tutorials
```

---

## 簡単なパブリッシャとサブスクライバの作成(2)

```shell
$ roscd beginner_tutorials
$ mkdir scripts
$ cd scripts
$ pwd
/home/ユーザ名/catkin_ws/src/beginner_tutorials/scripts
```

- `scripts`ディレクトリに下記二つのファイルをダウンロード
  - [talker.py](https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py)
  - [listener.py](https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py)

### 問題(2)

- ダウンロードした 2 つのファイルにユーザの実行権限をつけなさい。

---

```shell
$ chmod u+x talker.py
$ chmod u+x listener.py
```

- 権限が付与されたことを`ls -l`で確認すること。

```shell
$ ls -l
合計 8
-rwxrw-r-- 1 robocup2020 robocup2020 2406  2月 12 12:43 listener.py
-rwxrw-r-- 1 robocup2020 robocup2020 2217  2月 12 12:43 talker.py
```

---

## talker.py の実行

```shell
$ cd ~/catkin_ws
$ catkin_make
$ rosrun beginner_tutorials talker.py
```

- エラーが出て何も起きないはず。
  - どんなメッセージか確認する。
  - `Ctrl+C`でプログラムを終了させる。
- エラーが出ずに無事実行できた人は手順を飛ばしているか、勘の良い人。

---

## rosrun [パッケージ名][ノード名]

- あるパッケージに含まれるノードを実行する。
  - ただし、ノードの実行には原則事前に ROS マスターを起動しておくことが必要。
- 別のターミナルを開き`roscore`を実行する。
- さらに別のターミナルを開き次のコマンドを実行。

```shell
$ rosrun beginner_tutorials talker.py
[INFO] [1581037099.621621]: hello world 1581037099.62
[INFO] [1581037099.722943]: hello world 1581037099.72
[INFO] [1581037099.822706]: hello world 1581037099.82
...
```

- さらに別のターミナルを開き次のコマンドを実行。

```shell
$ rosrun beginner_tutorials listener.py
[INFO] [1581037131.453663]: /listener_8862_1581037131191I heard hello world 1581037131.45
[INFO] [1581037131.555024]: /listener_8862_1581037131191I heard hello world 1581037131.55
[INFO] [1581037131.658074]: /listener_8862_1581037131191I heard hello world 1581037131.65
...
```

- 二つのノードを動かしたまま、次項のコマンドを実行すること。

---

## rqt_graph

- ROS のノード同士のつながりを可視化する。

```shell
$ rqt_graph
```

![rqt-min.png](./basics_01/rqt-min.png)

## rostopic list

- 現在流れているトピックのリストを得る。

```shell
$ rostopic list
/chatter
/rosout
/rosout_agg
```

---

## rostopic echo [トピック名]

- [トピック名]のデータを表示する。
- 例:`rostopic echo /chatter`
  - トピック名は`tab`キー補完可能

```shell
$ rostopic echo /chatter
data: "hello world 1581037256.15"
---
data: "hello world 1581037256.25"
---
data: "hello world 1581037256.35"
...
```

## rostopic type [トピック名]

- [トピック名]の型を表示する。
- 例:`rostopic type /chatter`

```shell
$ rostopic type /chatter
std_msgs/String
```

---

## talker.py のポイント

- エディタ等で`talker.py`を見てみる。

```python
def talker():
    # 'chatter'というトピック名にデータをパブリッシュする準備。
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # 'talker'という名前でノードを生成する。
    # anonymous=True により、名前にランダムな番号が追加される。
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hzでループを回す。
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str) # 端末上に hello_str の内容を表示。
        pub.publish(hello_str) # hello_str をパブリッシュ。
        rate.sleep()
```

---

## listener.py のポイント

- エディタ等で`listener.py`を見てみる。

```python
def callback(data):
    # 受信したデータを表示。
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    # 'chatter'というトピック名のデータを受信する準備。
    # 受信した瞬間に callback というメソッドが呼ばれるようにしている。
    rospy.Subscriber('chatter', String, callback)
    # 無限ループ開始
    rospy.spin()

if __name__ == '__main__':
    listener()
```

### 問題(3)

- ROS マスター、`talker.py`、`listener.py` を全て`Ctrl+C`で終了させなさい。

---

## 応用問題(1)

- ROS の`std_msg`について調べなさい。
  - `String`以外にどのような型が用意されているか、[std_msgs](http://wiki.ros.org/std_msgs)を参考に調べなさい。

---

## 応用問題(2)

- `talker.py`を次のように変更しなさい。

### 修正(2-1)

```python
from std_msgs.msg import String
from std_msgs.msg import Int32 # 追記
```

### 修正(2-2)

```python
pub = rospy.Publisher('chatter', String, queue_size=10)
pubInt32 = rospy.Publisher('number', Int32, queue_size=10) # 追記
```

---

### 修正(2-3)

```python
pub.publish(hello_str)
pubInt32.publish(number) # 追記
number = (number + 1) % 20 # 追記
```

- `talker.py`を実行しなさい。
- `rostpic list`を使って、どのようなトピックが流れるようになったかを確認しなさい。
- `rostopic echo`で実際にデータの内容を確認しなさい。

---

## 応用問題(3)

- `listener.py`を次のように編集し、実行結果を確認しなさい。

### 修正(3-1)

```python
from std_msgs.msg import String
from std_msgs.msg import Int32 #追記
```

### 修正(3-2)

```python
rospy.Subscriber('chatter', String, callback)
rospy.Subscriber('number', Int32, callbackInt32) # 追記
```

### 修正(3-3)

```python
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def callbackInt32(data): # 追記
    rospy.loginfo(rospy.get_caller_id() + 'I heard %d', data.data) # 追記
```

## 応用問題(4)

- 2 人組になり[複数の PC で ROS の通信を行う設定](./basics_02.md)を行い、双方で`talker.py`と`listener.py`を動かし通信しなさい。
- `talker.py`が出力するデータを好きな文字に変えて再び実行し、通信相手にメッセージを届けなさい。
- マスタとスレーブの役割を交代しながら実施すること。

---

[ros/Home](Home.md)
