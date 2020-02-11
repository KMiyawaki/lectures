# ROS(2)

[ros/Home](Home.md)

---

## 他の PC との通信

- ROS が動作する PC を 2 台用意し、通信する。
- 以降用意した PC を、PC1、PC2 と表記する。

### 問題（１）

- `ifconfig`コマンドで PC1 および PC2 の IP アドレスを調べなさい。
- PC1 と PC2 が相互に通信できることを`ping`コマンドで確認しなさい。
  - 参考:[ロボット理工学科 演習](http://robot.isc.chubu.ac.jp/?p=538)

---

## ifconfig

- PC の IP アドレスを調べる。
  - `lo` (ローカルループバック)は自分自身のこと。
  - 有線と無線が接続されている場合は両方のアドレスが出ることがある。

![console_02-min.png](console_02-min.png)

---

## ping [IP アドレス]

- 指定した IP アドレスに接続できるかどうかを調べる。

```shell
$ ping 192.168.***.*** # 接続先のIPアドレスを指定する。
# ネットワーク接続に問題がない場合は次のような応答がある。
PING 192.168.***.*** (192.168.***.***) 56(84) bytes of data.
64 bytes from 192.168.***.***: icmp_seq=1 ttl=64 time=0.018 ms
64 bytes from 192.168.***.***: icmp_seq=2 ttl=64 time=0.065 ms
64 bytes from 192.168.***.***: icmp_seq=3 ttl=64 time=0.050 ms
...
# 接続できない場合は次のようなメッセージが出る。
From 192.168.***.*** icmp_seq=1 Destination Host Unreachable
```

---

## 他の PC との通信（２）

- 複数の PC を使う場合も ROS マスターは一つの PC のみで動かす。
- ROS マスターを動作させる PC をマスター、それ以外をスレーブと呼ぶ。

1. PC1 をマスター、PC2 をスレーブとする。
2. マスター・スレーブ両方とも`roscore`とすべての ROS ノードをを停止する。

### マスター側

- 任意のエディタで`~/.bashrc`を編集し、下記を末尾に追記する。

```shell
export ROS_IP=○○○.○○○.○○○.○○○ # 〇にはマスタの IPアドレス
export ROS_MASTER_URI=http://○○○.○○○.○○○.○○○:11311 # 同上
```

- ターミナルを開きなおして ROS マスタと`talker.py`を起動。

```shell
$ rosrun beginner_tutorials talker.py
```

---

### スレーブ側

- 任意のエディタで`~/.bashrc`を編集し、下記を末尾に追記する。

```shell
export ROS_IP=○○○.○○○.○○○.○○○ # 〇にはスレーブの IPアドレス
export ROS_MASTER_URI=http://△△△.△△△.△△△.△△△:11311 # △にはマスターのアドレス
```

- ターミナルを開きなおして`listener.py`を起動。

```shell
$ rosrun beginner_tutorials listener.py
```

---

## 応用問題（１）

- `talker.py`が出力するデータを好きな文字に変えて再び実行し、通信相手にメッセージを届けなさい。
- マスタとスレーブの役割を交代しながら実施すること。

---

## 後始末

- 任意のエディタで`~/.bashrc`を編集し、`ROS_IP`と`ROS_MASTER_URI`の`export`をコメントアウトしておくこと。
  - 行の先頭に「#」をつける。

```shell
# export ROS_IP=〇〇〇.〇〇〇.〇〇〇.〇〇〇
# export ROS_MASTER_URI=〇〇〇.〇〇〇.〇〇〇.〇〇〇
```

---

[ros/Home](Home.md)
