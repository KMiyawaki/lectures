# Jetson Microbot で作成した地図でシミュレーションする

[ros/robots/Home](Home.md)

---

## クライアント PC を用意する

- ネイティブな Linux に ROS その他の必要なソフトを[インストール](https://github.com/KMiyawaki/setup_robot_programming)する。
  - シミュレーションは仮想環境や WSL でも実行できる。
  - ただし、仮想環境の場合に`scp`でファイルをコピーする場合はネットワークをブリッジモードにしておくこと。
- ロボットに[Wifi 接続](./robot_for_ipbl_microbot_boot_shutdown.md)しておくこと。

### Jetson NANO に保存した地図をクライアント側で回収し、シミュレーション用ファイルを生成する

- [ロボットで作成した地図](https://github.com/KMiyawaki/lectures/blob/master/ros/robots/robot_for_ipbl_microbot_chrome.md#%E5%9C%B0%E5%9B%B3%E3%82%92%E4%BD%9C%E6%88%90%E3%81%99%E3%82%8B)をクライアント側にコピーする。
- ここでは作成した地図の名前を`my_map_01`として説明する。

```shell
$ roscd my_microbot_apps/maps
$ scp jetson@192.168.12.1:~/catkin_ws/src/my_microbot_apps/maps/my_map_01.* ./
jetson@192.168.12.1's password: # パスワードを入力する
my_map_01.pgm                                 100%   84KB 299.4KB/s   00:00
my_map_01.yaml                                100%  135    41.8KB/s   00:00
$ ls|grep my_map_01
my_map_01.pgm             my_map_01.yaml # 手元にコピーできた
$ ./make_simulation_world.sh my_map_01 # コピーしたマップを指定（拡張子なし）
Add black border into my_map_01.pgm...
Generated my_map_01_border.png
$ ls|grep my_map_01
my_map_01.pgm
my_map_01.world # シミュレーションに使うファイル
my_map_01.yaml
my_map_01_border.png # シミュレーションに使うファイル
```

- ここまでできたら、ロボットの電源を OFF にしてよい。

## クライアント PC だけで ROS のプログラムを実行できるようにする

- 任意のエディタで`~/.bashrc`を編集し、`ROS_IP`と`ROS_MASTER_URI`の`export`をコメントアウトしておくこと。
  - 行の先頭に「#」をつける。
  - すでに「#」がついていたり、該当する行がなかったりする場合はこの手順は不要。

```shell
# export ROS_IP=〇〇〇.〇〇〇.〇〇〇.〇〇〇
# export ROS_MASTER_URI=〇〇〇.〇〇〇.〇〇〇.〇〇〇
```

- 編集が終了したら、すべての端末を閉じる。
  - クライアント PC をロボットに接続する際は「#」を削除すること。

## クライアント PC でシミュレーションを起動する

```shell
$ roscd my_microbot_apps/launch/simulation
$ roslaunch navigation.launch map_name:=my_map_01 use_teleop:=true
```

- 起動後は[Stage Simulator （１）](../stage_simulator/stage_simulator_01.md)を参照。

---

[ros/robots/Home](Home.md)
