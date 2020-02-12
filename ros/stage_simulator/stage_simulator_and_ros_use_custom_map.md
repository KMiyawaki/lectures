# 実機で作成した地図をシミュレーションで使う

[stage_simulator/Home](Home.md)

---

## 作成した地図を再確認

- 問題
  - [実機を動かす(1)](../navigation/middle_size_robot.md)で作成した地図のファイルが何であったか、そしてどこのディレクトリに移動させたかを思い出し、確かにそこに存在することを確認しなさい。

## 地図画像を`Stage Simulator`で使える形式にする

- 問題

  - `HRC.pgm`はそのままでは Stage Simulator では使えない。理由は何であったか。[Stage Simulator (2)](stage_simulator_02.md)を見て思い出しなさい。

- 次のコマンドを実行して`HRC.pgm`を変換する。

```shell
roscd oit_navigation/maps
./make_border.sh HRC.pgm
```

- 問題
  - 上記のコマンドで`HRC_border.png`というファイルが作成されている。`HRC.pgm`に対してどのような加工がされたか確認しなさい。

## world ファイルの作成

- `oit_navigation/launch/simulation/worlds/HRC.world`を修正する。

```shell
roscd oit_navigation/launch/simulation/worlds
emacs HRC.world
```

- 修正箇所は次の 2 箇所

```text
# load an environment bitmap
floorplan
(
   bitmap "../../../maps/HRC_border.png"
   size [34.1 21.3 0.5] # <- 修正(1)
   pose [0.85 -5.5 0 0 0] # <- 修正(2)
)
```

### size の修正

- `size`の`[]`内はスペース区切りで 3 つの数値が書いてある。
  - 1 つめは地図の幅、 2 つ目は地図の高さにする。3 つ目の数値は変えない。
  - 幅と高さは地図画像の 1 画素あたり 0.05 メートルとして計算する。
  - 画像の幅と高さは GUI のファイルマネージャで画像ファイルを右クリック->プロパティなどで調べられる。

### pose の修正

- `pose`の`[]`内はスペース区切りで 5 つの数値が書いてある。
  - 1 つ目と 2 つ目の数値を地図の中心の X、Y 座標にする。
- まず、`HRC.yaml`の`origin`を確認する。

```shell
roscd oit_navigation/maps
less HRC.yaml
実行結果
image: HRC.pgm
resolution: 0.050000
origin: [-5.000000, -17.800000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

- この例では作成された地図の左下座標が (-5.000000, -17.800000) であることを示している。
- 地図の幅と高さは前項の`size`の修正で調べているはず。
  - 以上の数値から地図の中心座標を計算する。
- 例：`size [29.3 22.9 0.5]`とした場合。
  - 中心は (29.3 / 2 - 5.0, 22.9 / 2 - 17.8) = (9.65, −6.35) となる。
  - `HRC.world`で`pose [9.65 -6.35 0 0 0]`とすればよい。

## `Stage`の起動

### 動作確認

- 下記コマンドを実行して`Stage`が起動すれば OK 。

```shell
roscd oit_navigation/launch/simulation
roslaunch stage.launch
```

### JoyStick teleop（自己位置推定つき）

- JoyStick を接続してから下記コマンドを実行し、 自己位置推定後、操作してみる。

```shell
roscd oit_navigation/launch/simulation
roslaunch localization.launch
```

### ナビゲーション

- 問題：下記コマンド実行後、任意のゴールを指定してナビゲーションさせなさい。

```shell
roscd oit_navigation/launch/simulation
roslaunch navigation.launch
```

---

[stage_simulator/Home](Home.md)
