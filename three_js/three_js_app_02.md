# Three.js(付録2)

[three_js/Home](./Home.md)

---

## Microsoft 3D ライブラリの活用

### Node.js と gltf-pipeline のインストール

10.x 系でないと無理。

#### Win10

- latest-v10.x を入れる。
- [node-v10.19.0-x64.msi](https://nodejs.org/dist/latest-v10.x/node-v10.19.0-x64.msi)
- コマンドプロンプトを開く。

```shell
% node --version
v10.19.0
% npm install -g gltf-pipeline
C:\Users\{user name}\AppData\Roaming\npm\gltf-pipeline -> C:\Users\{user name}\AppData\Roaming\npm\node_modules\gltf-pipeline\bin\gltf-pipeline.js
+ gltf-pipeline@2.1.8
added 38 packages from 20 contributors in 15.622s
```

#### Ubuntu 18.04

```shell
$ curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
$ sudo apt install nodejs
$ node --version
v10.19.0
$ sudo npm install -g gltf-pipeline
```

## 頂点データの圧縮とテクスチャのリサイズ

`~/models/ship.glb`が対象のモデルとして。

```shell
$ cd ~/models
$ gltf-pipeline -i ship.glb -o ship.gltf -st
Total: 520.531ms
$  ls -lh
合計 43M
-rw-r--r-- 1 user user 600K  3月  9 08:44 ship.bin # 分離された頂点データのファイル。
-rwxrw-rw- 1 user user  22M  3月  9 07:11 ship.glb # 元ファイル。
-rw-r--r-- 1 user user 4.5K  3月  9 08:44 ship.gltf # 分離されたメタデータのファイル。
-rw-r--r-- 1 user user 4.5M  3月  9 08:44 ship0.png # 分離されたテクスチャファイル。
-rw-r--r-- 1 user user 4.7M  3月  9 08:44 ship1.png # 分離されたテクスチャファイル。
-rw-r--r-- 1 user user 7.0M  3月  9 08:44 ship2.png # 分離されたテクスチャファイル。
-rw-r--r-- 1 user user 4.8M  3月  9 08:44 ship3.png # 分離されたテクスチャファイル。
$ mogrify -resize 256x *.png # 解像度 256x256に 縮小する。解像度は 2 のべき乗にすること。
$ ls -lh
合計 23M
-rw-r--r-- 1 oit oit 600K  3月  9 08:44 ship.bin
-rwxrw-rw- 1 oit oit  22M  3月  9 07:11 ship.glb
-rw-r--r-- 1 oit oit 4.5K  3月  9 08:44 ship.gltf
-rw-r--r-- 1 oit oit  96K  3月  9 08:48 ship0.png # テクスチャのファイルサイズが激減する。
-rw-r--r-- 1 oit oit 104K  3月  9 08:48 ship1.png
-rw-r--r-- 1 oit oit 104K  3月  9 08:48 ship2.png
-rw-r--r-- 1 oit oit 107K  3月  9 08:48 ship3.png
$ gltf-pipeline -i ship.gltf -b -d
Total: 1011.394ms
$ ls -lh|grep glb
-rw-r--r-- 1 oit oit 506K  3月  9 08:51 ship-processed.glb # 完成した glb ファイル。これを使う。
-rwxrw-rw- 1 oit oit  22M  3月  9 07:11 ship.glb
```

---

[three_js/Home](./Home.md)
