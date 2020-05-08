# WSL

[Home](../README.md)

---

WSL(Windows Subsystem for Linux) は Windows10 上で Linux を動作させるシステム。WSL1 と wsl2 があるが、この記事では wsl1 を扱う。  
基本的にはインターネット上で「wsl インストール」などと検索すれば記事が出てくるのでそれに従えばよいが、GUI ソフトを使う際にいくつか注意事項がある。

## Windows Power Shell を管理者権限で起動する

Windows の検索ボックスから「PowerShell」と入力すると PowerShell が出るので、右クリックで管理者権限で起動する。

![2020-05-08_082304.png](./install_wsl/2020-05-08_082304.png)

## WSL1 のインストール

次の記事に従う。

- [Windows 10 用 Windows Subsystem for Linux のインストール ガイド](https://docs.microsoft.com/ja-jp/windows/wsl/install-win10)

ここで、**選択する Linux ディストリビューションは Ubuntu18 LTS である。**  
Microsoft Store も PowerShell のときと同じで Windows の検索ボックスから「Store」と検索すれば出てくる。Microsoft Store で「Ubuntu」と検索し Ubuntu18.04 LTS を選択する。  
インストール時にマイクロソフトアカウントを要求されるが、キャンセルすればそのままインストールできる。

![2020-05-08_082304.png](./install_wsl/2020-05-08_083516.png)

## WSL1 の起動

起動方法は一般的な Windows のプログラムと同じでスタートボタンから起動できる。

![2020-05-08_084422.png](./install_wsl/2020-05-08_084422.png)

起動後は次の記事に従い、ユーザ名とパスワードを設定する。

- [新しくインストールされたディストリビューションの初期化](https://docs.microsoft.com/ja-jp/windows/wsl/initialize-distro)

## GUI を使えるようにする

### VcXsrv をインストールする

下記記事の「4-2.VcXsrv 初期設定」までを実施する。**4-3 以降は絶対にやってはいけない。**

- [初心者のための WSL( 2 ) ~GUI 設定,デスクトップ環境設定編~](https://qiita.com/yoshige/items/7a17bb7a3582d72a7e48)

4-2 まで完了したら WSL を起動し、以下のコマンドを実行する。  
ここで`$`がある行が入力するコマンドを示す。ただし`$`は入力プロンプトなので、入力不要。
それ以外の行はコマンドの実行結果である。

**コピー＆ペーストで良いが、一行ずつ実行すること！**

```shell
$ sudo apt install x11-apps x11-utils x11-xserver-utils dbus-x11
[sudo] password for oit: # パスワードを入れる。
Reading package lists... Done
Building dependency tree
Reading state information... Done
x11-utils is already the newest version (7.7+3build1).
dbus-x11 is already the newest version (1.12.2-1ubuntu1.1).
The following additional packages will be installed:
  cpp cpp-7 gcc-7-base libisl19 libmpc3 libxcursor1 libxkbfile1 xbitmaps
Suggested packages:
  cpp-doc gcc-7-locales mesa-utils nickle cairo-5c xorg-docs-core
The following NEW packages will be installed:
  cpp cpp-7 gcc-7-base libisl19 libmpc3 libxcursor1 libxkbfile1 x11-apps x11-xserver-utils xbitmaps
0 upgraded, 10 newly installed, 0 to remove and 0 not upgraded.
Need to get 10.2 MB of archives.
After this operation, 30.0 MB of additional disk space will be used.
Do you want to continue? [Y/n] Y
Get:1 http://archive.ubuntu.com/ubuntu bionic-updates/main amd64 gcc-7-base amd64 7.5.0-3ubuntu1~18.04 [18.3 kB]
Get:2 http://archive.ubuntu.com/ubuntu bionic/main amd64 libisl19 amd64 0.19-1 [551 kB]
Get:3 http://archive.ubuntu.com/ubuntu bionic/main amd64 libmpc3 amd64 1.1.0-1 [40.8 kB]
# 省略
Processing triggers for libc-bin (2.27-3ubuntu1) ...
Processing triggers for man-db (2.8.3-2ubuntu0.1) ...
$ cd
$ echo 'export DISPLAY=localhost:0.0' >> ~/.bashrc
$ echo 'export LIBGL_ALWAYS_INDIRECT=0' >> ~/.bashrc
$ source ~/.bashrc
```

WSL を再起動し、`xeyes`コマンドを実行して次のようなウィンドウが開けば OK。

![2020-05-08_085753.png](./install_wsl/2020-05-08_085753.png)

### PC と VcXsrv の再起動

一旦 PC を再起動する。  
下記ファイルを PC のデスクトップにファイル名「config.xlaunch」として保存し、ダブルクリックする。

- [config.xlaunch](./install_wsl/config.xlaunch)

Windows のタスクトレイ右下の「^」マークをクリックし X のアイコンが**一つだけ**出ていれば OK。

![2020-05-08_090552.png](./install_wsl/2020-05-08_090552.png)

X のアイコンが 2 つ以上出ていたときは右クリック->Exit で全て終了させて再度 1 回だけ起動する。

![2020-05-08_090751.png](./install_wsl/2020-05-08_090751.png)

完了したら再び`xeyes`コマンドで GUI の動作確認をする。以降、PC 起動後はこの手順で VcXSrv を起動する。

---

[Home](../README.md)
