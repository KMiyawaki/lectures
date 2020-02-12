# ssh

[unix/Home](./Home.md)

---

## `ssh`コマンド

- ネットワーク上の他のマシンにログインするコマンド。

```shell
ssh ユーザ名@ホスト名または接続先IPアドレス [-X]
```

- `-X`オプションは GUI のソフトを実行する際に必要。
  - サーバ側の設定によっては GUI のソフトが起動できない場合もある。

### `ssh`の必要性

- ロボットに搭載する CPU の形式は多様
  - 一般的なノートパソコン、とは限らない。
  - ディスプレイやキーボードの備わっていない、むき出しの基盤かもしれない。
- したがって、ディスプレイやキーボードを備えた操作用の PC をロボットに一台だけ搭載し、他の CPU にはネットワーク経由でログインして操作する必要がある。

## 実習(1)

- 大学のサーバにログインする。

```shell
$ ssh 「自分のアカウント名」@XXXXXXX # 接続先は教員から指示される。
# 初めて接続するときは次のような質問を受けるかもしれない。`yes`をタイプして次に進めればよい。
The authenticity of host 'remote_host_name (XXXXXXX)' can't be established.
RSA key fingerprint is XXXXXXXXXXXXX.
Are you sure you want to continue connecting (yes/no)?
自分のアカウント@XXXXXXX's password: # パスワードが聞かれるので入力し（画面には表示されない）Enter
Welcome to Ubuntu ...
Last login: Thu Feb  ...
[自分のアカウント@XXXX ~]%
```

- 実行後、パスワードを入力（画面には入力した文字は出ない）。
- `ls`や`cd`など、適当なコマンドを実行し、大学のサーバの自分のディレクトリにいることが分かったら次のコマンドで`ssh`を終了させる。

```shell
$ exit
logout
Connection to XXXXXXX closed.
```

---

## 問題(1)

- `openssh-server`をインストールしなさい。

```shell
$ sudo apt-get -y install openssh-server
[sudo] XXX のパスワード: # PCのパスワードを入力してEnter
パッケージリストを読み込んでいます... 完了
依存関係ツリーを作成しています
状態情報を読み取っています... 完了
...
```

- 隣の人の PC に`ssh`接続しなさい。`ssh`コマンドの`@`の後ろには IP アドレスも指定可能。
- 接続後、確かに別の PC にログインしていることを確認する方法を説明しなさい（様々な方法がある）。

---

## 補足

- 次のようなメッセージが出て接続できない場合がある。

```shell
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@    WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!     @
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
IT IS POSSIBLE THAT SOMEONE IS DOING SOMETHING NASTY!
Someone could be eavesdropping on you right now (man-in-the-middle attack)!
It is also possible that the RSA host key has just been changed.
・・・
RSA host key for remote_host has changed and you have requested strict checking.
Host key verification failed.
```

- 接続先 PC が IP アドレスを変更した場合や IP アドレスが同じであっても異なる PC に接続する際に発生する。
- 最も簡単な対処方法は下記の通り。

```shell
$ rm ~/.ssh/known_hosts
```

- 次のページも参考になる。`.ssh/config` に設定を書いておくのもよい。
  - [SSH 接続エラー回避方法](https://qiita.com/grgrjnjn/items/8ca33b64ea0406e12938)
  - [OpenSSH の警告メッセージを出さないようにする方法](https://qiita.com/shotaTsuge/items/48bdaccdafa5475d9016)

---

[unix/Home](./Home.md)
