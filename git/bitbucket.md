# git(Bitbucket 編)

[git/Home](Home.md)

---

## [Bitbucket](https://bitbucket.org/)

- [Github](https://github.com/github)と並ぶ Git ホスティングサービス。
  - 無料プランでも非公開のプライベートリポジトリが作成可能。
  - 学校関係者は Academic Plan(容量無制限(大規模なバイナリは不可)、共同開発者無制限)が利用可能。メールアドレスで判定される。

### **!注意事項!**

- 非公開リポジトリとはいえネットワーク上に存在するものである、ということを忘れないようにすること。
- 個人情報の記載は厳禁。漏洩の可能性はゼロではない。
  - 学生プロジェクトのソフトは公開する場合もある。
- コミットオーサーの名前・・・フルネームは不可
- コミットオーサーのメールアドレス・・・各プロジェクトや授業で指定されたアドレス。

## ログイン

- [Bitbucket](https://bitbucket.org/)からログイン。
  - アカウント情報は教員より指示される。

## リモートリポジトリの作成

- ログイン直後の画面(ダッシュボード)の「リポジトリ」 「+」マークをクリック 「Repository」
- リポジトリ名を入力。「Include a README?」は「No」を選択
- 「詳細」をクリックして開発言語を選択(今回は Python) リポジトリの作成。
- リポジトリ名は oit\*test**\*\***(自分のイニシャル+誕生月日) -
  - 例: oit_test_km_0721
- リポジトリが作成される。ブラウザはこのままにしておく。
- 「既にプロジェクトがあります」をクリックし`git remote add`の引数(git@で始まるリモートリポジトリの URL)をいつでも参照できるようにしておく。

## git config

- git の初期設定をする。
- Linux アカウントごとに一回だけ必要。

```shell
$ git config --global user.name "oit-trial" # 教員より指示される
$ git config --global user.email XXXX@YYYY.com  # 教員より指示される
$ git config --global core.editor 'emacs -nw'
```

- git の練習用に ROS パッケージを作成する。

```shell
$ cd ~/catkin_ws/src
$ catkin_create_pkg oit_test_km_0721 std_msgs rospy roscpp
$ cd oit_test_km_0721 # 自分のリモートリポジトリ名に読み替え
```

## git init

- 現在のディレクトリをローカルリポジトリにする。

```shell
$ pwd
/home/ユーザ名/catkin_ws/src/oit_test_km_0721 # パッケージのディレクトリにいることを確認
$ git init # 作成したパッケージの git による管理を開始する。
$ git remote add origin git@bitbucket.org:XXXXXX/oit_test_km_0721.git # 教員から指示される。
```

## git remote add リモート URL 短縮名 URL

- 初期化したローカルリポジトリに対し、`push`(アップロード)先であるリモートリポジトリの URL 及びその短縮名を指定する。
- 短縮名は通常`origin`という名前をつける。

```shell
$ git remote add origin git@bitbucket.org:XXXXX/oit_test_km_0721.git # 教員から指示される。
```

- 以降`origin`は`git@bitbucket.org:XXXXX/oit_test_km_0721.git`と同意になる。
- URL はリモートリポジトリ作成直後のブラウザ上の画面に書いてあるのでコピーする。

## コミット用のシェルスクリプトを作成する

- ホームディレクトリに次のようなスクリプトを作成する。

```shell
$ cd
$ emacs commit_c15999.sh # 自分の学番など。
```

- 内容は次の通り。

```text
#!/bin/bash
git commit --author='K.Miyawaki <XXXX@YYYY.com>'
```

- 実行権限をつける。

```shell
$ chmod u+x ~/commit_c15999.sh
```

## git add

- ステージング(コミットの準備)を行う。

```shell
$ cd ~/catkin_ws/src/oit_test_km_0721
$ git add --all
$ git status
On branch master
  最初のコミット
  Changes to be committed:
  (use "git rm --cached <file>..." to unstage)
    new file:  CMakeLists.txt # どんなファイルがコミットに含まれるか良く確認すること。
    new file:  package.xml
```

## git commit

- 本演習では、シェルスクリプトを経由してコミットを行う。
- ローカルリポジトリで下記のようにシェルスクリプトを実行する。

```shell
$ cd ~/catkin_ws/src/oit_test_km_0721
$ ~/commit_c15999.sh # 先ほど作成したスクリプトの実行
```

- `emacs`エディタが開くはず。コミットログを編集して保存する。内容は次の通り。

```text
最初のコミット
(空行)
勉強会用のダミーリポジトリです。
頑張って勉強しましょう。
```

- `Ctrl+x`、`Ctrl+s`でログを保存。`Ctrl+x`、`Ctrl+c`で`emacs`を閉じる。
- これでコミット完了。
  - コミットをキャンセルしたい場合は 1 文字も入力せずに`Ctrl+x`、`Ctrl+c`

## git log

- コミットの履歴を見る。
- コミットハッシュ:コミットを識別するための ID。
  - ハッシュを指定して、ローカルリポジトリをその時点の状態に戻すことも可能(バグがあったら元に戻せる)。

```shell
$ git log
commit a3568bdd3743fd0a4a511916427f33bcf82e0e24
Author: Author: K.Miyawaki <XXXX@YYYY.com>
Date:
Mon Nov 13 10:14:09 2017 +0900
最初のコミット
勉強会用のダミーリポジトリです。
頑張って勉強しましょう。
```

## git push

- リモートリポジトリにプッシュする。

```shell
$ git push リモート URL 短縮名 ブランチ名
```

- 初回コミットした時点でデフォルトのブランチとして`master`が用意されている。

```shell
$ git push origin master
The authenticity of host 'bitbucket.org (XXX.XXX.XXX.XXX)' can't be established.
RSA key fingerprint is SHA256:****************************.
Are you sure you want to continue connecting (yes/no)?
```

- 一度も Bitbucket に接続していなかった場合、このようなメッセージが出る。
  - yes とタイプしてエンター。
- しかし、次のようなエラーが出て終了するはず

```shell
Connection to bitbucket.org closed by remote host.
fatal: Could not read from remote repository.
Please make sure you have the correct access rights
and the repository exists.
```

## SSH 鍵をインストールする

- 教員から秘密鍵と`ssh`の設定ファイルを受け取り、`~/.ssh`ディレクトリに配置する。
  - ここでは秘密鍵のファイル名を`oit_trial_rsa`、設定ファイルを`config`として説明する。
- 配置は次の通り

```shell
~/.ssh/config
~/.ssh/bitbucket/oit_trial_rsa
```

- 鍵のパーミッションを変更する。

```shell
$ chmod 600 ~/.ssh/bitbucket/oit_trial_rsa
```

- 再度プッシュするとエラー無くプッシュできる。

### config の内容

```text
Host bitbucket.org
    User git
    Port 22
    HostName bitbucket.org
    identityFile ~/.ssh/bitbucket/oit_trial_rsa
    TCPKeepAlive yes
    IdentitiesOnly yes
```

- このファイルにより、「どのホストに」「どの秘密鍵を使うか」を指定できる。
- したがって、利用する git ホストが増えても複数の秘密鍵をパソコンにインストールして利用できる。

### Bitbucket でコミット履歴を確認する

- ブラウザを再読み込みすれば、プッシュしたコミット履歴が見える。

## .gitignore

- `.gitignore`ファイルに常にバージョン管理対象外とするファイルを定義し、ローカルリポジトリのルートディレクトリに置いておく。
  - これにより`git add --all`の際に開発環境の都合で生成される中間ファイル等が自動的にバージョン管理から除外され、コミット対象外となる。
  - `.gitignore`自体もバージョン管理する。
- 誰もがよく使う`.gitignore`はすでに公開されている。
- `ROS`の[`.gitignore`](https://raw.githubusercontent.com/github/gitignore/master/ROS.gitignore)
  - ダウンロードして、`~/catkin_ws/src/oit_test_km_0721`に配置する。
  - ファイル名を`.gitignore`(先頭の「`.`」必要)に変えておくことを忘れないように。

## git mv 移動元ファイル 移動先ファイル

- `mv`という Linux コマンドがあったが、その`git`版(`git rm`もある)。
  - 単なる`mv`では`git`がファイルの移動を追跡できない。

### git mv の練習(1) ソースファイルの追加

- 何でも良いので Python スクリプトを作成し、追加してみる。

```shell
$ cd ~/catkin_ws/src/oit_test_km_0721/src # Python のスクリプトは scripts フォルダに入れるべきだが、わざと間違えている。
$ emacs talker.py # http://wiki.ros.org/ja/ROS/Tutorials/WritingPublisherSubscriber%28python%29の talker.py のコードを貼り付けて、emacs を終了させる。
```

### git mv の練習(2) ステージング

```shell
$ cd ~/catkin_ws/src/oit_test_km_0721
$ git add --all
$ git status
On branch master
Changes to be committed:
  (use "git reset HEAD <file>..." to unstage)
  new file: .gitignore
  new file: src/talker.py
```

### git mv の練習(3) コミット

- ローカルリポジトリでコミットのシェルスクリプトを実行する。

```shell
$ cd ~/catkin_ws/src/oit_test_km_0721
$ ~/commit_c15999.sh (先ほど作成したスクリプトの実行)
```

- `emacs`でコミットログを編集。内容は次の通り。

```text
ファイル追加
```

- 再度プッシュ

## git clone

- リポジトリをクローンする

```shell
$ git clone リポジトリ URL
```

- コマンド例

```shell
$ cd ~/catkin_ws/src
$ git clone git@bitbucket.org:XXXXX/oit_test_tonari.git
```

- 問題:隣の人が作成したリポジトリをクローンしなさい。
  - `git config --list`でリモートリポジトリが設定されていることを確認しなさい。
  - なければ`git remote add`でリモートリポジトリの URL を設定しなさい。

### git mv の練習(4) 隣の人のリポジトリに修正を加える

- 隣の人のリポジトリ名を`oit_test_tonari`として説明する。
- クローンしたリポジトリの内部を見てみる。

```shell
$ cd ~/catkin_ws/src/oit_test_tonari
$ ls
CMakeLists.txt package.xml src
$ cd src
$ ls
talker.py
```

- すでに実習したように Python スクリプトは`scripts`フォルダに入れるのが正しい。修正が必要。

```shell
$ cd ~/catkin_ws/src/oit_test_tonari /
$ mkdir scripts
$ git mv src/talker.py scripts/talker.py
$ git status
On branch master
Your branch is up-to-date with 'origin/master'.
Changes to be committed:
(use "git reset HEAD <file>..." to unstage)
renamed:
src/talker.py -> scripts/talker.py # ファイルの移動が追跡できている。
```

## git mv の練習(5) 修正をコミットしてプッシュする

- すでに作成したコミット用スクリプトを利用し間違いの無いようにコミットすること。
- コミットログは次の通り

```text
ファイルの移動

Python スクリプトは scripts フォルダに入れるのが正しい
```

- コミットできたらプッシュしてみる

```shell
$ git push origin master
```

- 問題:Bitbucket 上のコミット履歴から確かに隣の人に修正してもらったことを確認しなさい。

## git pull

- プル（リモートの変更をローカルに取り込む）

```shell
$ git pull リモート URL 短縮名 ブランチ名
```

- 隣の人に修正してもらった内容を自分のローカルリポジトリに取り込む。

```shell
$ cd ~/catkin_ws/src/oit_test_km_0721
$ git pull origin master
```

- 問題:隣の人に修正してもらった箇所がローカルに取り込めたことを確認しなさい。

## 競合(Conflict)

- 自分自身と隣の人でソースコードの同じ箇所に異なる編集を施して競合させる。
- まずは、役割(主開発者とサブ開発者)を決める。
- 役割を決めたら、主開発者のリポジトリのみを残して、それ以外のローカルリポジトリは削除する。
- 主開発者は`talker.py`の`str = "hello world %s"%rospy.get_time()`を`str = "main editor world %s"%rospy.get_time()`に変更してコミット&プッシュ(Bitbucket 上でも確認する)。
- コミットログは「出力メッセージ変更」という 1 行。
- 主開発者のプッシュが終了したら、サブ開発者は`str = "hello world %s"%rospy.get_time()`を`str = "sub editor world %s"%rospy.get_time()`に変更してコミット&プッシュ。
- コミットログは「サブ開発者も変更」の一行。
- エラーが出る。

```text
To git@bitbucket.org:XXXXX/oit_test_km_0721.git
! [rejected]
master -> master (fetch first)
error: failed to push some refs to 'git@bitbucket.org:XXXXX/oit_test_km_0721.git'
```

- サブ開発者はプルする。

```shell
$ git pull origin master
```

- 競合が検出される。

```shell
remote: Counting objects: 4, done.
remote: Compressing objects: 100% (3/3), done.
remote: Total 4 (delta 2), reused 0 (delta 0)
Unpacking objects: 100% (4/4), done.
From bitbucket.org:XXXXX/oit_test_km_0721
8cb6d06..a252df8 master
-> origin/master
Auto-merging scripts/talker.py
CONFLICT (content): Merge conflict in scripts/talker.py
Automatic merge failed; fix conflicts and then commit the result.
```

### 競合しているファイルを確認

```shell
$ git status
On branch master
Your branch and 'origin/master' have diverged,
and have 1 and 1 different commit each, respectively.
(use "git pull" to merge the remote branch into yours)
You have unmerged paths.
(fix conflicts and run "git commit")
Unmerged paths:(ここに競合ファイルの一覧が出る)
(use "git add <file>..." to mark resolution)
both modified:
talker.py
```

### 競合の解決

- サブ開発者が`talker.py`を編集

```shell
$ emacs talker.py
```

- 競合箇所が次のように表示されている。

```text
<<<<<<< HEAD
str = "sub editor world %s"%rospy.get_time()
=======
str = "main editor world %s"%rospy.get_time()
>>>>>>> a252df8d3648946629541cfdc63878f11a0548a5
```

- 手動で編集する。

```python
str = "all editor world %s"%rospy.get_time()
```

- 通常通り commit &プッシュ。コミットログはマージ ( merge :競合を解決してファイルを統合すること)したことが明記されているはずなのでそのまま使用する。
  - 問題:Bitbucket 上のコミット履歴でマージがどのように示されているか確認しなさい。
- メイン開発者はプルして`talker.py`の競合していた箇所がどのようになったかを確認。

```shell
$ git pull origin master
```

## git 利用時のチーム開発手順まとめ

- 主開発者はリモートリポジトリを作成し、ローカルで作成した ROS パッケージの初版をプッシュする。
- サブ開発者はリモートリポジトリを`catkin_ws/src`にクローンする。
- 以降、毎回開発を始める前に他の人の変更をプルしてからコーディングを始める。
  - 補足:プルの際に競合を無視したい場合、次のコマンドでローカルで行った変更を全て破棄しリモートと完全に一致させることが可能。

```shell
$ git fetch origin
$ git reset --hard origin/master
```

- 競合が発生してしまったらメール等でコミュニケーションし、解決したコードをプッシュする。

## ブランチ(参考:実習は不要。問題は実施)

- 例えば現在ソフトウェアが安定して動作しているが新しい機能を追加したい、となったとき Git では「新しいブランチ」を作成する。
- 「ブランチ」とは現在のソースコードの編集履歴を残したまま、別の編集履歴を残して行く機能。
- git 利用の際デフォルトでは`master`ブランチが用意され、コミット履歴を残して行く。
- ここで新しい関数を作成し、十分にテストしてからソフトに追加したい場合、次のようにして`new_func`ブランチを作成し関数を実装する。

```shell
$ git branch new_func
$ git checkout new_func
```

- `master`と`new_func`間はいつでも`git checkout`で切り替えられる。
- 下記のコマンドを実行すればソースコードは`master`に戻る。

```shell
$ checkout master
```

- `new_func`ブランチでテストが終わったら以下のコマンドで`new_func`の内容を`master`にマージする。
  - 競合が起きたら解決する。

```shell
$ git checkout master
$ git merge new_func
```

- 世間一般の開発ではブランチを活用し、`master`には常に安定版のソフトがあるようにしている。
- 問題:[ROS の Github リポジトリ](https://github.com/ros/ros)が持つブランチ(Branch)を確認しなさい。

## リモートリポジトリを削除する

- 削除対象のリポジトリの「設定」 「リポジトリの削除」をクリック。
- 問題:間違えないように自分の作成したリモートリポジトリを削除しなさい。
  - 次いでローカルも削除しなさい。
- 補足:間違えてリモートリポジトリを削除してもローカルにクローンしたものがあれば再現可能。

---

[git/Home](Home.md)
