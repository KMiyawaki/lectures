# Unity (1)

[unity/Home](./Home.md)

- - -

## Unity のプロジェクト作成

- 「プロジェクト」の項目をクリックし、「新規作成」をクリックする。

![fig_01_01.svg.png](./unity_01/fig_01_01.svg.png)

- テンプレートは 3D を選択する。
- Project Name には`BallGame01`と入力する。
- Location はデフォルトで構わない。

![fig_02.svg.png](./unity_01/fig_02.svg.png)

- 起動中にアクセス許可の警告が出る場合があるので、許可する。

![fig_02_02.png](./unity_01/fig_02_02.png)

- アップデートするかどうかを聞いてくる場合があるが、 Skip してよい。 Check for Updates のチェックを外しておけば、起動時に聞かれることは無くなる。

![fig_02_03.png](./unity_01/fig_02_03.png)

## 初期設定

- 「 Edit 」メニューから「 Preferences 」を選択する。

![fig_02_04.png](./unity_01/fig_02_04.png)

- 「 Languages 」を選択して日本語にする。

![fig_02_05.png](./unity_01/fig_02_05.png)

- 「外部ツール (英語：External Tools) 」を選択して「外部スクリプトエディタ (英語：External Script Editor) 」を Visual Studio Code にする。
- 終了後、「 Preferences 」のウィンドウを閉じる。

![fig_02_06.png](./unity_01/fig_02_06.png)

- このままでは Unity での作業中に常に下図のようなエラーが出た状態になる。

![fig_02_07.png](./unity_01/fig_02_07.png)

- メニューの「ウィンドウ」から「 Package Manager 」を選択する。

![fig_02_08.png](./unity_01/fig_02_08.png)

- 下図を参照して、「プロジェクト内」を選択し、「 Visual Studio Code Editor 」を扱う Unity 側のソフトウェアのバージョンを「 1.1.3 」にダウングレードする。
- [Unity 公式サイトの Q and A](https://answers.unity.com/questions/1696108/vscode-index-out-of-range-on-new-project-unity-201.html)

![fig_02_09.svg.png](./unity_01/fig_02_09.svg.png)

- 設定が完了したら「 Package Manager 」を終了し、一旦 Unity を終了する。終了後先ほど作成したプロジェクト`BallGame01`が「プロジェクト」の項目にあるので、クリックして作業を再開する。

![fig_02_10.png](./unity_01/fig_02_10.png)

## 床を作る

- Unity の操作画面（以降： Unity Editor と表記する）で「シーン」とかかれたタブをクリックする。
- 3D ビュー上でマウスのホイールを押したままドラッグ、右ボタンドラッグ等の操作を行い、効果を確認すること。

![fig_03.svg.png](./unity_01/fig_03.svg.png)

- Unity Editor 左上の「＋」マークをクリックし「3D オブジェクト」->「平面」を選択する。

![fig_04.svg.png](./unity_01/fig_04.svg.png)

- Unity Editor 左方の`Plane`をクリックした後右の方の「インスペクター」をクリックし、「位置」を 0, 0, 0 とする。

![fig_05.svg.png](./unity_01/fig_05.svg.png)

## 球体を作る

- 平面と同じ要領で「スフィア」を選択して作成する。

![fig_06.png](./unity_01/fig_06.png)

- `Sphere`の「位置」を 0, 5, 0 にする。

![fig_07.svg.png](./unity_01/fig_07.svg.png)

- 「コンポーネントを追加」をクリックし、検索ボックスに「rigid」と入力すると「リジッドボディ」が出てくるので、クリックする。

![fig_08.svg.png](./unity_01/fig_08.svg.png)

- 「リジッドボディ」の「 Constraint 」にある「回転を固定」の X と Z にチェックをいれる。

![fig_08_04.png](./unity_01/fig_08_04.png)

## カメラを球体に追従させる

- `Main Camera`をクリックして「コンポーネントを追加」をクリックする。

![fig_09_01.svg.png](./unity_01/fig_09_01.svg.png)

- `SmoothFollow2`と入力すると「新しいスクリプト」と表示されるので、クリックする。

![fig_09_02.png](./unity_01/fig_09_02.png)

- スクリプト名が`SmoothFollow2`であることを確認し、「作成して追加」をクリックする。

![fig_09_03.png](./unity_01/fig_09_03.png)

- `SmoothFollow2`左の「▽」印をクリックし出てきた「スクリプト `SmoothFollow2`」をダブルクリックし、 Visual Studio Code でスクリプトを開く。

![fig_09_04.svg.png](./unity_01/fig_09_04.svg.png)

- 外部サイトにある`SmoothFollow2.cs`のオリジナル（[WEBリンク](https://raw.githubusercontent.com/jrf0110/unity-test-1/master/Assets/Scripts/SmoothFollow2.cs)）を WEB ブラウザで開き、テキスト全てを選択してコピーする。
- コピーしたテキストを Visual Studio Code で開いているスクリプトに上書き貼り付けする。つまり、もともとのスクリプトは全て消去し、完全に書き換える。
- 貼り付け後は保存し、 Visual Studio Code は終了する。

![fig_09_05.png](./unity_01/fig_09_05.png)

- Unity Editor の`SmoothFollow2`で「ターゲット」を`Sphere`にする。

![fig_10_01.svg.png](./unity_01/fig_10_01.svg.png)

## パソコンでプレビュー

- 再生ボタンを押す。カメラが落下する球体に追従するように動くはずである。
- 球体が落ちて止まったら、再生ボタンをもう一度押して、プレビューを終了する。

![fig_11.svg.png](./unity_01/fig_11.svg.png)

## シーンやプロジェクトを保存する

- `Ctrl`キー＋`S`キーを押す。または「ファイル」メニューから「保存」を選択して保存する。

**シーンやプロジェクトはこまめに保存しよう。**

- - -

[unity/Home](./Home.md)
